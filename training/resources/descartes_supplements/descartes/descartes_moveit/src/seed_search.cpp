#include <descartes_moveit/seed_search.h>

#include <ros/ros.h>

using namespace descartes_moveit;

typedef std::vector<unsigned> BijectionVec;
typedef std::pair<unsigned, unsigned> JointPair;
typedef std::vector<JointPair> JointPairVec;
typedef std::vector<double> JointConfig;
typedef std::vector<JointConfig> JointConfigVec;

/**
 * @brief Forward kinematics helper
 */
bool doFK(moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, const std::string& tool,
          const JointConfig& joint_pose, Eigen::Affine3d& result)
{
  state.setJointGroupPositions(group, joint_pose);
  if (!state.knowsFrameTransform(tool))
  {
    ROS_WARN("No transform to this tool frame");
    return false;
  }

  if (!state.satisfiesBounds())
  {
    ROS_WARN("Joint angles do not satisfy robot bounds");
    return false;
  }

  result = state.getFrameTransform(tool);
  return true;
}

/**
 * @brief Inverse kinematics helper. Returns the solution in the result parameter.
 *        May return false if IK failed.
 */
bool doIK(moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, const std::string& group_name,
          const std::string& tool, const Eigen::Affine3d& pose, const JointConfig& seed, JointConfig& result)
{
  const static int N_ATTEMPTS = 1;
  const static double IK_TIMEOUT = 0.01;

  state.setJointGroupPositions(group_name, seed);
  if (!state.setFromIK(group, pose, tool, N_ATTEMPTS, IK_TIMEOUT))
  {
    return false;
  }
  state.copyJointGroupPositions(group, result);
  return true;
}

/**
 * @brief Returns true if the determinant of the jacobian is near zero.
 */
bool isSingularity(moveit::core::RobotState& state, const moveit::core::JointModelGroup* group)
{
  const static double MIN_DETERMINANT_VALUE = 0.0001;
  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd jacobian;
  state.getJacobian(group, state.getLinkModel(group->getLinkModelNames().back()), reference_point_position, jacobian);

  return std::abs(jacobian.determinant()) < MIN_DETERMINANT_VALUE;
}

// Compares the first n_compare bijection joint values from a and b and tests
// whether they are within 45 degrees of one another
inline bool isSameJointConfig(const JointConfig& a, const JointConfig& b, const JointPair& pair)
{
  const static double MIN_DIFF = M_PI_4;
  return std::abs(a[pair.first] - b[pair.first]) < MIN_DIFF && std::abs(a[pair.second] - b[pair.second]) < MIN_DIFF;
}

// Tests whether a given joint configuration, c, is similar to any joint set already
// in set using the given bijection values and number of joints.
inline bool isInJointSet(const JointConfig& c, const JointConfigVec& set, const JointPair& pair)
{
  for (std::size_t i = 0; i < set.size(); ++i)
  {
    if (isSameJointConfig(c, set[i], pair))
      return true;
  }
  return false;
}

std::vector<double> createValidJointPositions(const moveit::core::JointModel& model, double increment)
{
  // Should never be called with fixed joint
  const moveit::core::JointModel::Bounds& bounds = model.getVariableBounds();
  double min = bounds[0].min_position_;
  double max = bounds[0].max_position_;

  std::vector<double> result;
  while (min <= max)
  {
    result.push_back(min);
    min += increment;
  }

  return result;
}

std::vector<double> createSeedFromPerms(const std::vector<double>& initial, const std::vector<double>& a_perms,
                                        unsigned a_joint_idx, const std::vector<double>& b_perms, unsigned b_joint_idx,
                                        unsigned n)
{
  std::vector<double> seed = initial;
  double a_value = a_perms[n / b_perms.size()];
  double b_value = b_perms[n % b_perms.size()];
  seed[a_joint_idx] = a_value;
  seed[b_joint_idx] = b_value;
  return seed;
}

JointConfigVec findSeedStatesForPair(moveit::core::RobotState& state, const std::string& group_name,
                                     const std::string& tool_frame, const JointPair& pair)
{
  using namespace moveit::core;

  const JointModelGroup* group = state.getJointModelGroup(group_name);
  const std::vector<const JointModel*> active_joints = group->getActiveJointModels();

  // check each joint to see if it is revolute
  for (const JointModel* model : active_joints)
  {
    if (model->getType() != JointModel::REVOLUTE)
      ROS_WARN_STREAM("Joint '" << model->getName() << "' does not appear to be revolute");
  }

  // compute random starting values for all joints
  state.setToRandomPositions();
  JointConfig init_state;
  state.copyJointGroupPositions(group, init_state);

  // Precompute the valid positions for the two joints we'll iterate over
  std::vector<double> joint1_perms = createValidJointPositions(*active_joints[pair.first], M_PI_2);
  std::vector<double> joint2_perms = createValidJointPositions(*active_joints[pair.second], M_PI_2);

  std::set<size_t> final_seed_states;

  // Walk the valid combos
  const size_t iterations = joint1_perms.size() * joint2_perms.size();
  for (std::size_t i = 0; i < iterations; ++i)
  {
    // Make the seed state for this index from permutations
    JointConfig round_ik = createSeedFromPerms(init_state, joint1_perms, pair.first, joint2_perms, pair.second, i);
    // Containers for the valid seeds and ik solutions for THIS round
    std::vector<size_t> this_round_seeds;
    JointConfigVec this_round_iks;

    // Forward kinematics
    Eigen::Affine3d target_pose;
    if (!doFK(state, group, tool_frame, round_ik, target_pose))
    {
      ROS_DEBUG_STREAM("No FK for pose " << i);
      continue;
    }

    // Check to make sure we're not in a singularity
    if (isSingularity(state, group))
    {
      ROS_DEBUG_STREAM("Pose " << i << " at singularity.");
      continue;
    }

    // Add initial states to the iks/seeds for this point
    this_round_seeds.push_back(i);
    this_round_iks.push_back(round_ik);

    // Now we'll walk through all of the other seeds, starting with the ones that have worked so far
    for (size_t idx : final_seed_states)
    {
      // create joint config fom handle
      JointConfig seed = createSeedFromPerms(init_state, joint1_perms, pair.first, joint2_perms, pair.second, idx);
      JointConfig ik;
      // perform ik
      if (!doIK(state, group, group_name, tool_frame, target_pose, seed, ik))
      {
        continue;
      }
      // If we have a unique IK solution, then add to the iks seen this round
      if (!isInJointSet(ik, this_round_iks, pair))
      {
        this_round_iks.push_back(ik);
        this_round_seeds.push_back(idx);
      }
    }

    // Now try the rest of the joint possible seed states if they haven't been tried
    for (std::size_t j = 0; j < iterations; ++j)
    {
      // skip the situation where pose is generated from current seed we already have that ik
      if (i == j)
        continue;

      unsigned count = final_seed_states.count(j);
      if (count != 0)
        continue;

      JointConfig seed = createSeedFromPerms(init_state, joint1_perms, pair.first, joint2_perms, pair.second, j);
      JointConfig ik;
      if (!doIK(state, group, group_name, tool_frame, target_pose, seed, ik))
      {
        continue;
      }

      // If we have a new IK solution here, then it was generated
      // by a seed that has not yet been added to the overall seed
      // states to be returned
      if (!isInJointSet(ik, this_round_iks, pair))
      {
        this_round_iks.push_back(ik);
        this_round_seeds.push_back(j);
        final_seed_states.insert(j);
      }
    }

    ROS_DEBUG_STREAM("Calculated " << this_round_iks.size() << " unique IK states this round");
  }  // outer loop end

  // Consolidate set into vector for the result
  JointConfigVec result;
  result.reserve(final_seed_states.size());
  for (size_t idx : final_seed_states)
  {
    result.push_back(createSeedFromPerms(init_state, joint1_perms, pair.first, joint2_perms, pair.second, idx));
  }
  return result;
}

JointConfigVec seed::findSeedStatesByPairs(moveit::core::RobotState& state, const std::string& group_name,
                                           const std::string& tool_frame, const JointPairVec& pairs)
{
  JointConfigVec result;
  for (const auto& pair : pairs)
  {
    JointConfigVec partial_answer = findSeedStatesForPair(state, group_name, tool_frame, pair);
    result.insert(result.end(), partial_answer.begin(), partial_answer.end());
  }
  return result;
}

JointConfigVec seed::findRandomSeeds(moveit::core::RobotState& state, const std::string& group_name, unsigned n)
{
  auto group = state.getJointModelGroup(group_name);

  JointConfigVec result;

  for (unsigned i = 0; i < n; ++i)
  {
    state.setToRandomPositions();

    JointConfig c;
    state.copyJointGroupPositions(group, c);

    result.push_back(c);
  }
  return result;
}
