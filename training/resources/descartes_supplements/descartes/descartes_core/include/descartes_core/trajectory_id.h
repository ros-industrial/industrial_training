#ifndef TRAJECTORY_ID_H
#define TRAJECTORY_ID_H

#include <iostream>

#include <boost/thread/mutex.hpp>

namespace descartes_core
{
namespace detail
{
/**
 * @brief Unimplemented base for IdGenerator. Users should specialize
 *        this struct for the base ID type. It represents a concept
 *        that defines the following types:
 *        1. value_type typedef representing the type of unique state object
 *        2. value_type make_nil() function that returns a nil sentinel state object
 *        3. value_type make_id() function that returns a unique state object
 *        4. bool is_nil(value_type) function that tests if an object is the sentinel
 */
template <typename T>
struct IdGenerator;

/**
 * @brief This specialization of the id generator uses a 64 bit unsigned integer
 *        for the unique 'state'. Zero is reserved as a special value
 */
template <>
struct IdGenerator<uint64_t>
{
  typedef uint64_t value_type;

  static value_type make_nil()
  {
    return 0;
  }

  static value_type make_id()
  {
    boost::unique_lock<boost::mutex> scoped_lock(counter_mutex_);
    return counter_++;
  }

  static bool is_nil(value_type id)
  {
    return id == 0;
  }

private:
  // Initialized to 1
  static value_type counter_;
  static boost::mutex counter_mutex_;
};
}

/**
 * @brief TrajectoryID_ represents a unique id to be associated with a TrajectoryPt
 *
 */
template <typename T>
class TrajectoryID_
{
public:
  typedef T value_type;

  /**
   * @brief Constructor for generating a trajectory id using the given state object
   */
  TrajectoryID_(value_type id) : id_(id)
  {
  }

  /**
   * @brief Constructor for generating a trajectory with a default id. Here we default
   *        the points to nil. Default constructor is needed for STL containers.
   */
  TrajectoryID_() : id_(detail::IdGenerator<value_type>::make_nil())
  {
  }

  /**
   * @brief Tests if this ID is nil
   */
  bool is_nil() const
  {
    return detail::IdGenerator<value_type>::is_nil(id_);
  }

  /**
   * @brief Retrieves the value of the underlying id state object
   */
  value_type value() const
  {
    return id_;
  }

  /**
   * @brief Factory function to generate a trajectory ID with a unique state object
   */
  static TrajectoryID_<value_type> make_id()
  {
    return TrajectoryID_<value_type>(detail::IdGenerator<value_type>::make_id());
  }

  /**
   * @brief Factory function to generate a trajectory ID with a nil state object
   */
  static TrajectoryID_<value_type> make_nil()
  {
    return TrajectoryID_<value_type>(detail::IdGenerator<value_type>::make_nil());
  }

private:
  value_type id_;
};

//////////////////////
// Helper Functions //
//////////////////////

template <typename T>
inline bool operator==(TrajectoryID_<T> lhs, TrajectoryID_<T> rhs)
{
  return lhs.value() == rhs.value();
}

template <typename T>
inline bool operator!=(TrajectoryID_<T> lhs, TrajectoryID_<T> rhs)
{
  return !(lhs == rhs);
}

template <typename T>
inline bool operator<(TrajectoryID_<T> lhs, TrajectoryID_<T> rhs)
{
  return lhs.value() < rhs.value();
}

template <typename T>
inline std::ostream& operator<<(std::ostream& os, TrajectoryID_<T> id)
{
  os << "ID" << id.value();
  return os;
}

typedef TrajectoryID_<uint64_t> TrajectoryID;

}  // end namespace descartes_core

#endif
