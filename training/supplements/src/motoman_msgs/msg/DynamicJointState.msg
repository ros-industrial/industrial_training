#group[]: # length of this array must match num_groups
#    id:   control-group ID for use on-controller
#    num_joints: # of joints in this motion group
#    valid_fields: #bit field for following items
#    # length of the following items must match num_joints, order set by controller.  Invalid fields (see bit field above) are not included, resulting in a shorter message.
#    positions[]
#    velocities[]
#    accelerations[]
#    effort[]
#    position_desired[]
#    position_errors[]
#    velocity_desired[]
#    velocity_errors[]
#    effort_desired[]
#    effort_error[]

int16 group_number
int16 num_joints
int16 valid_fields
float64[] positions
float64[] velocities
float64[] accelerations
float64[] effort
float64[] positions_desired
float64[] positions_errors
float64[] velocities_desired
float64[] velocities_errors
float64[] accelerations_desired
float64[] accelerations_errors
float64[] effort_errors
float64[] effort_desired
