# DynamicJointPoint
#group: # length of this array must match num_groups
#    id:   control-group ID for use on-controller
#    num_joints: # of joints in this motion group
#    valid_fields: #bit field for following items
#    # length of the following items must match num_joints, order set by controller.  Invalid fields (see bit field above) are not included, resulting in a shorter message.
#    positions[]
#    velocities[]
#    accelerations[]
#    effort[]
#    time_from_start

int16 num_groups
DynamicJointsGroup[] groups
