# These are constraints for the torso joints
# See message def for specification of allowed values
JointPositionConstraint[] joint_position_constraints

# True if collision avoidance with environment should be disabled
bool disable_collision_avoidance

# True if collision avoidance of end effector (hand) with environment should be disabled
bool disable_end_effector_collision_avoidance

# The amount of padding (growing) to be added to robot
# links for collision checking
float32 robot_collision_padding

# The time factor to apply to the generated trajectories
#  (e.g. 0.5 makes trajectory take twice as long to execute)
float32 trajectory_time_factor

# The maximum height for octomaps to be used
float32 octomap_max_height

# Size of the goal cube that gets cleared
float32 goal_cube_clearance

# The planner to use (if empty, default planner is used)
string planner_id
