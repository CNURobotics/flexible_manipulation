# Extended planning options provide additional and convenience options
# beyond what the standard MoveIt MoveGroup Action provides.

# Types of motion supported. See below for further explanation
uint8 TYPE_FREE_MOTION=1
uint8 TYPE_CARTESIAN_WAYPOINTS=2
uint8 TYPE_CIRCULAR_MOTION=3

# Specify the desired motion type
uint8 target_motion_type

# Specify frame in which target poses below are specified
string target_frame

# Specify the desired waypoints.
# If this pose array is empty, the Action is interpreted in the standard
# MoveIt! way without consideration of extended planning options.
#
# The following semantics apply:
# TYPE_FREE_MOTION: Single target pose
# TYPE_CARTESIAN_WAYPOINTS: Waypoints in arbitrary frame
# TYPE_CIRCULAR_MOTION: Rotation center
geometry_msgs/Pose[] target_poses

# Specify frame in which reference point below is specified
# If empty, use standard group end frame
string reference_point_frame

# Specify the desired reference point
# If this contains a invalid quaternion, the reference point is
# ignored. Otherwise, the target poses for the endeffector frame
# are transformed according to this reference point.
geometry_msgs/Pose reference_point

# Specify if robot should continuously replan motion plan
# Currently this does nothing
bool continuous_replanning

# Specify if collisions with the environment should be avoided
# Self collisions are always avoided per default and this
# does not affect self collision checking
bool allow_environment_collisions

# Determines if incomplete cartesian plans should be executed
# (Happens if target moves out of reach). Defaults to false.
bool execute_incomplete_cartesian_plans

# Planning scene specific settings below
#----------------------------------------------

# These settings are applied as a diff to the planning scene after
# the planning scene diff from the standard planning_options is
# applied.
ExtendedPlanningSceneDiff extended_planning_scene_diff

# Circular motion settings below
#----------------------------------------------

# Used for specifying circular motion rotation angle
# Can be positive or negative
float32 rotation_angle

# Keep endeffector orientation if true (do not rotate around rotation_center_pose)
bool keep_endeffector_orientation

# Used for specifying circular motion with translation
# Can be positive or negative
float32 pitch
