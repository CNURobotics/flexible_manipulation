#!/usr/bin/env python

import rospy

from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient

from actionlib_msgs.msg import GoalStatus

from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, Constraints, JointConstraint, MoveItErrorCodes

from flexible_manipulation_flexbe_states.proxy import ProxyMoveItClient

'''
Created on 25-Jan-2018

@author: David Conner based on code by Alberto Romay
'''

class JointValuesToMoveActionState(EventState):
    '''
    State to send a specified joint configuration to a specified MoveGroup move action.

    This state attempts a plan and execute move to a specified move group action
    interface using joint names and values specified in parameters;
    the number of joint values must match the given joint names, and the joint
    names should be consistent with the specified move group

    This state uses the basic MoveGroupAction interface with minimal configuration.

    -- action_topic             string              Topic on which MoveIt is listening for action calls.
                                                            (default: None - use userdata )

    -- move_group               string              Name of the move group to be used for planning.
                                                            (default: None - use userdata )

    -- joint_names              string[]            List of joint names
                                                      Does not need to set all joints in group.
                                                        Same order as their corresponding names in joint_values.

    -- joint_values             float[]            List of joint values
                                                        Same order as their corresponding names in joint_names.

    -- joint_tolerance          float               Joint tolerance (above and below) (default: 0.0 use srdf limits)

    -- constraint_weight        float               How much to weight joint constraint relative to other constraints

    -- allowed_planning_time    float               Allowable planning time (seconds)

    -- wait_duration            float               How long to wait for action server (seconds)
                                                      (default: 0 seconds)

    -- timeout                  float               How long to wait for execution completion after planning
                                                      (default: 10 seconds)

    ># action_topic             string              Topic on which MoveIt is listening for action calls.
    ># move_group               string              Name of the move group to be used for planning.

    Pass the configuration data along for potential re-use later
    #> move_group               string              Name of the move group to be used for planning.

    #> action_topic             string              Topic on which MoveIt is listening for action calls.

    #> joint_names              string[]            Names of the target joints.

    #> joint_values             float[]             Target configuration of the joints.

    <= reached                                      Target joint configuration has been reached.
    <= param_error                                  Invalid user data - cannot send plan request
    <= planning_failed                              Failed to find a plan to the given joint configuration.
    <= control_failed                               Failed to move the arm along the planned trajectory.
    <= failed                                       Failed to send goal

    '''

    def __init__(self,joint_names, joint_values, move_group=None, action_topic=None,
                 joint_tolerance=0.0, constraint_weight=1.0,
                 allowed_planning_time=2.0, wait_duration=0, timeout = 10.0):
        '''
        Constructor
        '''
        super(JointValuesToMoveActionState, self).__init__(
            outcomes=['reached', 'param_error', 'planning_failed', 'control_failed', 'failed'],
            input_keys=['move_group', 'action_topic'],
            output_keys=['status_text', 'move_group', 'action_topic','joint_names', 'joint_values'])

        self.client       = None
        self.given_move_group   = move_group
        self.given_action_topic = action_topic
        self.joint_names  = joint_names
        self.joint_values = joint_values

        self.joint_tolerance = joint_tolerance
        self.constraint_weight = constraint_weight
        self.allowed_planning_time = allowed_planning_time
        self.wait_duration = wait_duration
        self.return_code = None
        self.status_text = None
        self.timeout_duration   = rospy.Duration(timeout)
        self.timeout_target     = None

        if (self.given_action_topic is not None) and (len(self.given_action_topic) > 0):
            self.client = ProxyActionClient({self.given_action_topic: MoveGroupAction},
                                              self.wait_duration)
        else:
            self.given_action_topic = None
        if (self.given_move_group is None) or (len(self.given_move_group) < 1):
            self.given_move_group = None

        self.current_action_topic = self.given_action_topic
        self.current_move_group = self.given_move_group

    def execute(self, userdata):
        '''
        Execute this state
        '''
        if self.return_code is not None:
            userdata.status_text = self.status_text
            return self.return_code

        if self.client.has_result(self.current_action_topic):
            result = self.client.get_result(self.current_action_topic)

            # Logger.loginfo('Move Group  - %s' % str(self.client.get_goal_status_text(self.current_action_topic)))

            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                self.status_text = 'Move Group success!  - %s' % str(result.error_code)
                self.return_code = 'reached'
            elif result.error_code.val == MoveItErrorCodes.PLANNING_FAILED:
                self.status_text = 'Move Group planning failed with result error code - %s' % str(result.error_code)
                self.return_code = 'planning_failed'
            elif result.error_code.val == MoveItErrorCodes.INVALID_MOTION_PLAN:
                self.status_text = 'Move Group invalid motion plan with result error code - %s' % str(result.error_code)
                self.return_code = 'planning_failed'
            elif result.error_code.val == MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
                self.status_text = 'Move Group invalid motion plan due to environment change with result error code - %s' % str(result.error_code)
                self.return_code = 'control_failed'
            elif result.error_code.val == MoveItErrorCodes.CONTROL_FAILED:
                self.status_text = 'Move Group control failed for move action of group - %s (error code - %s)' % (self.current_move_group, str(result.error_code))
                self.return_code = 'control_failed'
            elif result.error_code.val == MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA:
                self.status_text = 'Move Group cannot acquire sensor data with result error code - %s' % str(result.error_code)
                self.return_code = 'planning_failed'
            elif result.error_code.val == MoveItErrorCodes.TIMED_OUT:
                self.status_text = 'Move Group timeout with result error code - %s' % str(result.error_code)
                self.return_code = 'control_failed'
            elif result.error_code.val == MoveItErrorCodes.PREEMPTED:
                self.status_text = 'Move Group preempted with result error code - %s' % str(result.error_code)
                self.return_code = 'control_failed'
            elif result.error_code.val == MoveItErrorCodes.START_STATE_IN_COLLISION:
                self.status_text = 'Move Group start state in collision with result error code - %s' % str(result.error_code)
                self.return_code = 'planning_failed'
            elif result.error_code.val == MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS:
                self.status_text = 'Move Group start state violates path constraints with result error code - %s' % str(result.error_code)
                self.return_code = 'planning_failed'
            elif ( (result.error_code.val >= MoveItErrorCodes.INVALID_GROUP_NAME) and \
                   (result.error_code.val <= MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS) ):
                self.status_text = 'Move Group invalid request data with result error code - %s' % str(result.error_code)
                self.return_code = 'planning_failed'
            elif ( (result.error_code.val >= MoveItErrorCodes.SENSOR_INFO_STALE) and \
                   (result.error_code.val <= MoveItErrorCodes.FRAME_TRANSFORM_FAILURE) ):
                self.status_text = 'Move Group - system data failure result error code - %s' % str(result.error_code)
                self.return_code = 'planning_failed'
            elif result.error_code.val == MoveItErrorCodes.NO_IK_SOLUTION:
                self.status_text = 'Move Group no IK solution with result error code - %s' % str(result.error_code)
                self.return_code = 'planning_failed'
            elif result.error_code.val == MoveItErrorCodes.FAILURE:
                self.status_text = 'Move Group unknown failure with result error code - %s' % str(result.error_code)
                self.return_code = 'planning_failed'
            else:
                self.status_text = 'Move Group - failed with unknown error code   - %s' % str(result.error_code)
                self.return_code = 'planning_failed'

            Logger.loginfo(self.status_text)
            userdata.status_text = self.status_text
            return self.return_code

        elif self.client.get_state(self.current_action_topic) == GoalStatus.ABORTED:
            # No result returned is returned for this action, so go by the client state
            self.status_text = "MoveGroup- %s request aborted by action server" % (self.name)
            self.return_code = 'failed'
        elif self.client.get_state(self.current_action_topic) == GoalStatus.REJECTED:
            # No result returned is returned for this action, so go by the client state
            self.status_text = "MoveGroup - %s request rejected by action server" % (self.name)
            self.return_code = 'failed'
        elif (self.timeout_target is not None) and (rospy.Time.now() > self.timeout_target):
            self.status_text = "MoveGroup - timeout waiting for %s response (%f > %f (%f))" % (self.name, rospy.Time.now().to_sec(), self.timeout_target.to_sec(), self.timeout_duration.to_sec())
            self.return_code = 'failed'

        #Logger.logwarn("ExecuteKnownTrajectory - %s (%f > %f (%f))" % (self.name, rospy.Time.now().to_sec(), self.timeout_target.to_sec(), self.timeout_duration.to_sec()))
        if (self.return_code is not None):
            Logger.logwarn( self.status_text )
            userdata.status_text = self.status_text
            return self.return_code

    def on_enter(self, userdata):
        self.return_code = None

        # Retrieve the relevant data
        try :
            if (len(self.joint_values) != len(self.joint_names)):
                Logger.logwarn('Mismatch in joint names and values (%d vs. %d) -' % (len(self.joint_values), len(self.joint_names)))
                self.return_code = 'param_error'
                return
        except Exception as e:
            Logger.logwarn('Failed to get relevant user data -\n%s' % (str(e)))
            self.return_code = 'param_error'
            return

        try:

            if (self.given_move_group is None):
                self.current_move_group = userdata.move_group

            if (self.given_action_topic is None):
                self.current_action_topic = userdata.action_topic

            if (self.client is None):
                self.client = ProxyActionClient({self.current_action_topic: MoveGroupAction},
                                                  self.wait_duration)

            if (not self.client.is_available(self.current_action_topic)):
                Logger.logwarn('Action client is not available for %s' % (self.current_action_topic) )
                self.return_code = 'param_error'
                return

        except Exception as e:
            Logger.logwarn('Failed to set up the action client for %s\n%s' % (self.current_action_topic, str(e)))
            self.return_code = 'param_error'
            return

        try:
            # Action Initialization
            action_goal = MoveGroupGoal()
            action_goal.request.start_state.is_diff = True # Flags start state as empty to use current state on server
            action_goal.request.group_name = self.current_move_group
            action_goal.request.allowed_planning_time = self.allowed_planning_time
            action_goal.planning_options.planning_scene_diff.robot_state.is_diff = True # Flags start state as empty to use current state on server

            goal_constraints = Constraints()
            for i in range(len(self.joint_names)):
                    goal_constraints.joint_constraints.append(JointConstraint(
                            joint_name=self.joint_names[i],
                            position=self.joint_values[i],
                            tolerance_above=self.joint_tolerance,
                            tolerance_below=self.joint_tolerance,
                            weight=self.constraint_weight))
            action_goal.request.goal_constraints.append(goal_constraints)

            self.client.send_goal(self.current_action_topic, action_goal)
            if (self.timeout_duration > rospy.Duration(0.0)):
                self.timeout_target = rospy.Time.now() + rospy.Duration(self.allowed_planning_time) + self.timeout_duration
            else:
                self.timeout_target = None

            # Pass the configuration data along as user data
            userdata.move_group   = self.current_move_group
            userdata.action_topic = self.current_action_topic
            userdata.joint_names  = self.joint_names
            userdata.joint_values = self.joint_values

        except Exception as e:
            Logger.logwarn('Failed to send action goal for group - %s\n%s' % (self.current_move_group, str(e)))
            self.return_code = 'planning_failed'

    def on_stop(self):
            try:
                if ( self.client.is_available(self.current_action_topic) \
                     and not self.client.has_result(self.current_action_topic) ):
                    # Cancel any active goals
                    self.client.cancel(self.current_action_topic)
            except Exception as e:
                # client already closed
                Logger.logwarn('Action client already closed - %s\n%s' % (self.current_action_topic, str(e)))

    def on_pause(self):
        try:
            self.client.cancel(self.current_action_topic)
        except Exception as e:
            # client already closed
            Logger.logwarn('Action client already closed - %s\n%s' % (self.current_action_topic, str(e)))

    def on_resume(self, userdata):
            self.on_enter(userdata)
