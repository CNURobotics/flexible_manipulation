#!/usr/bin/env python

import rospy

from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient

from actionlib_msgs.msg import GoalStatus

from moveit_msgs.msg import Constraints, MoveGroupAction, MoveGroupGoal, Constraints, JointConstraint, MoveItErrorCodes

from flexible_manipulation_flexbe_states.proxy import ProxyMoveItClient

'''
Created on 25-Jan-2018

@author: David Conner based on code by Alberto Romay
'''

class UserDataToMoveActionState(EventState):
    '''
    State to send joint configurations to MoveGroup move action.

    This state attempts a plan and execute move based on FlexBE user data;
    as such it requires user data to completely specify the command.

    This state uses the basic MoveGroupAction interface with minimal configuration.

    This class is independent of the ProxyMoveItClient.

    -- joint_tolerance          float               Joint tolerance (above and below) (default: 0.0 use srdf limits)

    -- constraint_weight        float               How much to weight joint constraint relative to other constraints

    -- allowed_planning_time    double              Allowable planning time (seconds)

    -- wait_duration            double              How long to wait for action server (seconds) (0.0 waits indefinitely)

    -- timeout                  double              How long to wait for response from action server after planning time (seconds) (0.0 waits indefinitely)

    -- action_topic             string              Topic on which MoveIt is listening for action calls. (None = user User data)

    ># action_topic             string              Topic on which MoveIt is listening for action calls.

    ># move_group               string              Name of the move group to be used for planning.

    ># joint_names              string[]            Names of the target joints.
                                                        Same order as their corresponding names in joint_values.

    ># joint_values             float[]             Target configuration of the joints.
                                                        Same order as their corresponding names in joint_names.

    #> status_text               string              Status message text regarding any error

    <= reached                                  Target joint configuration has been reached.
    <= param_error                              Invalid user data - cannot send plan request
    <= planning_failed                          Failed to find a plan to the given joint configuration.
    <= control_failed                           Failed to move the arm along the planned trajectory.
    <= failed                                   Failed to process goal

    '''

    def __init__(self,joint_tolerance=0.0, constraint_weight=1.0, allowed_planning_time=2.0, wait_duration=2.0, timeout=1.0, action_topic=None):
        '''
        Constructor
        '''
        super(UserDataToMoveActionState, self).__init__(
            input_keys=['move_group', 'action_topic', 'joint_names', 'joint_values'],
            output_keys=['status_text'],
            outcomes=['reached', 'param_error','planning_failed', 'control_failed', 'failed'] )

        self.client = None
        self.joint_tolerance = joint_tolerance
        self.constraint_weight = constraint_weight
        self.allowed_planning_time = allowed_planning_time
        self.wait_duration    = wait_duration
        self.timeout_duration = rospy.Duration(timeout)
        self.return_code = None
        self.status_text  = None

        self.given_action_topic = action_topic
        if (self.given_action_topic is not None) and (len(self.given_action_topic) > 0):
            # If topic is defined, set the client up on startup
            self.client = ProxyActionClient({self.given_action_topic: MoveGroupAction},
                                              self.wait_duration)
        else:
            self.given_action_topic = None # handle empty string

        self.current_action_topic = self.given_action_topic


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
                self.status_text  = 'Move Group success for group %s (error code - %s)' % (self.move_group, str(result.error_code))
                self.return_code = 'reached'
            elif result.error_code.val == MoveItErrorCodes.PLANNING_FAILED:
                self.status_text  = 'Move Group planning failed for group %s (error code - %s)' % (self.move_group, str(result.error_code))
                self.return_code = 'planning_failed'
            elif result.error_code.val == MoveItErrorCodes.INVALID_MOTION_PLAN:
                self.status_text  = 'Move Group invalid motion plan for group %s (error code - %s)' % (self.move_group, str(result.error_code))
                self.return_code = 'planning_failed'
            elif result.error_code.val == MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
                self.status_text  = 'Move Group invalid motion plan due to environment change for group %s (error code - %s)' % (self.move_group, str(result.error_code))
                self.return_code = 'control_failed'
            elif result.error_code.val == MoveItErrorCodes.CONTROL_FAILED:
                self.status_text  = 'Move Group control failed for move action for group %s (error code - %s)' % (self.move_group, str(result.error_code))
                self.return_code = 'control_failed'
            elif result.error_code.val == MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA:
                self.status_text  = 'Move Group cannot acquire sensor data for group %s (error code - %s)' % (self.move_group, str(result.error_code))
                self.return_code = 'planning_failed'
            elif result.error_code.val == MoveItErrorCodes.TIMED_OUT:
                self.status_text  = 'Move Group timeout for group %s (error code - %s)' % (self.move_group, str(result.error_code))
                self.return_code = 'planning_failed'
            elif result.error_code.val == MoveItErrorCodes.PREEMPTED:
                self.status_text  = 'Move Group preempted for group %s (error code - %s)' % (self.move_group, str(result.error_code))
                self.return_code = 'control_failed'
            elif result.error_code.val == MoveItErrorCodes.START_STATE_IN_COLLISION:
                self.status_text  = 'Move Group start state in collision for group %s (error code - %s)' % (self.move_group, str(result.error_code))
                self.return_code = 'planning_failed'
            elif result.error_code.val == MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS:
                self.status_text  = 'Move Group start state violates path constraints for group %s (error code - %s)' % (self.move_group, str(result.error_code))
                self.return_code = 'planning_failed'
            elif ( (result.error_code.val >= MoveItErrorCodes.INVALID_GROUP_NAME) and \
                   (result.error_code.val <= MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS) ):
                self.status_text  = 'Move Group invalid request data for group %s (error code - %s)' % (self.move_group, str(result.error_code))
                self.return_code = 'planning_failed'
            elif ( (result.error_code.val >= MoveItErrorCodes.SENSOR_INFO_STALE) and \
                   (result.error_code.val <= MoveItErrorCodes.FRAME_TRANSFORM_FAILURE) ):
                self.status_text  = 'Move Group - system data failure for group %s (error code - %s)' % (self.move_group, str(result.error_code))
                self.return_code = 'planning_failed'
            elif result.error_code.val == MoveItErrorCodes.NO_IK_SOLUTION:
                self.status_text  = 'Move Group no IK solution for group %s (error code - %s)' % (self.move_group, str(result.error_code))
                self.return_code = 'planning_failed'
            elif result.error_code.val == MoveItErrorCodes.FAILURE:
                self.status_text  = 'Move Group unknown failure for group %s (error code - %s)' % (self.move_group, str(result.error_code))
                self.return_code = 'planning_failed'
            else:
                self.status_text  = 'Move Group - failed with unknown error code for group %s (error code - %s)' % (self.move_group, str(result.error_code))
                self.return_code = 'planning_failed'

            # Handle any result outcome
            Logger.loginfo(self.status_text)
            userdata.status_text = self.status_text
            return self.return_code

        elif self.client.get_state(self.current_action_topic) == GoalStatus.ABORTED:
            # No result returned is returned for this action, so go by the client state
            self.status_text  = "Move Group - %s request aborted by action server" % (self.name)
            self.return_code = 'failed'
            Logger.loginfo(self.status_text)
            userdata.status_text = self.status_text
            return self.return_code
        elif self.client.get_state(self.current_action_topic) == GoalStatus.REJECTED:
            # No result returned is returned for this action, so go by the client state
            self.status_text  = "Move Group - %s request rejected by action server" % (self.name)
            self.return_code = 'failed'
            Logger.loginfo(self.status_text)
            userdata.status_text = self.status_text
            return self.return_code
        elif self.timeout_target is not None and rospy.Time.now() > self.timeout_target:
            self.status_text  = "Move Group - timeout waiting for %s to (%f > %f (%f))" % (self.name, rospy.Time.now().to_sec(), self.timeout_target.to_sec(), self.timeout_duration.to_sec())
            self.return_code = 'failed'
            Logger.loginfo(self.status_text)
            userdata.status_text = self.status_text
            return self.return_code

        # Normal monitoring


    def on_enter(self, userdata):
        self.return_code = None
        self.status_text = None
        self.current_action_topic = self.given_action_topic

        # Retrieve the relevant data
        try :
            if (self.given_action_topic is None):
                self.current_action_topic    = userdata.action_topic

            self.move_group      = userdata.move_group
            self.joint_values    = userdata.joint_values
            self.joint_names     = userdata.joint_names
            if (len(self.joint_values) != len(self.joint_names)):
                self.status_text = 'Mismatch in joint names and values (%d vs. %d) -' % (len(self.joint_values), len(self.joint_names))
                self.return_code = 'param_error'
                Logger.logerr(self.status_text)
                return

        except Exception as e:
            self.status_text = 'Failed to get relevant user data -\n%s' % (str(e))
            self.return_code = 'param_error'
            Logger.logerr(self.status_text)
            return

        try:

            if (self.client is None):
                self.client = ProxyActionClient({self.current_action_topic: MoveGroupAction},
                                                  self.wait_duration)

            if not self.client.is_available(self.current_action_topic):
                self.client.setupClient(self.current_action_topic, MoveGroupAction, self.wait_duration)
                if not self.client.is_available(self.current_action_topic):
                    self.status_text = 'Action client is not available for %s' % (self.current_action_topic)
                    self.return_code = 'param_error'
                    Logger.logerr(self.status_text)
                    return

        except Exception as e:
            self.status_text = 'Failed to set up the action client for %s\n%s' % (self.current_action_topic, str(e))
            self.return_code = 'param_error'
            Logger.logerr(self.status_text)
            return

        try:

            # Action Initialization
            action_goal = MoveGroupGoal()
            action_goal.request.group_name = self.move_group
            action_goal.request.allowed_planning_time = self.allowed_planning_time
            action_goal.request.start_state.is_diff = True # Flags start state as empty to use current state on server
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

            if (self.timeout_duration > rospy.Duration(0.0)):
                self.timeout_target = rospy.Time.now() + self.timeout_duration + rospy.Duration(self.allowed_planning_time)
            else:
                self.timeout_target = None

            self.client.send_goal(self.current_action_topic, action_goal)
        except Exception as e:
            self.status_text = 'Failed to send action goal for group - %s\n%s' % (self.move_group, str(e))
            self.return_code = 'planning_failed'
            Logger.logerr(self.status_text)
            return

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
