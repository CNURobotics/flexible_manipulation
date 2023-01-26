#!/usr/bin/env python

import rospy

from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient

from actionlib_msgs.msg import GoalStatus

from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, MoveItErrorCodes, MotionPlanRequest, PlanningOptions, RobotState, RobotTrajectory

from flexible_manipulation_flexbe_states.proxy import ProxyMoveItClient

'''
Created on 12-Mar-2018

@author: David Conner, Julie Gates, Jenny Gu
'''

class MoveGroupState(EventState):
    '''
    State using MoveGroupAction.

    This state takes joint names and values from user data, and creates a move request
    to plan and execute using the ProxyMoveItClient.

    This state presumes the associated action topic is defined in the ProxyMoveItClient for each move group

    -- timeout              double           How long to wait for execution completion (must include execution time)
                                                (default: 30 seconds)

    -- wait_duration        double           How long to wait for action server (seconds) (0.0 waits indefinitely)

    -- move_group           string           Move group name
                                                (default: None --> Use user data)

    ># move_group           string           Name of the move group to be used for planning (if parameter is None )

    ># joint_names          string[]         Names of the target joints.
                                                Same order as their corresponding names in joint_values

    ># joint_values         float[]          Target configuration of the joints.
                                                Same order as their corresponding names in joint_names

    #> status_text          string           An error code reflecting what went wrong

    #> planned_trajectory   RobotTrajectory  The trajectory that moved group produced for execution

    #> executed_trajectory  RobotTrajectory  The trace of the trajectory recorded during execution

    <= reached                               Successful
    <= control_failed                        Planned trajectory, but failed to complete execution
    <= planning_failed                       Failed to find a plan to the given joint configuration.
    <= param_error                           Invalid user data - cannot send plan request
    <= failed                                Failed to process goal


    '''

    def __init__(self, timeout=30.0, wait_duration=0.01, move_group=None):
        '''
        Constructor
        '''
        super(MoveGroupState, self).__init__(
            input_keys=['move_group','joint_names','joint_values'],
            outcomes=['reached', 'control_failed', 'planning_failed', 'param_error', 'failed'],
            output_keys=['status_text','planned_trajectory','executed_trajectory'] )

        self.moveit_client = None
        self.status_text   = None
        self.timeout_duration = None
        if (timeout > 0.0):
            self.timeout_duration = rospy.Duration(timeout)

        self.wait_duration = wait_duration

        try:
            self.moveit_client = ProxyMoveItClient(None)
        except Exception as e:
            Logger.logerr(" %s  -  exception on initialization of ProxyMoveItClient \n%s"% (self.name, str(e)))


        self.given_move_group   = move_group
        if (move_group is not None) and (len(move_group) < 1):
            self.given_move_group = None # handle empty string

        self.current_move_group = self.given_move_group
        self.current_action_topic = None
        self.status_text = None
        self.return_code = None
        self.planned_trajectory=None
        self.executed_trajectory=None

    def execute(self, userdata):
        '''
        Execute this state
        '''
        if self.return_code is not None:
            # Return any previously assigned return code if we are in pause
            userdata.status_text = self.status_text
            userdata.planned_trajectory = self.planned_trajectory
            userdata.executed_trajectory = self.executed_trajectory
            return self.return_code

        if self.moveit_client.has_result(self.current_action_topic):
            result = self.moveit_client.get_result(self.current_action_topic)

            self.status_text = " %s : %s" % (self.name, self.moveit_client.get_error_msg(result.error_code) )
            self.moveit_client.set_robot_start_state(self.current_move_group, result.trajectory_start)
            self.planned_trajectory = result.planned_trajectory
            self.executed_trajectory = result.executed_trajectory
            if (result.error_code.val == MoveItErrorCodes.SUCCESS):
                self.return_code = 'reached'
            elif (result.error_code.val == MoveItErrorCodes.CONTROL_FAILED):
                self.return_code = 'control_failed'
            elif result.error_code.val == MoveItErrorCodes.PLANNING_FAILED:
                self.return_code = 'planning_failed'
            elif result.error_code.val == MoveItErrorCodes.INVALID_MOTION_PLAN:
                self.return_code = 'planning_failed'
            elif result.error_code.val == MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
                self.return_code = 'control_failed'
            elif result.error_code.val == MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA:
                self.return_code = 'planning_failed'
            elif result.error_code.val == MoveItErrorCodes.TIMED_OUT:
                self.return_code = 'planning_failed'
            elif result.error_code.val == MoveItErrorCodes.PREEMPTED:
                self.return_code = 'control_failed'
            elif result.error_code.val == MoveItErrorCodes.START_STATE_IN_COLLISION:
                self.return_code = 'planning_failed'
            elif result.error_code.val == MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS:
                self.return_code = 'planning_failed'
            elif ( (result.error_code.val >= MoveItErrorCodes.INVALID_GROUP_NAME) and \
                   (result.error_code.val <= MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS) ):
                self.return_code = 'planning_failed'
            elif ( (result.error_code.val >= MoveItErrorCodes.SENSOR_INFO_STALE) and \
                   (result.error_code.val <= MoveItErrorCodes.FRAME_TRANSFORM_FAILURE) ):
                self.return_code = 'planning_failed'
            elif result.error_code.val == MoveItErrorCodes.NO_IK_SOLUTION:
                self.return_code = 'planning_failed'
            elif result.error_code.val == MoveItErrorCodes.FAILURE:
                self.return_code = 'planning_failed'
            else:
                Logger.logwarn(' Unknown code '+self.status_text+str(result.error_code.val)+' not '+str(MoveItErrorCodes.SUCCESS))
                self.return_code = 'failed'

        elif self.moveit_client.get_state(self.current_action_topic) == GoalStatus.ABORTED:
            # No result returned is returned for this action, so go by the client state
            self.status_text = "MoveGroup - %s request aborted by move group action server" % (self.name)
            self.return_code = 'failed'
        elif self.moveit_client.get_state(self.current_action_topic) == GoalStatus.REJECTED:
            # No result returned is returned for this action, so go by the client state
            self.status_text = "MoveGroup - %s request rejected by move group action server" % (self.name)
            self.return_code = 'failed'
        elif (self.timeout_target is not None) and (rospy.Time.now() > self.timeout_target):
            self.status_text = "MoveGroup - timeout waiting for %s move group (%f > %f (%f))" % (self.name, rospy.Time.now().to_sec(), self.timeout_target.to_sec(), self.timeout_duration.to_sec())
            self.return_code = 'failed'

        if self.return_code is not None:
            # If anything changed above
            if (self.return_code == 'done'):
                Logger.loginfo(self.status_text)
            else:
                Logger.logwarn(self.status_text)

            userdata.status_text = self.status_text
            userdata.planned_trajectory = self.planned_trajectory
            userdata.executed_trajectory = self.executed_trajectory
            return self.return_code

    def on_enter(self, userdata):
        self.status_text = None
        self.return_code = None
        self.planned_trajectory=None
        self.executed_trajectory=None
        self.timeout_target = None

        # Retrieve the relevant data
        try:
            self.joint_names     = userdata.joint_names
        except Exception as e:
            self.joint_names = None

        try :

            if (self.given_move_group is None):
                self.current_move_group = userdata.move_group

            self.current_action_topic   = self.moveit_client._move_group_clients_[self.current_move_group]

            self.joint_values    = userdata.joint_values
            if (self.joint_names == None):
                self.joint_names = self.moveit_client.get_joint_names(self.current_move_group)

            if (len(self.joint_values) != len(self.joint_names)):
                Logger.logwarn('Mismatch in joint names and values (%d vs. %d) -' % (len(self.joint_values), len(self.joint_names)))
                self.return_code = 'param_error'
                return

        except Exception as e:
            Logger.logwarn('Failed to set up the action client for %s - invalid user data parameters\n%s' % (self.name, str(e)))
            self.return_code = 'failed'
            return


        try:

            if not self.moveit_client.is_available(self.current_action_topic):
                self.moveit_client.setup_action_client(self.current_action_topic, MoveGroupAction, self.wait_duration)
                if not self.moveit_client.is_available(self.current_action_topic):
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
            Logger.loginfo(" %s - ready to send target ..." % self.name)
            self.moveit_client.send_move_group_joint_space_goal(self.current_move_group,
                                                                self.joint_names, self.joint_values,
                                                                False) # Plan AND execute
            if (self.timeout_duration > rospy.Duration(0.0)):
                self.timeout_target = rospy.Time.now() + self.timeout_duration
            else:
                self.timeout_target = None
            Logger.loginfo(" %s - sent target!" % self.name)

        except Exception as e:
            Logger.logwarn('Failed to send action goal for %s\n%s' % (self.name, str(e)))
            self.return_code = 'failed'

    def on_stop(self):
            try:
                if ( self.moveit_client.is_available(self.current_action_topic) \
                     and not self.moveit_client.has_result(self.current_action_topic) ):
                    # Cancel any active goals
                    self.moveit_client.cancel(self.current_action_topic)
            except Exception as e:
                # client already closed
                Logger.logwarn('Action client already closed - %s\n%s' % (self.current_action_topic, str(e)))

    def on_pause(self):
        try:
            self.moveit_client.cancel(self.current_action_topic)
        except Exception as e:
            # client already closed
            Logger.logwarn('Action client already closed - %s\n%s' % (self.current_action_topic, str(e)))

    def on_resume(self, userdata):
        # If paused during state execution, then re-send the clear command
        self.on_enter(userdata)
