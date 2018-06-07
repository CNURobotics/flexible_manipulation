#!/usr/bin/env python

import rospy

from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient

from actionlib_msgs.msg import GoalStatus

from flexible_manipulation_msgs.msg import ExecuteKnownTrajectoryAction, ExecuteKnownTrajectoryGoal

from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory

from flexible_manipulation_flexbe_states.proxy import ProxyMoveItClient


'''
Created on 02-Apr-2018

@author: David Conner, Julie Gates, and Jenny Gu
'''

class ExecuteKnownTrajectoryState(EventState):
    '''
    State to execute known RobotTrajectory from MoveIt planner using ExecuteKnownTrajectoryAction capability.

    -- timeout              float            How long to wait for execution completion
                                                (default: 5 seconds)
    -- max_delay            float            Seconds that a trajectory may be delayed and still be sent to controller
                                                            (default: -1 ignore time and send trajectory)
    -- wait_duration        float            How long to wait for action server (seconds) (0.0 waits indefinitely)
    -- action_topic         string           Topic name for the ExecuteKnownTrajectory action server
                                                (default: None --> Use user data)

    ># action_topic         string           Topic name for the ExecuteKnownTrajectory action server (if None )

    ># trajectory           RobotTrajectory  The trajectory to execute.

    #> status_text          string           Status text reflecting outcome
    #> goal_names           string[]         List of joint names for goal state
    #> goal_values          float[]          List of joint values for goal state

    <= done                                  Successful.
    <= failed                                Failed.
    <= param_error                           Invalid parameters - no trajectory sent

    '''

    def __init__(self, timeout=5.0, max_delay=-1.0, wait_duration=5, action_topic=None):
        '''
        Constructor
        '''
        super(ExecuteKnownTrajectoryState, self).__init__(
            input_keys=['action_topic', 'trajectory'],
            outcomes=['done', 'failed', 'param_error'],
            output_keys=['status_text', 'goal_names', 'goal_values'] )

        self.action_client  = None
        self.goal_names     = None
        self.goal_values    = None
        self.timeout_duration   = rospy.Duration(timeout)
        self.timeout_target     = None
        self.wait_duration      = wait_duration
        self.given_action_topic = action_topic
        self.max_delay          = None
        if max_delay > 0.0:
            self.max_delay = rospy.Duration(max_delay)

        try:
            if (self.given_action_topic is not None) and (len(self.given_action_topic) > 0):
                # If topic is defined, set the client up on startup
                self.action_client = ProxyActionClient({self.given_action_topic: ExecuteKnownTrajectoryAction},
                                                       self.wait_duration)
            else:
                self.given_action_topic = None # override ""

        except Exception as e:
            Logger.logerr(" %s  -  exception on initialization of ProxyMoveItClient \n%s"% (self.name, str(e)))

        self.current_action_topic = self.given_action_topic
        self.status_text  = None
        self.return_code = None

    def execute(self, userdata):
        '''
        Execute this state
        '''
        if self.return_code is not None:
            # Return any previously assigned return code if we are in pause
            userdata.status_text = self.status_text
            userdata.goal_names  = self.goal_names
            userdata.goal_values = self.goal_values
            return self.return_code

        if self.action_client.has_result(self.current_action_topic):
            result = self.action_client.get_result(self.current_action_topic)

            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                self.status_text = 'ExecuteKnownTrajectory %s success!  ' % self.name
                self.return_code = 'done'
            elif result.error_code.val == MoveItErrorCodes.PREEMPTED:
                self.status_text = 'ExecuteKnownTrajectory %s preempted '% self.name
                self.return_code = 'failed'
            elif result.error_code.val == MoveItErrorCodes.TIMED_OUT:
                self.status_text = 'ExecuteKnownTrajectory %s timed out '% self.name
                self.return_code = 'failed'
            elif result.error_code.val == MoveItErrorCodes.CONTROL_FAILED:
                self.status_text = 'ExecuteKnownTrajectory %s control failed '% self.name
                self.return_code = 'failed'
            else:
                self.status_text = 'ExecuteKnownTrajectory %s failed ' % self.name
                self.return_code = 'failed'

            Logger.loginfo('ExecuteKnownTrajectory result for %s - %s\n%s' % (self.name, self.status_text, str(self.action_client.get_goal_status_text(self.current_action_topic))))

            userdata.status_text = self.status_text
            userdata.goal_names  = self.goal_names
            userdata.goal_values = self.goal_values
            return self.return_code

        elif self.action_client.get_state(self.current_action_topic) == GoalStatus.ABORTED:
            # No result returned is returned for this action, so go by the client state
            self.status_text = "ExecuteKnownTrajectory- %s request aborted by execute known trajectory action server" % (self.name)
            self.return_code = 'failed'
        elif self.action_client.get_state(self.current_action_topic) == GoalStatus.REJECTED:
            # No result returned is returned for this action, so go by the client state
            self.status_text = "ExecuteKnownTrajectory - %s request rejected by execute known trajectory action server" % (self.name)
            self.return_code = 'failed'
        elif (self.timeout_target is not None) and (rospy.Time.now() > self.timeout_target):
            self.status_text = "ExecuteKnownTrajectory - timeout waiting for %s to execute known trajectory(%f > %f (%f))" % (self.name, rospy.Time.now().to_sec(), self.timeout_target.to_sec(), self.timeout_duration.to_sec())
            self.return_code = 'failed'

        #Logger.logwarn("ExecuteKnownTrajectory - %s (%f > %f (%f))" % (self.name, rospy.Time.now().to_sec(), self.timeout_target.to_sec(), self.timeout_duration.to_sec()))
        if (self.return_code is not None):
            Logger.logwarn( self.status_text )
            userdata.status_text = self.status_text
            userdata.goal_names  = self.goal_names
            userdata.goal_values = self.goal_values
            return self.return_code

    def on_enter(self, userdata):
        self.return_code = None
        self.status_text = None
        self.goal_names  = None
        self.goal_values = None

        # Retrieve the relevant data
        try :
            if (self.given_action_topic is None):
                self.current_action_topic    = userdata.action_topic

        except Exception as e:
            self.status_text = 'Failed to set up the action client for %s - invalid user data parameters\n%s' % (self.current_action_topic, str(e))
            self.return_code = 'param_error'
            Logger.logerr(self.status_text)
            return

        try:
            if (self.action_client is None):
                self.action_client = ProxyActionClient({self.current_action_topic: ExecuteKnownTrajectoryAction},
                                                        self.wait_duration)

            if not self.action_client.is_available(self.current_action_topic):
                # Try to re-connect if not available
                self.action_client.setupClient(self.current_action_topic, ExecuteKnownTrajectoryAction, self.wait_duration)
                if not self.action_client.is_available(self.current_action_topic):
                    self.status_text = '%s - no connection to %s available!' % (self.name, self.current_action_topic)
                    self.return_code = 'param_error'
                    Logger.loginfo(self.status_text)
                    return

        except Exception as e:
            self.status_text = 'Failed to set up the action client for %s\n%s' % (self.current_action_topic, str(e))
            self.return_code = 'param_error'
            Logger.logerr(self.status_text)
            return

        try:
            # Action Initialization
            action_goal = ExecuteKnownTrajectoryGoal()
            action_goal.trajectory = userdata.trajectory

            # Check that our trajectory is still fresh
            elapsed = rospy.Time.now() - action_goal.trajectory.joint_trajectory.header.stamp
            if self.max_delay is not None and elapsed > self.max_delay:
                self.status_text = '%s  -  Stale trajectory - %f > %f - do not send action for %s' % (self.name, elapsed.to_sec(), self.max_delay.to_sec(), self.current_action_topic )
                self.return_code = 'param_error'
                Logger.logerr(self.status_text)
                return
            elif elapsed.to_sec() > 2.0:
                Logger.logwarn("%s  -  trajectory older than %f - send anyway!" % (self.name, elapsed.to_sec()))

            # Reset the timestamp before sending as we assume execute from begining in this state
            action_goal.trajectory.joint_trajectory.header.stamp = rospy.Time(0)

            self.action_client.send_goal(self.current_action_topic, action_goal)
            if (self.timeout_duration > rospy.Duration(0.0)):
                self.timeout_target = rospy.Time.now() + action_goal.trajectory.joint_trajectory.points[-1].time_from_start + self.timeout_duration
            else:
                self.timeout_target = None

            self.goal_names  = action_goal.trajectory.joint_trajectory.joint_names
            self.goal_values = action_goal.trajectory.joint_trajectory.points[-1].positions

        except Exception as e:
            self.status_text = 'Failed to send ExecuteKnownTrajectoryGoal for %s\n%s' % (self.name, str(e))
            self.return_code = 'param_error'
            Logger.logerr(self.status_text)

    def on_stop(self):
            try:
                if ( self.action_client.is_available(self.current_action_topic) \
                     and not self.action_client.has_result(self.current_action_topic) ):
                    # Cancel any active goals
                    self.action_client.cancel(self.current_action_topic)
            except Exception as e:
                # client already closed
                Logger.logwarn('Action client already closed - %s\n%s' % (self.current_action_topic, str(e)))

    def on_pause(self):
        try:
            self.action_client.cancel(self.current_action_topic)
        except Exception as e:
            # client already closed
            Logger.logwarn('Action client already closed - %s\n%s' % (self.current_action_topic, str(e)))

    def on_resume(self, userdata):
        # If paused during state execution, then re-send the command
        self.on_enter(userdata)
