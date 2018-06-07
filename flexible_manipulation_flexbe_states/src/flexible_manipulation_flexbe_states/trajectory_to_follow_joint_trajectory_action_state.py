#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient

from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult, JointTolerance
from flexible_manipulation_flexbe_states.proxy import ProxyMoveItClient
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

'''
Created on 06-Feb-2018

@author: David Conner
'''


class TrajectoryToFollowJointTrajectoryActionState(EventState):
    '''
        State to execute a provided trajectory using the FollowJointTrajectoryAction
        interface.

        This state provides more information about execution failures than the similar
        ExecuteKnownTrajectoryState that uses a MoveIt! capability interface.

        BEWARE! This state performs no self-/collison checking on the trajectory.
        The implicit assumption is that the trajectory provided has been checked during planning stage and is still valid.

        -- goal_time_tolerance       float            How long to wait after trajectory spec finishes
        -- max_delay                 float            Seconds that a trajectory may be delayed and still be sent to controller
                                                            (default: -1 ignore time and send trajectory)
        -- wait_duration             float            How long to wait for action server (seconds) (0.0 waits indefinitely)

        -- timeout                   float            How long to wait on response after trajectory should finish (seconds ) (0.0 waits indefinitely)

        -- action_topic              string           Topic name for the StateValidation action server
                                                      (default: None --> Use user data)

        ># trajectory_action_topic  string            Topic on which controller is listening for action calls.
        ># joint_trajectory         JointTrajectory   Trajectory to execute (can be MoveIt RobotTrajectory structure)
        ># joint_goal_tolerances    JointTolerance[]  Joint Tolerances to apply to goal
        ># joint_path_tolerances    JointTolerance[]  Joint Tolerances to apply along the path

        #> status_text              string            Status message text regarding any error
        #> goal_names               string[]          List of joint names for goal state
        #> goal_values              float[]           List of joint values for goal state

        <= reached                                  Terminal state reached
        <= goal_failed                              Failed to reach goal within tolerances.
        <= path_failed                              Failed to execute the known trajectory; early stop.
        <= invalid_request                          Invalid trajectory request
        <= param_error                              Invalid parameters
        <= failed                                   Failed

        '''

    def __init__(self, goal_time_tolerance=10.0, max_delay=-1.0, wait_duration=2.0, timeout = 1.0,action_topic=None):
        '''
            Constructor
        '''
        super(TrajectoryToFollowJointTrajectoryActionState, self).__init__(
            input_keys=['trajectory_action_topic','joint_trajectory','joint_goal_tolerances','joint_path_tolerances'],
            output_keys=['status_text', 'goal_names', 'goal_values'],
            outcomes=['reached', 'goal_failed','path_failed',  'invalid_request','param_error','failed'])

        self.goal_time_tolerance = rospy.Duration.from_sec(goal_time_tolerance)

        self.max_delay           = None
        if max_delay > 0.0:
            self.max_delay = rospy.Duration(max_delay)

        self.wait_duration       = wait_duration
        self.timeout_duration    = rospy.Duration(timeout)
        self.return_code         = None

        self.action_client = None
        self.status_text = None
        self.goal_names  = None
        self.goal_values = None

        self.given_action_topic  = action_topic
        if (self.given_action_topic is not None) and (len(self.given_action_topic) > 0):
            # If topic is defined, set the client up on startup
            self.action_client = ProxyActionClient({self.given_action_topic: FollowJointTrajectoryAction},
                                                    self.wait_duration)
        else:
            self.given_action_topic = None # handle empty string

        self.current_action_topic = self.given_action_topic

    def execute(self, userdata):
        '''
        Execute this state
        '''
        if self.return_code is not None:
            # Preserve the return code in case we are blocked or failed on_enter
            userdata.status_text = self.status_text
            userdata.goal_names  = self.goal_names
            userdata.goal_values = self.goal_values
            return self.return_code

        if self.action_client.has_result(self.current_action_topic):
            result = self.action_client.get_result(self.current_action_topic)

            if result.error_code == FollowJointTrajectoryResult.SUCCESSFUL:
                self.status_text  = 'FollowJointTrajectoryAction -  %s reached goal for %s' % (self.name, self.current_action_topic)
                self.return_code = 'reached'
            elif result.error_code == FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED:
                self.status_text  = 'FollowJointTrajectoryAction -  %s path tolerance violated  %s (error code - %s)\n %s' % (self.name, self.current_action_topic, str(result.error_code), result.error_string)
                self.return_code = 'path_failed'
            elif result.error_code == FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED:
                self.status_text  = 'FollowJointTrajectoryAction -  %s goal tolerance violated  %s (error code - %s)\n %s' % (self.name, self.current_action_topic, str(result.error_code), result.error_string)
                self.return_code = 'goal_failed'
            elif result.error_code == FollowJointTrajectoryResult.OLD_HEADER_TIMESTAMP:
                self.status_text  = 'FollowJointTrajectoryAction -  %s old header timestamp   %s (error code - %s)\n %s' % (self.name, self.current_action_topic, str(result.error_code), result.error_string)
                self.return_code = 'invalid_request'
            elif result.error_code == FollowJointTrajectoryResult.INVALID_JOINTS:
                self.status_text  = 'FollowJointTrajectoryAction -  %s invalid joints   %s (error code - %s)\n %s' % (self.name, self.current_action_topic, str(result.error_code), result.error_string)
                self.return_code = 'invalid_request'
            elif result.error_code == FollowJointTrajectoryResult.INVALID_GOAL:
                self.status_text  = 'FollowJointTrajectoryAction -  %s invalid goal  %s (error code - %s)\n %s' % (self.name, self.current_action_topic, str(result.error_code), result.error_string)
                self.return_code = 'invalid_request'
            else:
                self.status_text  = 'FollowJointTrajectoryAction -  %s unsuccesful result - %s (error code - %s)\n %s' % (self.name, self.current_action_topic, str(result.error_code), result.error_string)
                self.return_code = 'invalid_request'

        elif self.action_client.get_state(self.current_action_topic) == GoalStatus.ABORTED:
            # No result returned is returned for this action, so go by the client state
            self.status_text  = "FollowJointTrajectoryAction - %s request aborted by action server" % (self.name)
            self.return_code = 'failed'
        elif self.action_client.get_state(self.current_action_topic) == GoalStatus.REJECTED:
            # No result returned is returned for this action, so go by the client state
            self.status_text  = "FollowJointTrajectoryAction - %s request rejected by action server" % (self.name)
            self.return_code = 'failed'
        elif self.timeout_target is not None and rospy.Time.now() > self.timeout_target:
            #Logger.loginfo("FJTA - timeout ...")
            self.status_text  = "FollowJointTrajectoryAction - timeout waiting for %s (%f > %f (%f))" % (self.name, rospy.Time.now().to_sec(), self.timeout_target.to_sec(), self.timeout_duration.to_sec())
            self.return_code = 'failed'

        # Normal monitoring
        if self.return_code is not None:
            # If something changed above
            Logger.loginfo(self.status_text)
            userdata.status_text = self.status_text
            userdata.goal_names  = self.goal_names
            userdata.goal_values = self.goal_values
            return self.return_code


    def on_enter(self, userdata):
        self.return_code    = None
        self.current_action_topic = self.given_action_topic
        self.goal_names  = None
        self.goal_values = None

        try:

            if self.current_action_topic is None:
                #Logger.logwarn(" %s  -  using the action topic to %s from %s for client!"%(self.name, userdata.trajectory_action_topic, self.current_action_topic) )
                self.current_action_topic = userdata.trajectory_action_topic

            if self.action_client is None:
                self.action_client = ProxyActionClient({self.current_action_topic: FollowJointTrajectoryAction},
                                                 self.wait_duration)
                Logger.loginfo('%s - created ProxyActionClient for %s!' % (self.name, self.current_action_topic))

            if not self.action_client.is_available(self.current_action_topic):
                self.action_client.setupClient(self.current_action_topic, FollowJointTrajectoryAction, self.wait_duration)
                if not self.action_client.is_available(self.current_action_topic):
                    self.status_text = '%s - no connection to %s available!' % (self.name, self.current_action_topic)
                    self.return_code = 'param_error'
                    Logger.loginfo(self.status_text)
                    return

            # Action Initialization
            action_goal = FollowJointTrajectoryGoal()
            if isinstance(userdata.joint_trajectory,RobotTrajectory):
                Logger.loginfo('%s - using RobotTrajectory ' % self.name)
                action_goal.trajectory      = userdata.joint_trajectory.joint_trajectory
            elif isinstance(userdata.joint_trajectory, JointTrajectory):
                Logger.loginfo('%s - using JointTrajectory ' % self.name)
                action_goal.trajectory      = userdata.joint_trajectory
            else:
                self.status_text = '%s - user data does not have valid trajectory message!\n          Must be  RobotTrajectory or JointTrajectory type!' % (self.name)
                self.return_code = 'param_error'
                Logger.logerr(self.status_text)
                return

            action_goal.path_tolerance      = userdata.joint_path_tolerances
            action_goal.goal_tolerance      = userdata.joint_goal_tolerances
            action_goal.goal_time_tolerance = self.goal_time_tolerance

            # Validate that the trajectory and constraints are consistent
            num_joints = len(action_goal.trajectory.points[0].positions)
            num_path   = len(action_goal.path_tolerance)
            num_goal   = len(action_goal.goal_tolerance)

            if num_joints != num_path:
                self.status_text = '%s - Invalid number of path constraints %d vs. %d for %s' % (self.name, num_path, num_joints, self.current_action_topic)
                self.return_code = 'param_error'

            if num_joints != num_goal:
                self.status_text = '%s  -  Invalid number of goal constraints %d vs. %d for %s' % (self.name, num_path, num_joints, self.current_action_topic)
                self.return_code = 'param_error'

            for c, name in enumerate(action_goal.trajectory.joint_names):
                if action_goal.path_tolerance[c].name != name:
                    self.status_text = '%s  -  %s  name mismatch %s vs. %s for path constraints' % (self.name, self.current_action_topic, action_goal.path_tolerance[c].name, name )
                    self.return_code = 'param_error'

                if action_goal.goal_tolerance[c].name != name:
                    self.status_text = '%s  -  %s  name mismatch %s vs. %s for goal constraints' % (self.name, self.current_action_topic, action_goal.goal_tolerance[c].name, name )
                    self.return_code = 'param_error'

            if self.return_code is not None:
                # Something went wrong
                Logger.logerr(self.status_text)
                return

            # Check that our trajectory is still fresh
            elapsed = rospy.Time.now() - action_goal.trajectory.header.stamp
            if self.max_delay is not None and elapsed > self.max_delay:
                self.status_text = '%s  -  Stale trajectory - %f > %f - do not send action for %s' % (self.name, elapsed.to_sec(), self.max_delay.to_sec(), self.current_action_topic )
                self.return_code = 'param_error'
                Logger.logerr(self.status_text)
                return
            elif elapsed.to_sec() > 2.0:
                Logger.logwarn("%s  -  trajectory older than %f - send anyway!" % (self.name, elapsed.to_sec()))

            if (self.timeout_duration > rospy.Duration(0.0)):
                self.timeout_target = rospy.Time.now() + action_goal.trajectory.points[-1].time_from_start + self.timeout_duration
            else:
                self.timeout_target = None

            # Good to go for it!
            action_goal.trajectory.header.stamp = rospy.Time(0.0) # Start from the beginning of the trajectory with this node
            self.action_client.send_goal(self.current_action_topic, action_goal)

            self.goal_names  = action_goal.trajectory.joint_names
            self.goal_values = action_goal.trajectory.points[-1].positions

        except Exception as e:
            self.status_text = '%s  -  Failed to send action for %s\n%s' % (self.name, self.current_action_topic, str(e))
            self.return_code = 'param_error'
            Logger.logerr(self.status_text)
            return

    def on_exit(self, userdata):
        try:
            if self.action_client is not None and \
               self.action_client.is_active( self.current_action_topic):
                self.action_client.cancel( self.current_action_topic)
                Logger.loginfo("%s  -  cancelling active FJT action goal for %s." % (self.name, self.current_action_topic))
        except Exception as e:
            Logger.logwarn('%s  -  caught exception on exit\n%s' % (self.name, str(e) ) )

    def on_pause(self, userdata):
        try:
            if self.action_client is not None and \
               self.action_client.is_active( self.current_action_topic):
                self.action_client.cancel( self.current_action_topic)
                Logger.loginfo("%s  -  cancelling active FJT action goal for %s." % (self.name, self.current_action_topic))
        except Exception as e:
            Logger.logwarn('%s  -  caught exception on exit\n%s' % (self.name, str(e) ) )
