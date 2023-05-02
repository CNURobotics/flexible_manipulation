#!/usr/bin/env python

import rospy
import copy

from flexbe_core import EventState, Logger
from flexible_manipulation_flexbe_states.proxy import ProxyMoveItClient

from actionlib_msgs.msg import GoalStatus

from moveit_msgs.msg import MoveGroupGoal, Constraints, MoveItErrorCodes

'''
Created on 2-Feb-2018

@author: David Conner based on code by Philipp Schillinger
'''

class JointValuesToMoveItPlanState(EventState):
    '''
    Plans trajectory to reach the given joint configuration

    This version uses the basic MoveGroupAction interface.
    It presumes that motion plan request and planning options are previously defined
    in the ProxyMoveItClient for the relevant move group name.

    -- timeout              double              How long to wait for clearing map confirmation
                                                   (default: 5 seconds)
    -- enter_wait_duration  float               Seconds to wait to establish a new connection if unavailable
    -- action_topic         string              Topic name for the ClearOctomap action server
                                                     (default: None --> Use user data)

    ># action_topic          string             Topic on which MoveIt is listening for action calls.
    ># move_group            string             Move group (e.g. arm) used for planning
    ># joint_names           string[]           Names of the target joints.
                                                        Same order as their corresponding names in joint_values.

    ># joint_values          float[]            Target configuration of the joints.
                                                        Same order as their corresponding names in joint_names.

    #> joint_trajectory      RobotTrajectory    Trajectory of the end effector to perform the requested motion.

    <= planned                                  Was able to generate a valid motion plan.
    <= failed                                   Failed to create a motion plan at all.
    <= topics_unavailable                       No connection on the specified action topic
    <= param_error                              Configuration error - did not request plan
    '''

    def __init__(self, timeout=5.0,
                 enter_wait_duration=0.0,
                 action_topic=None):
        '''
        Constructor
        '''
        super(JointValuesToMoveItPlanState, self).__init__(outcomes=['planned', 'failed', 'topics_unavailable', 'param_error'],
                                                           input_keys=['action_topic', 'move_group','joint_names','joint_values'],
                                                           output_keys=['joint_trajectory'])

        self.timeout_duration    = rospy.Duration(timeout)
        self.enter_wait_duration = enter_wait_duration
        self.given_action_topic  = action_topic
        self.moveit_client       = None
        self.action_client       = None
        self.return_code         = None
        self.move_group          = None
        self.joint_names         = None
        self.joint_values        = None

        try:
            self.moveit_client = ProxyMoveItClient(None)

            if (self.given_action_topic is not None) and (len(self.given_action_topic) > 0):
                # If topic is defined, set the client up on startup
                self.moveit_client.setup_action_client( self.given_action_topic,
                                                        "MoveGroupAction",
                                                        self.enter_wait_duration)
            else:
                self.given_action_topic = None # override empty string

        except Exception as e:
            Logger.logerr(" Exception on initialization of %s\n %s"% (self.name, str(e)))

        self.current_action_topic = self.given_action_topic

    def execute(self, userdata):
        '''
        Execute this state
        '''
        if self.return_code is not None:
            userdata.joint_trajectory = self.robot_trajectory
            return self.return_code

        if self.action_client.has_result( self.current_action_topic):
            #@todo - log status from action client
            result = self.action_client.get_result( self.current_action_topic)

            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                self.robot_trajectory = result.planned_trajectory

                if (self.robot_trajectory.joint_trajectory.header.stamp == rospy.Time(0.0)):
                    # Add a timestamp to let us track how old the trajectory is, but
                    # most of our execution states will set back to 0.0 to force execution
                    # of the full trajectory
                    self.robot_trajectory.joint_trajectory.header.stamp = self.request_time
                self.return_code = 'planned'
                userdata.joint_trajectory = self.robot_trajectory
                Logger.loginfo('MoveIt %s found plan at %d.%d' % (self.name, self.robot_trajectory.joint_trajectory.header.stamp.secs, self.robot_trajectory.joint_trajectory.header.stamp.nsecs))
                return self.return_code
            else:
                Logger.logwarn('MoveIt failed with the following error - %s' % self.moveit_client.get_error_msg(result.error_code))
                self.return_code = 'failed'
                return self.return_code

        elif self.action_client.get_state(self.current_action_topic) == GoalStatus.ABORTED:
            # No result returned is returned for this action, so go by the client state
            Logger.logwarn("MoveIt - %s request aborted action server" % (self.name))
            self.return_code = 'failed'
            return self.return_code
        elif self.action_client.get_state(self.current_action_topic) == GoalStatus.REJECTED:
            # No result returned is returned for this action, so go by the client state
            Logger.logwarn("MoveIt - %s request rejected by action server" % (self.name))
            self.return_code = 'failed'
            return self.return_code
        elif (self.timeout_target is not None) and (rospy.Time.now() > self.timeout_target):
            Logger.logwarn("MoveIt - timeout waiting for %s to plan trajectory (%f > %f (%f))" % (self.name, rospy.Time.now().to_sec(), self.timeout_target.to_sec(), self.timeout_duration.to_sec()))
            self.return_code = 'failed'
            return self.return_code

    def on_enter(self, userdata):

        # Clear return information
        self.action_client    = None # Clear prior connection in case we changed topics
        self.request_time     = None
        self.robot_trajectory = None
        self.return_code      = None

        # We require action_topic and move_group to be set to use this state
        if (self.given_action_topic is None) and (not hasattr(userdata, 'action_topic') or userdata.action_topic is None):
            self.return_code = 'param_error'
            Logger.logwarn('Userdata action topic of state %s does not exist or is currently undefined!' % self.name)
            return

        if not hasattr(userdata, 'move_group') or userdata.move_group is None:
            self.return_code = 'param_error'
            Logger.logwarn('Userdata move_group of state %s does not exist or is currently undefined!' % self.name)
            return


        if self.moveit_client is None:
            try:
                self.moveit_client = ProxyMoveItClient(None)
            except Exception as e:
                self.moveit_client = None
                self.return_code = 'param_error'
                Logger.logerr(str(e))
                Logger.logwarn('No MoveItClient configured - state %s does not exist or is currently undefined!' % self.name)
                return

        try:
            if (self.given_action_topic is None):
                if (self.current_action_topic != userdata.action_topic) or \
                   (self.move_group != userdata.move_group) :
                   # Make sure connection is established for change in topics
                   self.current_action_topic = userdata.action_topic
                   self.move_group   = userdata.move_group

                   Logger.loginfo("%s  -  trying to connect to %s on moveit client ..." % (self.name, self.current_action_topic) )
                   self.moveit_client.setup_action_client( self.current_action_topic,
                                                           "MoveGroupAction",
                                                           self.enter_wait_duration)
            else:
                self.current_action_topic = self.given_action_topic

            if not self.moveit_client.is_available(self.current_action_topic):
                # Try to re-connect if not available
                ret = False
                if self.enter_wait_duration >= 0.0:
                    Logger.loginfo("%s  -  trying to reconnect to %s on moveit client ..." % (self.name, self.current_action_topic) )
                    ret = self.moveit_client.connect_action_server(self.current_action_topic, "MoveGroupAction", self.enter_wait_duration)

                if not ret:
                    Logger.logwarn('State %s -action topic %s connection to action server is not available' % (self.name, current_action_topic))
                    self.return_code = 'topics_unavailable'
                    return


            # Retrieve reference to the relevant action client
            self.action_client = self.moveit_client._action_clients[self.current_action_topic]


        except Exception as e:
            Logger.logwarn('State %s - invalid action connection !\n%s' % (self.name, str(e)))
            self.return_code = 'param_error'
            return

        try:
            joint_values = userdata.joint_values
            joint_names = ProxyMoveItClient._joint_names[userdata.move_group]
            if hasattr(userdata, 'joint_names'):
                joint_names = userdata.joint_names
                if len(joint_names) != len(joint_values):
                    Logger.logwarn('%s  -  Joint values mismatch %d vs. %d'% (self.name, len(joint_names), len(joint_values)))
                    self.return_code = 'param_error'
                    return

            # Grab the current data and create a deep copy for local mods
            action_goal = MoveGroupGoal(request=copy.deepcopy(ProxyMoveItClient._motion_plan_requests[userdata.move_group]),
                                        planning_options=copy.deepcopy(ProxyMoveItClient._planning_options[userdata.move_group] ) )

            action_goal.planning_options.plan_only = True

            Logger.loginfo('State %s - set goal joint constraints !' % (self.name))
            constraints = Constraints()
            constraints.joint_constraints = self.moveit_client.get_goal_joint_constraints(self.move_group, joint_values, joint_names )
            action_goal.request.goal_constraints.append(constraints)
            self.request_time = rospy.Time.now()

            print("------------  Planning Goal -----")
            print(str(action_goal))

            self.action_client.send_goal(self.current_action_topic, action_goal)
            if self.timeout_duration > rospy.Duration(0.0):
                self.timeout_target = self.request_time + self.timeout_duration + rospy.Duration(action_goal.request.allowed_planning_time)
            else:
                self.timeout_target = None


        except Exception as e:
            Logger.logwarn('Could not request a plan!\n%s' % str(e))
            self.return_code = 'param_error'


    def on_exit(self, userdata):
        try:
            if self.action_client is not None and \
               self.action_client.is_active( self.current_action_topic):
                self.action_client.cancel( self.current_action_topic)
                Logger.loginfo("%s  -  cancelling active action goal for %s." % (self.name, self.current_action_topic))
        except Exception as e:
            Logger.logwarn('%s  -  caught exception on exit\n%s' % (self.name, str(e) ) )

        self.action_client = None

    def on_pause(self, userdata):
        try:
            if self.action_client is not None and \
               self.action_client.is_active( self.current_action_topic):
                self.action_client.cancel( self.current_action_topic)
                Logger.loginfo("%s  -  cancelling active action goal for %s." % (self.name, self.current_action_topic))
        except Exception as e:
            Logger.logwarn('%s  -  caught exception on exit\n%s' % (self.name, str(e) ) )
