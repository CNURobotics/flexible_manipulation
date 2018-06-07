#!/usr/bin/env python

import rospy

from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient

from flexible_manipulation_msgs.msg import GetCartesianPathAction, GetCartesianPathGoal

from moveit_msgs.msg import RobotState, Constraints, RobotTrajectory, MoveItErrorCodes

from geometry_msgs.msg import Pose

from flexible_manipulation_flexbe_states.proxy import ProxyMoveItClient

'''
Created on 07-Mar-2018

@author: David Conner, Jenny Gu, and Julie Gates
'''

class GetCartesianPathState(EventState):
    '''
    State to get cartesian path using GetCartesianPathAction.

    -- timeout              double          How long to wait for clearing map confirmation
                                                (default: 5 seconds)
    -- wait_duration        double          How long to wait for action server (seconds) (0.0 waits indefinitely)
    -- max_step             float           Maximum distance between consecutive points in the returned path.
                                                    Must be specified and greater than 0
    -- jump_threshold       float           If above 0, this value assumed to be maximum distance allowed.
                                                    If this distance is above maximum threshold, the path computation fails.
    -- avoid_collision      bool            Set to true if collisions should be avoided when possible.
    -- action_topic         string          Topic name for the GetCartesianPath action server
                                                (default: None --> Use user data)

    ># action_topic         string          Topic name for the GetCartesianPath action server (if None )

    ># frame_id             string          Frame for specified waypoints

    ># start_state          RobotState      State at which to start Cartesian path

    ># group_name           string          Mandatory name of group to compute the path for

    ># link_name            string          Optional name of IK link for which waypoints are specified

    ># waypoints            Pose[]          Sequence of waypoints to be followed by the specified link while moving the specified group.

    #> trajectory           RobotTrajectory The computed solution trajectory, for the desired group, in configuration space

    #> fraction             float           If computation was incomplete, this value indicates the fraction of the path that was computed.

    #> status_text          string          Text description of error (if any)

    <= done                                 Cartesian path was successfully retrieved.
    <= failed                               Cartesian path was unable to be retrieved.


    '''

    def __init__(self, timeout=5.0, wait_duration=0.001, max_step= 1,jump_threshold=1, avoid_collision=True,action_topic=None):
        '''
        Constructor
        '''
        super(GetCartesianPathState, self).__init__(
            input_keys=['action_topic', 'header','start_state','group_name','link_name','waypoints'],
            outcomes=['done', 'failed'],
            output_keys=['trajectory','fraction', 'error_code'] )

        self.client = None
        self.timeout_duration = rospy.Duration(timeout)
        self.wait_duration = wait_duration
        self.max_step = max_step
        self.jump_threshold = jump_threshold
        self.avoid_collision = avoid_collision
        self.given_action_topic  = action_topic
        if (self.given_action_topic is not None) and (len(self.given_action_topic) > 0):
            # If topic is defined, set the client up on startup
            self.client = ProxyActionClient({self.given_action_topic: GetCartesianPathAction},
                                              self.wait_duration)
        else:
            self.given_action_topic = None # Protect against empty string
        self.current_action_topic = self.given_action_topic

        self.trajectory = list()
        self.fraction = None
        self.return_code = None
        self.status_text = None

    def execute(self, userdata):
        '''
        Execute this state
        '''
        if self.return_code is not None:
            # Return any previously assigned return code if we are in pause
            userdata.trajectory  = self.trajectory
            userdata.fraction    = self.fraction
            userdata.status_text = self.status_text
            return self.return_code

        if self.client.has_result(self.current_action_topic):
            result = self.client.get_result(self.current_action_topic)
            self.trajectory  = result.trajectory
            self.fraction    = result.fraction
            self.status_text = " %s : %s" % (self.name, self.moveit_client.get_error_msg(result.error_code) )
            self.return_code = 'done'
        elif self.client.get_state(self.current_action_topic) == GoalStatus.ABORTED:
            # No result returned is returned for this action, so go by the client state
            self.trajectory = None
            self.fraction   = None
            self.status_text = "GetCartesianPath - %s request aborted by get cartesian path action server" % (self.name)
            self.return_code = 'failed'
        elif self.client.get_state(self.current_action_topic) == GoalStatus.REJECTED:
            # No result returned is returned for this action, so go by the client state
            self.trajectory = None
            self.fraction   = None
            self.status_text = "GetCartesianPath - %s request rejected by get cartesian path action server" % (self.name)
            self.return_code = 'failed'
            return self.return_code
        elif (self.timeout_target is not None) and (rospy.Time.now() > self.timeout_target):
            self.trajectory = None
            self.fraction   = None
            self.status_text = "GetCartesianPath - timeout waiting for %s to get cartesian path(%f > %f (%f))" % (self.name, rospy.Time.now().to_sec(), self.timeout_target.to_sec(), self.timeout_duration.to_sec())
            self.return_code = 'failed'
            return self.return_code

        if self.return_code is not None:
            # If anything changed above
            userdata.trajectory  = self.trajectory
            userdata.fraction    = self.fraction
            userdata.status_text = self.status_text
            return self.return_code

        #Logger.logwarn("GetCartesianPath- %s (%f > %f (%f))" % (self.name, rospy.Time.now().to_sec(), self.timeout_target.to_sec(), self.timeout_duration.to_sec()))

    def on_enter(self, userdata):
        self.return_code = None
        self.error_code = None

        # Retrieve the relevant data
        try :
            self.frame_id         = userdata.frame_id
            self.start_state      = userdata.start_state
            self.group_name       = userdata.group_name
            self.link_name        = userdata.link_name
            self.waypoints        = userdata.waypoints
            self.path_constraints = userdata.path_constraints

        except Exception as e:
            Logger.logwarn('Failed to set up the action client for %s - invalid user data parameters\n%s' % (self.name, str(e)))
            self.error_code = 'Failed to set up the action client for %s - invalid user data parameters\n%s' % (self.name, str(e))
            self.return_code = 'failed'
            return


        try:
            if (self.given_action_topic == None):
                self.current_action_topic    = userdata.action_topic

            if (self.client is None):
                self.client = ProxyActionClient({self.current_action_topic: GetCartesianPathAction},
                                                  self.wait_duration)
        except Exception as e:
            Logger.logwarn('Failed to set up the action client for %s\n%s' % (self.current_action_topic, str(e)))
            self.error_code = 'Failed to set up the action client for %s\n%s' % (self.current_action_topic, str(e))
            self.return_code = 'failed'
            return

        try:
            # Action Initialization
            action_goal = GetCartesianPathGoal()
            # Fill out the action goal data structure
            action_goal.header.frame_id = self.frame_id
            action_goal.header.stamp = rospy.Time.now()

            action_goal.group_name = self.move_group 
            action_goal.link_name  = self.link_name
            action_goal.start_state = self.moveit_client.get_robot_start_state(self.move_group)

            action_goal.waypoints = self.waypoints
            action_goal.max_step  = self.max_step
            action_goal.jump_threshold = self.jump_threshold
            action_goal.avoid_collisions = self.avoid_collision

            action_goal.path_constraints.joint_constraints    =  self.moveit_client._joint_constraints[   self.move_group]
            action_goal.path_constraints.position_constraints =  self.moveit_client._position_constraints[self.move_group]


            self.client.send_goal(self.current_action_topic, action_goal)
            self.timeout_target = rospy.Time.now() + self.timeout_duration

        except Exception as e:
            Logger.logwarn('Failed to send action goal for group - %s\n%s' % (self.name, str(e)))
            self.error_code = 'Failed to send action goal for group - %s\n%s' % (self.name, str(e))
            self.return_code = 'failed'

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
        # If paused during state execution, then re-send the command
        self.on_enter(userdata)
