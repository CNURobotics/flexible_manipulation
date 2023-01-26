#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

from flexbe_core import EventState, Logger
from flexible_manipulation_flexbe_states.proxy import ProxyMoveItClient

'''
Created on 2-Feb-2018

@author: David Conner based on code by Philipp Schillinger
'''

class DetermineScanMotionState(EventState):
    '''
    Plans trajectory to reach the given pose with the specified endeffector

    This version uses the base MoveIt capabilities.

    -- ignore_collisions     boolean            Should collisions be ignored? Only pass True if you are sure that it is safe.
    -- include_torso         boolean            Should the torso be included in the trajectory or stay fixed?
    -- allowed_collisions    tuple[]            List of 2-tuples (strings) for each pair of links which is allowed to collide.
                                                Set the second value to an empty string if collision to any other link is allowed.

    -- planner_id            string             Sets the ID of the planner to use (MoveIt planner id - default = "RRTConnectkConfigDefault")

    ># move_group            string             Move group (e.g. arm) used for planning
    ># target_pose           PoseStamped        Target pose to reach.

    #> joint_trajectory      JointTrajectory    Trajectory of the end effector to perform the requested motion.

    <= planned                                  Was able to generate a valid motion plan.
    <= failed                                   Failed to create a motion plan at all.
    <= param_error                              Configuration error - did not request plan

    '''

    def __init__(self):
        '''
        Constructor
        '''
        super(DetermineScanMotionState, self).__init__(outcomes=['low', 'medium', 'high'],
                                                             input_keys=['height'])

        # Logger.logerr("The MoveItPlanEndeffectorPoseState is out of whack")
        # throw "The MoveItPlanEndeffectorPoseState is out of whack"

    def execute(self, userdata):
        '''
        Execute this state
        '''

        if userdata.height > 1:
            self._return = 'high'
        elif userdata.height < 0.5:
            self._return = 'low'
        else:
            self._return = 'medium'

        return self._return



    def on_enter(self, userdata):
        self._return = None
