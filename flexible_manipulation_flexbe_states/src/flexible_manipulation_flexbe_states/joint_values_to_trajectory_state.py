#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from flexible_manipulation_flexbe_states.proxy import ProxyMoveItClient

'''
Created on 27-Jan-2018

@author: David Conner based on code by Alberto Romay
'''


class JointValuesToTrajectoryState(EventState):
    '''
        State to convert a target joint configuration into a single point trajectory.

        BEWARE! This state performs no self-/collison planning!

        -- duration             float               Duration of trajectory to target point (seconds)

        ># joint_names          string[]            Names of the target joints.

        ># joint_values         float[]             Target configuration of the joints.
                                                        Same order as their corresponding names in joint_names

        #> joint_trajectory     JointTrajectory     Trajectory to execute

        <= done                                     Target joint configuration has been reached.
        <= param_error                              Failed to request the service.

        '''

    def __init__(self, duration=10.0):
        '''
            Constructor
        '''
        super(JointValuesToTrajectoryState, self).__init__(
            input_keys=[ 'joint_names', 'joint_values'],
            output_keys=[ 'joint_trajectory' ],
            outcomes=['done', 'param_error'])

        self.return_code    = None
        self.duration = duration

    def execute(self, userdata):
        if (self.return_code is None):
            # This should have been set by on_enter
            Logger.logerr("Invalid handling of SRDF data!")
            self.return_code = 'param_error'

        return self.return_code

    def on_enter(self, userdata):
        self.return_code   = None

        try:

            if (len(userdata.joint_names) != len(userdata.joint_values)):
                Logger.logerr('Mismatch in joint names and values (%d vs. %d) -' % (len(userdata.joint_values), len(self.joint_names)))
                self.return_code = 'param_error'
                return

            # Action Initialization
            joint_trajectory = JointTrajectory()
            joint_trajectory.header.stamp = rospy.Time.now()
            joint_trajectory.joint_names = [name for name in userdata.joint_names]
            joint_trajectory.points = [JointTrajectoryPoint()]
            joint_trajectory.points[0].time_from_start = rospy.Duration(self.duration)
            joint_trajectory.points[0].positions = [value for value in userdata.joint_values]

            # Assign to the user data
            userdata.joint_trajectory = joint_trajectory
            self.return_code = 'done'

        except Exception as e:
            Logger.logerr('Unable to convert joint values to trajectory - \n%s' % str(e))
            self.return_code = 'param_error'
