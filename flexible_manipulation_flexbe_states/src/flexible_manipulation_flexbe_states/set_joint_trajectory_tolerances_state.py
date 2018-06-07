#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

from control_msgs.msg import JointTolerance

from flexible_manipulation_flexbe_states.proxy import ProxyMoveItClient

'''
Created on 6-Feb-2018

@author: David Conner
'''

class SetJointTrajectoryTolerancesState(EventState):
    '''
    Simple state to set user data for control_msgs/JointTolerance.msg data used by the
    FollowJointTrajectoryAction for path and goal tolerances given joint names and constraint values

    If a single constraint is given, it is used for all joints in the userdata joint names; if multiple
    constraints are given, then the number must match the number of names in the joint names vector.

    -- position_constraints     float[]            Joint position constraints - single for common (default: [0.0])

    -- velocity_constraints     float[]            Joint velocity constraints - single for common (default: [0.0])

    -- acceleration_constraints float[]            Joint position constraints - single for common (default: [0.0])

    ># joint_names              string[]           Joint names used by system
    #> joint_tolerances         JointTolerance[]   Tolerances for joint

    <= configured                    Tolerances are defined

    <= param_error                   Something went wrong when assigning constraints (likely name mismatch)


    '''

    def __init__(self, position_constraints=[0.0], velocity_constraints=[0.0], acceleration_constraints=[0.0]):
        '''
        Constructor
        '''
        super(SetJointTrajectoryTolerancesState, self).__init__(outcomes=['configured', 'param_error'],
                                                                input_keys =['joint_names'],
                                                                output_keys=['joint_tolerances'])

        self.position_constraints     = position_constraints
        self.velocity_constraints     = velocity_constraints
        self.acceleration_constraints = acceleration_constraints
        self.joint_names = None
        self.return_code = None


    def execute(self, userdata):
        if (self.return_code is None):
            # This should have been set by on_enter
            Logger.logerr("Invalid handling of tolerance data for %s!"%(self.name))
            self.return_code = 'param_error'

        # Return the code determined when entering the state
        return self.return_code

    def on_enter(self, userdata):

        # Clear data
        userdata.joint_tolerances = None
        self.return_code         = None


        try:

            num_names = len( userdata.joint_names)
            num_posn  = len(self.position_constraints)
            num_vel   = len(self.velocity_constraints)
            num_acc   = len(self.acceleration_constraints)

            if (num_posn > 1 and num_posn != num_names):
                Logger.logwarn('Mis-match in joint names and position constraints -%d vs. %d' % (num_names,  num_posn ) )
                self.return_code = 'param_error'

            if (num_vel > 1 and num_vel != num_names):
                Logger.logwarn('Mis-match in joint names and position constraints - %d vs. %d' % (num_names,  num_vel  ) )
                self.return_code = 'param_error'

            if (num_acc > 1 and num_acc != num_names):
                Logger.logwarn('Mis-match in joint names and position constraints - %d vs. %d' % (num_names,  num_acc  ) )
                self.return_code = 'param_error'

            if self.return_code is not None:
                return self.return_code

            tolerances = []

            for ndx, name in enumerate(userdata.joint_names):
                jt = JointTolerance()
                jt.name = name
                if (num_posn > 1):
                    jt.position = self.position_constraints[ndx]
                else:
                    jt.position = self.position_constraints[0]

                if (num_vel > 1):
                    jt.velocity = self.velocity_constraints[ndx]
                else:
                    jt.velocity = self.velocity_constraints[0]

                if (num_acc > 1):
                    jt.acceleration = self.acceleration_constraints[ndx]
                else:
                    jt.acceleration = self.acceleration_constraints[0]

                tolerances.append(jt)

            # Successfully set the joint tolerances and added to user data
            userdata.joint_tolerances = tolerances
            self.return_code         = 'configured'

        except Exception as e:
            Logger.logwarn('Unable to configure joint tolerances for %s \n%s' % (self.name, str(e)) )
            self.return_code = 'param_error'
            return
