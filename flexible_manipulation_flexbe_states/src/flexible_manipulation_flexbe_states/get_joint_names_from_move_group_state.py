#!/usr/bin/env python

import rospy
import os
import xml.etree.ElementTree as ET
from rospkg import RosPack
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient

from flexible_manipulation_flexbe_states.proxy import ProxyMoveItClient

'''
Created on 14-Feb-2018

@author: David Conner
'''

class GetJointNamesFromMoveGroupState(EventState):
    '''
    Simple state to look up joint names given move group .

    ># robot_name            string     Robot associated with desired joint configuration
    ># selected_move_group   string     Move group associated with desired joint names

    #> move_group            string     Move group associated with joint names
    #> joint_names           string[]   List of joint names associated with the move group

    <= retrieved          Joint names are available.

    <= param_error        Something went wrong when accessing the SRDF parameter.

    '''

    def __init__(self):
        '''
        Constructor
        '''
        super(GetJointNamesFromMoveGroupState, self).__init__(outcomes=['retrieved', 'param_error'],
                                                              input_keys=['robot_name','selected_move_group'],
                                                              output_keys=['move_group', 'joint_names'])
        self.moveit_client = None
        try:
            self.moveit_client = ProxyMoveItClient(None)

        except Exception as e:
            Logger.logerr(" %s  -  exception on initialization of ProxyMoveItClient \n%s"% (self.name, str(e)))

        self.return_code = None


    def execute(self, userdata):
        if (self.return_code is None):
            # This should have been set by on_enter
            Logger.logerr("Invalid handling of data!")
            self.return_code = 'param_error'

        # Return the code determined when entering the state
        return self.return_code

    def on_enter(self, userdata):

        # Clear data
        self.joint_names     = None
        userdata.joint_names = None
        userdata.move_group  = None
        self.return_code     = None

        try:
            if userdata.selected_move_group in self.moveit_client._joint_names:
                self.joint_names = [ name for name in self.moveit_client._joint_names[userdata.selected_move_group] ]

            if self.joint_names is None:
                Logger.logwarn('Did not find move group %s name in MoveIt! client' % userdata.selected_move_group)
                self.return_code = 'param_error'
                return
            else:
                userdata.joint_names = self.joint_names
                userdata.move_group = userdata.selected_move_group
                self.return_code = 'retrieved'
        except Exception as e:
            Logger.logwarn('Error retrieving joint names for %s from  pre-loaded SRDF -\n%s' % (userdata.selected_move_group, str(e)))
            self.return_code = 'param_error'
            return
