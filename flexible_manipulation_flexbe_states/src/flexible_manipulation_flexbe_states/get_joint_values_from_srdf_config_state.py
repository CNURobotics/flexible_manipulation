#!/usr/bin/env python

import rospy
import os
import xml.etree.ElementTree as ET
from rospkg import RosPack
from flexbe_core import EventState, Logger

from flexible_manipulation_flexbe_states.proxy import ProxyMoveItClient

'''
Created on 14-Feb-2018

@author: David Conner based on code by Philipp Schillinger
'''

class GetJointValuesFromSrdfConfigState(EventState):
    '''
    Simple state to look up a pre-defined joint configuration from the given SRDF description .

    ># robot_name            string       Robot associated with desired joint configuration
    ># selected_move_group   string       Move group associated with desired joint configuration
    ># config_name           string       Name of the joint configuration of interest.

    #> move_group            string       Move group associated with joint configuration
    #> joint_names           string[]     List of joint names  for the requested config.
    #> joint_values          float[]      List of joint values for the requested config.

    <= retrieved          Joint values are available.

    <= param_error        Something went wrong when accessing the SRDF parameter.

    '''

    def __init__(self):
        '''
        Constructor
        '''
        super(GetJointValuesFromSrdfConfigState, self).__init__(outcomes=['retrieved', 'param_error'],
                                                                input_keys=['robot_name','selected_move_group', 'config_name'],
                                                                output_keys=['move_group','joint_names','joint_values'])


        try:
            self.moveit_client = ProxyMoveItClient(None)

        except Exception as e:
            Logger.logerr(" %s  -  exception on initialization of ProxyMoveItClient \n%s"% (self.name, str(e)))

        self.return_code = None


    def execute(self, userdata):
        if (self.return_code is None):
            # This should have been set by on_enter
            Logger.logerr("Invalid handling of SRDF data!")
            self.return_code = 'param_error'

        # Return the code determined when entering the state
        return self.return_code

    def on_enter(self, userdata):

        # Clear data
        userdata.joint_names  = None
        userdata.joint_values = None
        self.return_code     = None

        config = None
        try:
            config = self.moveit_client.get_group_state(userdata.config_name, userdata.selected_move_group, userdata.robot_name)

            if config is None:
                Logger.logwarn('Did not find config name in pre-loaded SRDF - %s' % userdata.config_name)
                self.return_code = 'param_error'
                return

        except Exception as e:
            Logger.logwarn('Error retrieving config %s from  pre-loaded SRDF -\n%s' % (userdata.config_name, str(e)))
            self.return_code = 'param_error'
            return

        try:
            self.joint_names  = [  str(j.attrib['name'] ) for j in config.iter('joint')]
            self.joint_values = [float(j.attrib['value']) for j in config.iter('joint')]
            if (len(self.joint_names) != len(self.joint_values)):
                Logger.logwarn('Mis-match in joint names and values - %d vs. %d' % (len(self.joint_names),  len(self.joint_values)))
                self.return_code = 'param_error'
                return

            Logger.loginfo('GetSRDFConfig for '+str(userdata.config_name)+':\n'+str(self.joint_names)+'\n'+str(self.joint_values))

            # Successfully retrieved the robot names and values and added both to user data
            userdata.move_group   = userdata.selected_move_group
            userdata.joint_names  = self.joint_names
            userdata.joint_values = self.joint_values
            self.return_code      = 'retrieved'

        except Exception as e:
            Logger.logwarn('Unable to parse joint values from SRDF configuration -\n%s' % str(e))
            self.return_code = 'param_error'
            return
