#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached

from sensor_msgs.msg import JointState

from flexible_manipulation_flexbe_states.proxy import ProxyMoveItClient

'''
Created on 23-Jan-2018

@author: David Conner based on code by Alberto Romay
'''

class GetCurrentJointValuesUserState(EventState):
    '''
    Retrieves current values of specified joints from a topic.
    In this version, specified joint names are passed by user data

    -- timeout                double       Timeout value (optional)

    -- joint_states_topic    string        Optional name of joint states topic
                                           (default: /joint_states)

    ># joint_names        string[]         List of desired joint names.

    #> joint_values       float[]          List of current joint values.

    <= retrieved                           Joint values are available.
    <= timeout                             Joint values are not available.

    '''

    def __init__(self, timeout=None, joint_states_topic='/joint_states'):
        '''
        Constructor
        '''
        super(GetCurrentJointValuesUserState, self).__init__(
            outcomes=['retrieved','timeout'],
            output_keys=['joint_values'],
            input_keys=['joint_names'])

        self.topic = joint_states_topic
        self.sub = ProxySubscriberCached({self.topic: JointState})

        self.joint_names = None
        self.joint_values = list()
        self.return_code = None

        if (timeout is not None) and (timeout > 0.0):
            self.timeout_duration = rospy.Duration(timeout)
        else:
            self.timeout_duration = None

    def execute(self, userdata):
        if (self.return_code is not None):
            # Handle blocked transition or error during on_enter
            return self.return_code

        while self.sub.has_buffered(self.topic):
            msg = self.sub.get_from_buffer(self.topic)
            for i in range(len(msg.name)):
                if msg.name[i] in self.joint_names \
                and self.joint_values[self.joint_names.index(msg.name[i])] is None:
                    self.joint_values[self.joint_names.index(msg.name[i])] = msg.position[i]

        if all(v is not None for v in self.joint_values):
            userdata.joint_values = self.joint_values
            self.return_code = 'retrieved'
            return 'retrieved'

        if (self.timeout_duration is not None and \
            (rospy.Time.now()-self.start_time) > self.timeout_duration ):
            Logger.logerr('Timeout %s - found %s of %s' % (self.name, str(self.joint_values), str(self.joint_names)))
            self.return_code = 'timeout'
            return 'timeout'


    def on_enter(self, userdata):
        self.sub.enable_buffer(self.topic)
        self.sub.remove_last_msg(self.topic, True)
        self.joint_names = userdata.joint_names
        self.joint_values = [None] * len(self.joint_names)
        self.return_code = None
        self.start_time = rospy.Time.now()
        userdata.joint_values = None

    def on_exit(self, userdata):
        self.sub.disable_buffer(self.topic)
