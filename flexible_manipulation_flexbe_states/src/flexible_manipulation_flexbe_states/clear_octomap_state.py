#!/usr/bin/env python

import rospy

from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient

from actionlib_msgs.msg import GoalStatus

from flexible_manipulation_msgs.msg import ClearOctomapAction, ClearOctomapGoal

from flexible_manipulation_flexbe_states.proxy import ProxyMoveItClient

'''
Created on 28-Feb-2018

@author: Julie Gates, Jenny Gu, and David Conner
'''

class ClearOctomapState(EventState):
    '''
    State to clear octomap using ClearOctomapAction.

    -- timeout              double          How long to wait for clearing map confirmation
                                                (default: 5 seconds)
    -- wait_duration        double          How long to wait for action server (seconds) (0.0 waits indefinitely)
    -- action_topic         string          Topic name for the ClearOctomap action server
                                                (default: None --> Use user data)

    ># action_topic         string          Topic name for the ClearOctomap action server (if None )

    <= done                                 Octomap was successfully cleared.
    <= failed                               Octomap was unable to be cleared.


    '''

    def __init__(self, timeout=5.0, wait_duration=5, action_topic=None):
        '''
        Constructor
        '''
        super(ClearOctomapState, self).__init__(
            input_keys=['action_topic'],
            outcomes=['done', 'failed'] )

        self.client = None
        self.timeout_duration = rospy.Duration(timeout)
        self.wait_duration = wait_duration
        self.given_action_topic  = action_topic
        if (self.given_action_topic is not None) and (len(self.given_action_topic) > 0):
            # If topic is defined, set the client up on startup
            self.client = ProxyActionClient({self.given_action_topic: ClearOctomapAction},
                                              self.wait_duration)
        else:
            self.given_action_topic = None # override empty string

        self.current_action_topic = self.given_action_topic

        self.return_code = None

    def execute(self, userdata):
        '''
        Execute this state
        '''
        if self.return_code is not None:
            # Return any previously assigned return code if we are in pause
            return self.return_code

        if self.client.get_state(self.current_action_topic) == GoalStatus.SUCCEEDED:
            # No result returned is returned for this action, so go by the client state
            Logger.loginfo("ClearOctomap - %s request succeeded." % (self.name))
            self.return_code = 'done'
            return self.return_code
        elif self.client.get_state(self.current_action_topic) == GoalStatus.ABORTED:
            # No result returned is returned for this action, so go by the client state
            Logger.logwarn("ClearOctomap - %s request aborted by clear octomap action server" % (self.name))
            self.return_code = 'failed'
            return self.return_code
        elif self.client.get_state(self.current_action_topic) == GoalStatus.REJECTED:
            # No result returned is returned for this action, so go by the client state
            Logger.logwarn("ClearOctomap - %s request rejected by clear octomap action server" % (self.name))
            self.return_code = 'failed'
            return self.return_code
        elif (self.timeout_target is not None) and (rospy.Time.now() > self.timeout_target):
            Logger.logwarn("ClearOctomap - timeout waiting for %s to clear octomap (%f > %f (%f))" % (self.name, rospy.Time.now().to_sec(), self.timeout_target.to_sec(), self.timeout_duration.to_sec()))
            self.return_code = 'failed'
            return self.return_code

    def on_enter(self, userdata):
        self.return_code = None

        # Retrieve the relevant data
        if self.given_action_topic == None:
            # Not action topic define, check userdata
            try :
                self.current_action_topic    = userdata.action_topic
            except Exception as e:
                Logger.logwarn('Failed to get relevant user data -\n%s' % (str(e)))
                self.return_code = 'failed'
                return

        try:
            if (self.client is None):
                self.client = ProxyActionClient({self.current_action_topic: ClearOctomapAction},
                                                  self.wait_duration)
        except Exception as e:
            Logger.logwarn('Failed to set up the action client for %s\n%s' % (self.current_action_topic, str(e)))
            self.return_code = 'failed'
            return

        try:
            # Action Initialization
            action_goal = ClearOctomapGoal()
            self.client.send_goal(self.current_action_topic, action_goal)
            if self.timeout_duration > rospy.Duration(0.0):
                self.timeout_target = rospy.Time.now() + self.timeout_duration
            else:
                self.timeout_target = None

        except Exception as e:
            Logger.logwarn('Failed to send action goal for group - %s\n%s' % (self.name, str(e)))
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
        # If we are executing something, we should cancel on pause, but nothing to cancel for this action
        #try:
        #    self.client.cancel(self.current_action_topic)
        #except Exception as e:
        #    # client already closed
        #    Logger.logwarn('Action client already closed - %s\n%s' % (self.current_action_topic, str(e)))
        pass

    def on_resume(self, userdata):
        # If paused during state execution, then re-send the clear command
        self.on_enter(userdata)
