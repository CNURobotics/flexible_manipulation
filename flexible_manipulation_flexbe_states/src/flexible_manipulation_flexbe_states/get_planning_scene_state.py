#!/usr/bin/env python

import rospy

from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient

from actionlib_msgs.msg import GoalStatus

from flexible_manipulation_msgs.msg import GetPlanningSceneAction, GetPlanningSceneGoal

from moveit_msgs.msg import PlanningScene, PlanningSceneComponents

from flexible_manipulation_flexbe_states.proxy import ProxyMoveItClient

'''
Created on 28-Feb-2018

@author: Julie Gates, Jenny Gu, and David Conner
'''

class GetPlanningSceneState(EventState):
    '''
    State to get planning scene using GetPlanningSceneAction.

    -- components           uint32                  Bitfield defining the relevant parts of planning scene that are of interest.
                                                            (default: 1023 - send everything )
    -- timeout              double                  How long to wait for planning scene.
                                                            (default: 5 seconds)
    -- wait_duration        double                  How long to wait for action server (seconds) (0.0 waits indefinitely)
    -- action_topic         string                  Topic name for the ApplyPlanningScene action server.
                                                        (default: None --> Use user data)

    ># action_topic         string                 Topic name for the ApplyPlanningScene action server (if None )


    #> scene                PlanningScene          Output scene

    <= done                                        Planning scene was successfully applied.
    <= failed                                      Applying planning scene failed.


    '''

    def __init__(self, components=1023, timeout=5.0, wait_duration=2.0, action_topic=None):
        '''
        Constructor
        '''
        super(GetPlanningSceneState, self).__init__(
            input_keys=['action_topic'],
            outcomes=['done', 'failed'],
            output_keys=['scene'] )

        self.client = None
        self.components  = components

        self.timeout_duration = rospy.Duration(timeout)
        self.wait_duration = wait_duration
        self.given_action_topic  = action_topic
        if (self.given_action_topic is not None) and (len(self.given_action_topic) > 0):
            # If topic is defined, set the client up on startup
            self.client = ProxyActionClient({self.given_action_topic: GetPlanningSceneAction},
                                              self.wait_duration)
        else:
            self.given_action_topic = None

        self.current_action_topic = self.given_action_topic

        self.scene = list()

        self.return_code = None

    def execute(self, userdata):
        '''
        Execute this state
        '''
        if self.return_code is not None:
            # Return any previously assigned return code if we are in pause
            userdata.scene = self.scene
            return self.return_code

        if self.client.has_result(self.current_action_topic):
            result = self.client.get_result(self.current_action_topic)
            self.scene = result.scene
            self.return_code = 'done'
            userdata.scene = self.scene
            Logger.loginfo("GetPlanningScene - %s request succeeded." % (self.name))
            return self.return_code
        elif self.client.get_state(self.current_action_topic) == GoalStatus.ABORTED:
            # No result returned is returned for this action, so go by the client state
            Logger.logwarn("GetPlanningScene - %s request aborted by get planning scene action server" % (self.name))
            self.return_code = 'failed'
            return self.return_code
        elif self.client.get_state(self.current_action_topic) == GoalStatus.REJECTED:
            # No result returned is returned for this action, so go by the client state
            Logger.logwarn("GetPlanningScene - %s request rejected by get planning scene action server" % (self.name))
            self.return_code = 'failed'
            return self.return_code
        elif (self.timeout_target is not None) and (rospy.Time.now() > self.timeout_target):
            Logger.logwarn("GetPlanningScene - timeout waiting for %s to get planning scene (%f > %f (%f))" % (self.name, rospy.Time.now().to_sec(), self.timeout_target.to_sec(), self.timeout_duration.to_sec()))
            self.return_code = 'failed'
            return self.return_code

    def on_enter(self, userdata):
        self.return_code = None
        self.scene = None
        # Retrieve the relevant data
        try :
            if (self.given_action_topic is None):
                self.current_action_topic    = userdata.action_topic

        except Exception as e:
            Logger.logwarn('Failed to set up the action client for %s - invalid user data parameters\n%s' % (self.current_action_topic, str(e)))
            self.return_code = 'failed'
            return

        try:
            if (self.client is None):
                self.client = ProxyActionClient({self.current_action_topic: GetPlanningSceneAction},
                                                  self.wait_duration)
        except Exception as e:
            Logger.logwarn('Failed to set up the action client for %s\n%s' % (self.current_action_topic, str(e)))
            self.return_code = 'failed'
            return

        try:
            if not self.client.is_available(self.current_action_topic):
                self.client.setup_action_client(self.current_action_topic, GetPlanningSceneAction, self.wait_duration)
                if not self.setup_action_client.is_available(self.current_action_topic):
                    Logger.logerr( 'Action client is not available for %s' % (self.current_action_topic))
                    self.return_code = 'failed'
                    return
        except Exception as e:
            Logger.logwarn('Failed to setup the action client for %s\n%s' % (self.name, str(e)))
            self.return_code = 'failed'

        try:
            # Action Initialization
            action_goal = GetPlanningSceneGoal()
            action_goal.components = PlanningSceneComponents()
            action_goal.components.components = self.components

            if (self.timeout_duration > rospy.Duration(0.0)):
                self.timeout_target = rospy.Time.now() + self.timeout_duration
            else:
                self.timeout_target = None

            self.client.send_goal(self.current_action_topic, action_goal)

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

    def on_resume(self, userdata):
        # If paused during state execution, then re-send the clear command
        self.on_enter(userdata)
