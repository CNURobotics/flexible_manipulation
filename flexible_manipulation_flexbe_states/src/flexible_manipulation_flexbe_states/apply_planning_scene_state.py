#!/usr/bin/env python

import rospy

from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient

from actionlib_msgs.msg import GoalStatus

from flexible_manipulation_msgs.msg import ApplyPlanningSceneAction, ApplyPlanningSceneGoal

from flexible_manipulation_flexbe_states.proxy import ProxyMoveItClient

from moveit_msgs.msg import PlanningScene

'''
Created on 05-Mar-2018

@author: David Conner, Julie Gates, and Jenny Gu
'''

class ApplyPlanningSceneState(EventState):
    '''
    State to apply planning scene using ApplyPlanningSceneAction.

    -- timeout              double          How long to wait for clearing map confirmation
                                                (default: 5 seconds)
    -- wait_duration        double          How long to wait for action server (seconds) (0.0 waits indefinitely)
    -- action_topic         string          Topic name for the ApplyPlanningScene action server
                                                (default: None --> Use user data)

    ># action_topic         string          Topic name for the ApplyPlanningScene action server (if None )

    <= done                                 Planning scene was successfully applied.
    <= failed                               Applying planning scene failed.


    '''

    def __init__(self, timeout=5.0, wait_duration=5, action_topic=None):
        '''
        Constructor
        '''
        super(ApplyPlanningSceneState, self).__init__(
            input_keys=['action_topic'],
            outcomes=['done', 'failed'])

        self.client = None
        self.timeout_duration = rospy.Duration(timeout)
        self.wait_duration = wait_duration
        self.action_topic  = action_topic
        if (self.action_topic is not None) and (len(self.action_topic) > 0):
            # If topic is defined, set the client up on startup
            self.client = ProxyActionClient({self.action_topic: ApplyPlanningSceneAction},
                                              self.wait_duration)
        else:
            self.action_topic = None # override ""

        self.moveit_client = None
        try:
            self.moveit_client = ProxyMoveItClient(None)

        except Exception as e:
            Logger.logerr(" %s  -  exception on initialization of ProxyMoveItClient \n%s"% (self.name, str(e)))

        self.success = None
        self.return_code = None

    def execute(self, userdata):
        '''
        Execute this state
        '''
        if self.return_code is not None:
            # Return any previously assigned return code if we are in pause
            return self.return_code

        if self.client.has_result(self.action_topic):
            result = self.client.get_result(self.action_topic)

            Logger.loginfo('ApplyPlanningScene - %s' % str(self.client.get_goal_status_text(self.action_topic)))

            if result.success:
                Logger.loginfo('ApplyPlanningScene success!  ')
                self.return_code = 'done'
                return self.return_code
            else:
                Logger.logwarn('ApplyPlanningScene failed ' )
                self.return_code = 'failed'
                return self.return_code

        elif self.client.get_state(self.action_topic) == GoalStatus.ABORTED:
            # No result returned is returned for this action, so go by the client state
            userdata.success = self.success
            Logger.logwarn("ApplyPlanningScene - %s request aborted by apply planning scene action server" % (self.name))
            self.return_code = 'failed'
            return self.return_code
        elif self.client.get_state(self.action_topic) == GoalStatus.REJECTED:
            # No result returned is returned for this action, so go by the client state
            userdata.success = self.success
            Logger.logwarn("ApplyPlanningScene - %s request rejected by apply planning scene action server" % (self.name))
            self.return_code = 'failed'
            return self.return_code
        elif rospy.Time.now() > self.timeout_target:
            userdata.success = self.success
            Logger.logwarn("ApplyPlanningScene - timeout waiting for %s to apply planning scene (%f > %f (%f))" % (self.name, rospy.Time.now().to_sec(), self.timeout_target.to_sec(), self.timeout_duration.to_sec()))
            self.return_code = 'failed'
            return self.return_code

        #Logger.logwarn("ApplyPlanningScene - %s (%f > %f (%f))" % (self.name, rospy.Time.now().to_sec(), self.timeout_target.to_sec(), self.timeout_duration.to_sec()))

    def on_enter(self, userdata):
        self.return_code = None

        # Retrieve the relevant data
        try :
            if (self.action_topic is None):
                self.action_topic    = userdata.action_topic

        except Exception as e:
            Logger.logwarn('Failed to set up the action client for %s - invalid user data parameters\n%s' % (self.action_topic, str(e)))
            self.return_code = 'failed'
            return

        try:
            if (self.client is None):
                self.client = ProxyActionClient({self.action_topic: ApplyPlanningSceneAction},
                                                  self.wait_duration)
        except Exception as e:
            Logger.logwarn('Failed to set up the action client for %s\n%s' % (self.action_topic, str(e)))
            self.return_code = 'failed'
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
            # Action Initialization
            action_goal = ApplyPlanningSceneGoal()
            action_goal.scene = self.moveit_client._planning_scene

            self.client.send_goal(self.action_topic, action_goal)
            self.timeout_target = rospy.Time.now() + self.timeout_duration

        except Exception as e:
            Logger.logwarn('Failed to send ApplyPlanningSceneGoal for group - %s\n%s' % (self.name, str(e)))
            self.return_code = 'failed'

    def on_stop(self):
            try:
                if ( self.client.is_available(self.action_topic) \
                     and not self.client.has_result(self.action_topic) ):
                    # Cancel any active goals
                    self.client.cancel(self.action_topic)
            except Exception as e:
                # client already closed
                Logger.logwarn('Action client already closed - %s\n%s' % (self.action_topic, str(e)))

    def on_pause(self):
        pass

    def on_resume(self, userdata):
        # If paused during state execution, then re-send the clear command
        self.on_enter(userdata)
