#!/usr/bin/env python

import rospy

from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient

from actionlib_msgs.msg import GoalStatus

from flexible_manipulation_msgs.msg import StateValidationAction, StateValidationGoal
from moveit_msgs.msg import Constraints, RobotState, ContactInformation, CostSource, ConstraintEvalResult

from flexible_manipulation_flexbe_states.proxy import ProxyMoveItClient

'''
Created on 05-Mar-2018

@author: David Conner, Julie Gates, and Jenny Gu
'''

class StateValidationState(EventState):
    '''
    State to validate state using StateValidationAction.

    -- timeout              double          How long to wait for state validation
                                                (default: 5 seconds)
    -- wait_duration        double          How long to wait for action server (seconds) (0.0 waits indefinitely)
    -- action_topic         string          Topic name for the StateValidation action server
                                                (default: None --> Use user data)

    -- group_name           string          Group name for the StateValidation request
                                                (default: None --> Use user data)

    ># action_topic         string          Topic name for the StateValidation action server (if None )

    ># group_name           string          Name of the group to be used for query

    #> status_text          string          Status text for any errors

    <= valid                                State has been validated.
    <= invalid                              State is not valid given constraints
    <= failed                               State has not been validated.


    '''

    def __init__(self, timeout=5.0, wait_duration=0.001, action_topic=None, group_name=None):
        '''
        Constructor
        '''
        super(StateValidationState, self).__init__(
            input_keys=['action_topic','group_name'],
            outcomes=['valid', 'invalid', 'failed'],
            output_keys=['status_text'] )

        self.client = None
        self.wait_duration = wait_duration
        self.given_action_topic  = action_topic
        if (self.given_action_topic is not None) and (len(self.given_action_topic) > 0):
            # If topic is defined, set the client up on startup
            self.client = ProxyActionClient({self.given_action_topic: StateValidationAction},
                                              self.wait_duration)
        else:
            self.given_action_topic = None # handle empty string

        self.current_action_topic = self.given_action_topic

        self.group_name = group_name
        if (self.group_name is not None and len(self.group_name)==0):
            self.group_name = None # handle empty string

        # Not doing anything with result values for now
        #self.valid = None
        #self.contacts = None
        #self.cost_sources = None
        #self.constraint_result = None

        self.moveit_client = None
        try:
            self.moveit_client = ProxyMoveItClient(None)

        except Exception as e:
            Logger.logerr(" %s  -  exception on initialization of ProxyMoveItClient \n%s"% (self.name, str(e)))

        self.timeout_duration = rospy.Duration(timeout)
        self.return_code = None
        self.status_text  = None

    def execute(self, userdata):
        '''
        Execute this state
        '''
        if self.return_code is not None:
            # Return any previously assigned return code if we are in pause
            userdata.status_text = self.status_text
            return self.return_code

        if self.client.has_result(self.current_action_topic):
            result = self.client.get_result(self.current_action_topic)

            if (result.valid):
                self.return_code = 'valid'
                self.status_text = 'checked robot state is valid'
            else:
                self.return_code = 'invalid'
                self.status_text = 'checked robot state is not valid! number of contacts = %d' % (len(result.contacts))

            Logger.loginfo(self.status_text)
            userdata.status_text = self.status_text
            return self.return_code

        elif self.client.get_state(self.current_action_topic) == GoalStatus.ABORTED:
            self.status_text = "StateValidation - %s request aborted by state validation action server\n %s" % (self.name, self.action_client.get_goal_status_text(self.current_action_topic))
            self.return_code = 'failed'
            Logger.logerr(self.status_text)
            userdata.status_text = self.status_text
            return self.return_code
        elif self.client.get_state(self.current_action_topic) == GoalStatus.REJECTED:
            # No result returned is returned for this action, so go by the client state
            userdata.valid = self.valid
            self.status_text = "StateValidation - %s request rejected by state validation action server" % (self.name, self.action_client.get_goal_status_text(self.current_action_topic))
            self.return_code = 'failed'
            Logger.logerr(self.status_text)
            userdata.status_text = self.status_text
            return self.return_code
        elif self.timeout_target is not None and rospy.Time.now() > self.timeout_target:
            self.status_text = "StateValidation - timeout waiting for %s to state validation (%f > %f (%f))" % (self.name, rospy.Time.now().to_sec(), self.timeout_target.to_sec(), self.timeout_duration.to_sec())
            self.return_code = 'failed'
            Logger.logerr(self.status_text)
            userdata.status_text = self.status_text
            return self.return_code

        #Logger.logwarn("StateValidation - %s (%f > %f (%f))" % (self.name, rospy.Time.now().to_sec(), self.timeout_target.to_sec(), self.timeout_duration.to_sec()))
        # Normal monitoring

    def on_enter(self, userdata):
        self.return_code = None
        self.status_text  = None

        # Retrieve the relevant data
        group_name   = self.group_name
        self.current_action_topic = self.given_action_topic

        try :

            if (self.group_name is None):
                group_name      = userdata.group_name

            if (self.given_action_topic is None):
                self.current_action_topic    = userdata.action_topic

        except Exception as e:
            self.status_text = 'Failed to set up the action client for %s - invalid user data parameters\n%s' % (self.current_action_topic, str(e))
            self.return_code = 'failed'
            Logger.logerr(self.status_text)
            return


        try:
            if (self.client is None):
                self.client = ProxyActionClient({self.current_action_topic: StateValidationAction},
                                                  self.wait_duration)

            if not self.client.is_available(self.current_action_topic):
                self.client.setupClient(self.current_action_topic, StateValidationAction, self.wait_duration)
                if not self.client.is_available(self.current_action_topic):
                    self.status_text = 'StateValidationAction client is not available for %s' % (self.current_action_topic)
                    self.return_code = 'param_error'
                    Logger.logerr(self.status_text)
                    return

        except Exception as e:
            Logger.logwarn('Failed to set up the StateValidationAction client for %s\n%s' % (self.current_action_topic, str(e)))
            self.return_code = 'failed'
            return

        try:
            # Action Initialization
            action_goal = StateValidationGoal()
            action_goal.group_name  = group_name
            action_goal.robot_state = self.moveit_client.get_robot_state(group_name)
            action_goal.constraints = self.moveit_client.get_constraints(group_name)

            if (self.timeout_duration > rospy.Duration(0.0)):
                self.timeout_target = rospy.Time.now() + self.timeout_duration
            else:
                self.timeout_target = None

            Logger.logwarn('before send goal get state in the StateValidationAction client for %s' % (self.name))
            print str(action_goal)
            print self.current_action_topic

            self.client.send_goal(self.current_action_topic, action_goal)
            Logger.logwarn('after send goal in the StateValidationAction client for %s' % (self.name))

        except Exception as e:
            self.status_text = 'Failed to send action goal for group - %s\n%s' % (self.name, str(e))
            self.return_code = 'failed'
            Logger.logerr(self.status_text)
            return

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
        pass

    def on_resume(self, userdata):
        # If paused during state execution, then re-send the clear command
        self.on_enter(userdata)
