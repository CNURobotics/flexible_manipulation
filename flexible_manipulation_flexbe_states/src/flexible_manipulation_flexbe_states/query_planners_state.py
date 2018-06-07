#!/usr/bin/env python

import rospy

from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient

from actionlib_msgs.msg import GoalStatus
from flexible_manipulation_flexbe_states.proxy import ProxyMoveItClient
from flexible_manipulation_msgs.msg import QueryPlannersAction, QueryPlannersGoal
from moveit_msgs.msg import PlannerInterfaceDescription

'''
Created on 07-Mar-2018

@author: David Conner, Julie Gates, and Jenny Gu
'''

class QueryPlannersState(EventState):
    '''
    State using QueryPlannersAction.

    -- timeout              double                          How long to wait for clearing map confirmation
                                                                (default: 5 seconds)

    -- wait_duration        double                          How long to wait for action server (seconds) (0.0 waits indefinitely)

    -- action_topic         string                          Topic name for the QueryPlanners action server
                                                                (default: None --> Use user data)

    ># action_topic         string                          Topic name for the QueryPlanners action server (if default is None )

    #> planner_interfaces   PlannerInterfaceDescription[]   The planning instances that could be used in the benchmark.

    <= done                                 Successful.
    <= failed                               Failed.


    '''

    def __init__(self, timeout=5.0, wait_duration=0.001, action_topic=None):
        '''
        Constructor
        '''
        super(QueryPlannersState, self).__init__(
            input_keys=['action_topic'],
            outcomes=['done', 'failed'],
            output_keys=['planner_interfaces'] )

        self.client = None
        self.timeout_duration = rospy.Duration(timeout)
        self.wait_duration = wait_duration
        self.given_action_topic  = action_topic
        if (self.given_action_topic is not None) and (len(self.given_action_topic) > 0):
            # If topic is defined, set the client up on startup
            self.client = ProxyActionClient({self.given_action_topic: QueryPlannersAction},
                                              self.wait_duration)
        else:
            self.given_action_topic = None # handle empty string

        self.current_action_topic = self.given_action_topic

        self.planner_interfaces = list()
        self.return_code = None

    def execute(self, userdata):
        '''
        Execute this state
        '''
        if self.return_code is not None:
            # Return any previously assigned return code if we are in pause
            return self.return_code

        if self.client.has_result(self.current_action_topic):
            result = self.client.get_result(self.current_action_topic)
            userdata.planner_interfaces = result.planner_interfaces
            self.return_code = 'done'
            Logger.loginfo('\nPlanners:\n %s' % str(result.planner_interfaces))
            return self.return_code
        elif self.client.get_state(self.current_action_topic) == GoalStatus.ABORTED:
            Logger.logwarn("QueryPlanners - %s request aborted by query planners action server" % (self.name))
            self.return_code = 'failed'
            return self.return_code
        elif self.client.get_state(self.current_action_topic) == GoalStatus.REJECTED:
            Logger.logwarn("QueryPlanners - %s request rejected by query planners action server" % (self.name))
            self.return_code = 'failed'
            return self.return_code
        elif self.timeout_target is not None and rospy.Time.now() > self.timeout_target:
            Logger.logwarn("QueryPlanners - timeout waiting for %s (%f > %f (%f))" % (self.name, rospy.Time.now().to_sec(), self.timeout_target.to_sec(), self.timeout_duration.to_sec()))
            self.return_code = 'failed'
            return self.return_code

        #Normal monitoring for response
        #Logger.logwarn("QueryPlanners - %s (%f > %f (%f))" % (self.name, rospy.Time.now().to_sec(), self.timeout_target.to_sec(), self.timeout_duration.to_sec()))

    def on_enter(self, userdata):
        self.return_code = None

        self.current_action_topic = self.given_action_topic

        # Retrieve the relevant data
        if self.current_action_topic == None:
            # Not action topic define, check userdata
            try :
                self.current_action_topic    = userdata.action_topic
            except Exception as e:
                Logger.logwarn('Failed to get relevant user data -\n%s' % (str(e)))
                self.return_code = 'failed'
                return

        try:
            if (self.client is None):
                self.client = ProxyActionClient({self.current_action_topic: QueryPlannersAction},
                                                  self.wait_duration)

            if not self.client.is_available(self.current_action_topic):
                self.client.setupClient(self.current_action_topic, QueryPlannersAction, self.wait_duration)
                if not self.client.is_available(self.current_action_topic):
                    self.status_text = 'Action client is not available for %s' % (self.current_action_topic)
                    self.return_code = 'failed'
                    return

        except Exception as e:
            Logger.logwarn('Failed to set up the action client for %s\n%s' % (self.action_topic, str(e)))
            self.return_code = 'failed'
            return

        try:
            # Action Initialization
            action_goal = QueryPlannersGoal()
            self.client.send_goal(self.current_action_topic, action_goal)

            if (self.timeout_duration > rospy.Duration(0.0)):
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
        pass

    def on_resume(self, userdata):
        # If paused during state execution, then re-send the clear command
        self.on_enter(userdata)
