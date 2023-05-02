#!/usr/bin/env python

import rospy

from flexbe_core import EventState, Logger
from flexible_manipulation_flexbe_states.proxy import ProxyMoveItClient

'''
Created on 3-Feb-2018

@author: David Conner
'''

class SetupProxyMoveItClientState(EventState):
    '''
    Initializes the ProxyMoveItClient with specified action_topic and optional move groups

    This version uses the base MoveIt capabilities.

    -- robot_description          string        robot description parameter (default: robot_description )

    -- robot_description_semantic string        robot description parameter (default: None (append to robot_description) )

    -- move_group_capabilities    string        list of lists with MoveGroupAction topic name and associated
                                                move groups, or single string of action topic name
                                                    for common capability  (default: "/move_group")

    -- action_type_and_topics     list          list of lists with action type and list action topics
                                                    (default: None )

    -- enter_wait_duration        float64       seconds to wait when initializing on entry to a state

    #> robot_name                 string        Name of the loaded robot (defaults to first if more than one in SRDF file)
    #> move_groups                string[]      List of available move groups

    <= connected                                Proxy is initialized and ready for use
    <= topics_unavailable                       One or more topics are unavailable
    <= param_error                              Proxy creation failed

    '''

    def __init__(self,
                 robot_description="/robot_description",
                 robot_description_semantic=None,
                 move_group_capabilities="/move_group",
                 action_type_and_topics=[ ["MoveGroupAction", ["/move_group"] ] ],
                 enter_wait_duration=0.0):
        '''
        Constructor
        '''
        super(SetupProxyMoveItClientState, self).__init__(
            output_keys=['robot_name','move_groups'],
            outcomes=['connected', 'topics_unavailable', 'param_error'])

        self.action_type_and_topics = action_type_and_topics
        self.enter_wait_duration = enter_wait_duration

        # Attempt to initialize the Proxy now on initialization
        self.return_code = None
        try:

            if isinstance(move_group_capabilities, list):
                try:
                    # Convert to dictionary of topics by group
                    capability_list = move_group_capabilities
                    move_group_capabilities = {}
                    for capability in capability_list:
                        topic = capability[0]
                        groups= capability[1]

                        for group in groups:
                            move_group_capabilities[group] = topic
                    print("Associated MoveGroup capabilities:",move_group_capabilities)
                except Exception as e:
                    Logger.logerr("SetupProxyMoveItClient - failed!\n %s" % (str(e)))
                    Logger.logerr("    Failed to process list of capability/move group names ")
                    Logger.logerr("    Should be of the form:\n [ [topic_name1, [group1, group2, ...]], [topic_name_2, [ group3, group4, ...]]]")
                    self.client = None
                    self.return_code = 'param_error'
                    return
            else:
                Logger.loginfo(" Using common MoveGroup capability topic <%s>" % move_group_capabilities)

            Logger.loginfo("SetupProxyMoveItClient - attempting to set up the ProxyMoveItClient !")
            self.client = ProxyMoveItClient( robot_description,
                                             robot_description_semantic,
                                             move_group_capabilities,
                                             enter_wait_duration)

            Logger.loginfo("SetupProxyMoveItClient - attempt connections for the ProxyMoveItClient !")
            self.client.setup_multiple_action_clients(self.action_type_and_topics,
                                                       self.enter_wait_duration)

            self.action_topics = list(ProxyMoveItClient._action_clients.keys() )

            Logger.loginfo("SetupProxyMoveItClient - success creating ProxyMoveItClient !")

        except Exception as e:
            Logger.logerr("SetupProxyMoveItClient - failed!\n %s" % (str(e)))
            Logger.logerr("    if you get a loading assimp error, there is a possible fix at github.com/ros-planning/moveit/issues/86")
            self.client = None
            self.return_code = 'param_error'

    def execute(self, userdata):
        '''
        Execute this state
        '''
        if self.return_code is not None:
            return self.return_code

        Logger.logerr("SetupProxyMoveItClient - failed to setup during on_enter!")
        self.return_code = 'param_error'
        return self.return_code


    def on_enter(self, userdata):

        if self.client is None:
            Logger.logerr("SetupProxyMoveItClient - failed during initialization!")
            self.return_code = 'param_error'
            return self.return_code

        self.return_code = None

        userdata.robot_name = ProxyMoveItClient._robot_name
        userdata.move_groups = ProxyMoveItClient._move_group_list


        try:
          # Try to establish any connections that may have previously failed

          # Try to establish connection with any move groups
          Logger.loginfo("SetupProxyMoveItClient - verify MoveGroup connections ... ")
          all_available = True
          for group in self.client._move_group_clients_:
            action_topic = self.client._move_group_clients_[group]
            ret = self.client.is_available(action_topic)
            print("            ",action_topic," ",ret)
            if not ret:
                ret = self.client.setup_action_client(action_topic, "MoveGroupAction", self.enter_wait_duration)
                if not ret:
                    Logger.logwarn('SetupProxyMoveItClient - Action topic %s connection to action server is not available' % (action_topic))
                    all_available = False
                else:
                    Logger.logwarn('SetupProxyMoveItClient - New connection to %s action server established!' % (action_topic))

          # Try any additional topics
          if self.action_type_and_topics is not None:
            Logger.loginfo("SetupProxyMoveItClient - verify additional Action server connections ... ")
            for action_type, action_topics in self.action_type_and_topics:
                print("     Action type: ", action_type, "  topics: ",action_topics)
                for action_topic in action_topics:
                    ret = self.client.is_available(action_topic)
                    print("            ",action_topic," ",ret)
                    if not ret:
                        ret = self.client.setup_action_client(action_topic, action_type, self.enter_wait_duration)
                        if not ret:
                            Logger.logwarn('SetupProxyMoveItClient - Action topic %s connection to action server is not available' % (action_topic))
                            all_available = False
                        else:
                            Logger.logwarn('SetupProxyMoveItClient - Connection to %s action server established!' % (action_topic))
                    else:
                        Logger.logwarn('SetupProxyMoveItClient - Connection to %s action server previously established!' % (action_topic))

          # Update the list of available topics
          self.action_topics = list(ProxyMoveItClient._action_clients.keys() )
          Logger.loginfo(" ProxyMoveItClient : Available topics: %s" % str(self.action_topics))

          if all_available:
            self.return_code = 'connected'
          else:
            self.return_code = 'topics_unavailable'

        except Exception as e:
            Logger.logwarn('SetupProxyMoveItClient - Could not request connections!\n%s' % str(e))
            self.return_code = 'param_error'
            return

        # Debugging
        Logger.loginfo("SetupProxyMoveItClient - Output the current setup data ...")
        Logger.logwarn(" Data - %s" % (self.client.dump_moveit_client()))
