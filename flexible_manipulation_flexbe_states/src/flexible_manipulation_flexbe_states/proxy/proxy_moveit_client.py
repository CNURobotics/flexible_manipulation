#!/usr/bin/env python

import rospy
import actionlib
from threading import Timer
import time
import sys
import json
import copy

import xml.etree.ElementTree as ET

import moveit_commander
#from moveit_msgs.msg import *

from flexbe_core.logger import Logger
from flexbe_core.proxy import ProxyActionClient
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, MoveGroupAction, MoveGroupGoal, MoveItErrorCodes, PlanningOptions, PlanningScene, RobotState, RobotTrajectory

class ProxyMoveItClient(object):
    """
    A proxy for easily using MoveIt! standard capabilities via the RobotCommander interface

    This code shares one RobotCommander interface among several action topics.

    The proxy is used to handle parameterization of several MoveIt capabilities, and to initialize
    various action interfaces.  The actual interfaces to the capabilities are via individual
    state implementations with Action interfaces.
    """
    _robot_commander = None
    _goal_defaults  = None
    _robot_name = None
    _robot_semantics = None
    _robot_description = None
    _semantic_description = None

    _action_clients = {}        # Dictionary of Action clients per topic

    _planning_scene = PlanningScene()
    _planning_scene.robot_state.is_diff = True # Flags start state as empty to use current state on server

    Logger.logerr(" ProxyMoveItClient: TODO - set up the default planning scene data and add handlers ")
    # Plan request data structures used by several actions
    _motion_plan_requests = {}          # Dictionary of active plan requests per move group
    _default_motion_plan_requests = {}  # Dictionary of default plan requests per move group

    _planning_options = {}              # Dictionary of active planning options per move group
    _default_planning_options = {}      # Dictionary of default planning optionsper move group

    _joint_constraints    = {}          # Dictionary of dictionaries per move group and joint name
    _default_joint_constraints    = {}  # Dictionary of dictionaries per move group and joint name

    _position_constraints    = {}          # Dictionary of position constraints per move group
    _default_position_constraints    = {}  # Dictionary of position constraints  per move group

    _orientation_constraints    = {}          # Dictionary of orientation constraints per move group
    _default_orientation_constraints    = {}  # Dictionary of orientation constraints  per move group

    _visibility_constraints    = {}          # Dictionary of visibility constraints per move group
    _default_visibility_constraints   = {}   # Dictionary of visibility constraints  per move group

    _robot_states = {}                       # Dictionary of robot state queries per move group
    _default_robot_states = {}               # Dictionary of robot state queries per move group

    _robot_start_states = {}                 # Dictionary of robot state queries per move group
    _robot_target_states = {}                # Dictionary of robot state queries per move group

    _move_group_list = None             # List of available move groups for this robot
    _move_group_clients_={}             # Dictionary of action topic names for each move group

    _joint_names={}                     # Dictionary of joint name list dictionaries per move_group per topic

    def __init__(self,
                 robot_description="/robot_description",
                 semantic_description=None,
                 move_group_capabilities="/move_group",
                 wait_duration=0.5):

        """
        Creates a proxy for RobotCommander using a particular robot description.
        @param robot_description : string name of robot description (default: "robot_description")
        @param semantic_description : string name of robot semantic description (default: "None")
        @param move_group_capabilities : string or dictionary of topics by group name
        @param wait_duration : how long to wait for action server to become available
        """
        if (robot_description is None):
            #Logger.loginfo("ProxyMoveItClient - calling with robot_description is None " )
            try:
                if ProxyMoveItClient._robot_commander is None:
                    Logger.loginfo("ProxyMoveItClient - No robot commander is currently initialized!" )
                else:
                    Logger.loginfo("ProxyMoveItClient - Using existing RobotCommander instance with %s" % (ProxyMoveItClient._robot_description))
            except:
                Logger.loginfo("ProxyMoveItClient global not initialized yet " )

        elif (ProxyMoveItClient._robot_commander is None):
            Logger.loginfo("Initializing robot data for proxy MoveIt client with %s ..." % robot_description)
            moveit_commander.roscpp_initialize(sys.argv)

            Logger.loginfo("    Loading robot commander for %s" % (robot_description))
            ProxyMoveItClient._robot_commander = moveit_commander.RobotCommander(robot_description)
            ProxyMoveItClient._robot_description = robot_description

            # Load the semantic description as element tree to provide more direct access to data
            if semantic_description is None:
                semantic_description = robot_description+"_semantic"

            Logger.loginfo("    Loading semantic description parameter %s" % (semantic_description))
            ProxyMoveItClient._semantic_description = self.load_semantic_description(semantic_description)


            # Get the active move groups for this set up
            ProxyMoveItClient._move_group_list = ProxyMoveItClient._robot_commander.get_group_names()
            Logger.loginfo("    Available move groups %s for this robot on proxy MoveIt client ..." % (str(ProxyMoveItClient._move_group_list)))

            for group in ProxyMoveItClient._move_group_list:
                joint_names = ProxyMoveItClient._robot_commander.get_joint_names(group)
                ProxyMoveItClient._joint_names[group] = [name for name in joint_names]
                Logger.loginfo("         %s : %s " % (group, str(ProxyMoveItClient._joint_names[group])))

                # Set the default data for this move group
                self.set_global_defaults(group)

                try:
                    action_topic=None
                    if isinstance(move_group_capabilities, dict) and group in move_group_capabilities:
                        # Allow for a dictionary of action topics per move group
                        action_topic = move_group_capabilities[group]
                    else: # Should be a single string for default single move group capability
                        action_topic = move_group_capabilities

                    if not isinstance(action_topic,str):
                        raise Exception(" ProxyMoveItClient - Invalid action topic %s " % ( action_topic ))

                    if action_topic not in ProxyMoveItClient._action_clients:
                        # Defining a new MoveGroupAction client for this move group
                        Logger.loginfo("Initializing proxy MoveGroup client for "+action_topic+" ...")

                        ProxyMoveItClient._action_clients[action_topic] = ProxyActionClient({action_topic: eval("MoveGroupAction") }, wait_duration)

                    # Store the topic name of the client reference in per move group dictionary as well
                    ProxyMoveItClient._move_group_clients_[group] = action_topic

                except Exception as e:
                    Logger.logwarn("ProxyMoveItClient MoveGroupAction client setup error - %s"% (str(e)))
                    raise e


        else:
            Logger.loginfo("Already initialized robot data for proxy MoveIt client - new call with %s ..." % robot_description)
            if robot_description is not None and robot_description != ProxyMoveItClient._robot_description:
                Logger.logerr(" Using existing Proxy with robot description %s vs. %s " % ( ProxyMoveItClient._robot_description, robot_description))
                raise Exception(" ProxyMoveItClient - Requested Proxy with diffent robot description %s vs. %s - not supported!" % ( ProxyMoveItClient._robot_description, robot_description))


    def load_semantic_description(self, semantic_description):
        '''
        Load semantic description from the parameter server for later retrieval
        '''
        if semantic_description is None:
            Logger.logwarn(" No semantic description file loaded !")
            return None


        # Check existence of SRDF parameter.
        # Values are read during runtime to allow modifications.
        srdf_param = None
        if rospy.has_param(semantic_description):
            srdf_param = rospy.get_param(semantic_description)
        else:
            Logger.logwarn('Unable to get SRDF parameter - %s' % (semantic_description ))
            return None


        try:
            ProxyMoveItClient._srdf_ET = ET.fromstring(srdf_param)
            Logger.loginfo("     Loaded %s SRDF file " % (semantic_description))
        except Exception as e:
            Logger.logerr('Unable to parse given semantic description (SRDF) file - %s\n%s' % (semantic_description, str(e)))
            return None

        try:
            for r in ProxyMoveItClient._srdf_ET.iter('robot'):
                if ProxyMoveItClient._robot_name is None:
                    ProxyMoveItClient._robot_name = r.attrib['name']
                    ProxyMoveItClient._robot_semantics = r
                    Logger.loginfo("     Found robot %s in SRDF" % (ProxyMoveItClient._robot_name))
                else:
                    Logger.loginfo("     additional robot %s in SRDF - not used at this time " % (r.attrib['name']))
        except Exception as e:
            Logger.logerr('Unable to parse given semantic description (SRDF) element tree for robot - %s\n%s' % (semantic_description, str(e)))
            return None


    def get_group_state(self, config_name, group_name, robot_name=None ):
        '''
        Get named configuration from semantic description
        @param config_name  string name of configuration
        @param group_name  string   move group name that the configuration belongs to
        @param robot_name   string  robot (default uses current or first robot in SRDF)
        '''
        config = None
        try:
            # Find the designated robot (or first in file)

            if (robot_name is not None and robot_name != ProxyMoveItClient._robot_name):
                for r in ProxyMoveItClient._srdf_ET.iter('robot'):
                    if robot_name == r.attrib['name']:
                        ProxyMoveItClient._robot_name = r.attrib['name']
                        ProxyMoveItClient._robot_semantics = r
                        Logger.loginfo("   Switching proxy to robot %s " % (ProxyMoveItClient._robot_name) )

            if ProxyMoveItClient._robot_semantics is None:
                Logger.logwarn('Did not find robot name in SRDF - %s' % robot_name)
                return None


            # Find the desired configuration
            for c in ProxyMoveItClient._robot_semantics.iter('group_state'):
                if (group_name is None or group_name == '' or group_name == c.attrib['group']) \
                    and c.attrib['name'] == config_name:
                    config = c
                    break

            return config

        except Exception as e:
            Logger.logwarn('Unable to find named configuration in SRDF -\n%s' % str(e))
            return None


    def set_global_defaults( self, group ):
        """
        Initializes the proxy with default parameters to use with connections
        @param group : string move group name
        """

        # Default planning parameters for new connections
        num_planning_attempts=5
        allowed_planning_time=10.0
        plan_only=True
        planner_id='RRTConnectkConfigDefault'

        # Set the default plan request data
        plan_request = MotionPlanRequest()
        plan_request.start_state.is_diff = True # Flags start state as empty to use current state on server
        plan_request.num_planning_attempts  = num_planning_attempts
        plan_request.allowed_planning_time  = allowed_planning_time
        plan_request.planner_id             = planner_id
        plan_request.group_name             = group

        #@TODO - retrieve from ROS parameters
        ProxyMoveItClient._motion_plan_requests[group] = plan_request
        ProxyMoveItClient._default_motion_plan_requests[group] = copy.deepcopy(plan_request)

        planning_options = PlanningOptions()
        planning_options.plan_only     = plan_only
        planning_options.planning_scene_diff.robot_state.is_diff = True # Flags start state as empty to use current state on server

        #@TODO - retrieve from ROS parameters

        ProxyMoveItClient._planning_options[group] = planning_options
        ProxyMoveItClient._default_planning_options[group] = copy.deepcopy(planning_options)

        # Set up the default joint constraints
        joint_constraints = {}
        for name in ProxyMoveItClient._joint_names[group]:
            joint_constraints[name] = JointConstraint(joint_name=name, tolerance_above=0.05, tolerance_below=0.05,weight=1.0)

        #@TODO - retrieve from ROS parameters

        ProxyMoveItClient._joint_constraints[group]         = joint_constraints
        ProxyMoveItClient._default_joint_constraints[group] = copy.deepcopy(joint_constraints)

        # @TODO - add real constraints
        ProxyMoveItClient._position_constraints[group]            = []
        ProxyMoveItClient._default_position_constraints[group]    = []
        ProxyMoveItClient._orientation_constraints[group]         = []
        ProxyMoveItClient._default_orientation_constraints[group] = []
        ProxyMoveItClient._visibility_constraints[group]          = []
        ProxyMoveItClient._default_visibility_constraints[group]  = []

        ProxyMoveItClient._robot_states[group]          = RobotState()
        ProxyMoveItClient._default_robot_states[group]  = RobotState()

    def setup_multiple_action_clients(self, action_topics,  wait_duration=2.0):
        """
        Tries to set up a MoveIt MoveGroup action client for calling it later.
        @param action_topics : list of tuples of Action type and topic names
        @param wait_duration: Defines how long to wait for the given client if it is not available right now.
        """
        if action_topics is not None:
            print "ProxyMoveItClient: setup_multiple_action_clients ..."
            for action_type, topics in action_topics:
                print "Action type: ",action_type," : ",topics
                for topic in topics:
                    self.setup_action_client(topic, action_type, wait_duration)
        else:
            print "ProxyMoveItClient: no additional action clients"

    def setup_action_client(self, action_topic, action_type, wait_duration):
        """
        Tries to set up a MoveIt MoveGroup action client for calling it later.

        @param action_topic  : string - The topic of the action to call.
        @param action_type   : string - class definition name for interface
        @param wait_duration : Defines how long to wait for the given client if it is not available right now.
        """

        if not isinstance(action_topic,str):
            raise Exception(" ProxyMoveItClient - Invalid action topic %s " % ( action_topic ))

        if action_topic not in ProxyMoveItClient._action_clients:
            # We have not initialized this client yet
            Logger.loginfo("Initializing proxy MoveIt client for "+action_topic+" ...")

            try:
                ProxyMoveItClient._action_clients[action_topic] = ProxyActionClient({action_topic: eval(action_type) }, wait_duration)
            except Exception as e:
                Logger.logwarn("ProxyMoveItClient setup error - %s"% (str(e)))
                raise e

    def dump_moveit_client(self):
        '''
        This dumps the ProxyMoveItClient data for debugging and status
        '''
        data  = "\n*******************************\n"
        data += "------ ProxyMoveItClient ------\n"
        data += " Robot: "+ProxyMoveItClient._robot_description+"\n"
        data += " --- Action Clients ---\n"
        for topic in ProxyMoveItClient._action_clients:
            data += "  client: "+topic+"\n"
        data += "----------------------------\n"
        for group in ProxyMoveItClient._move_group_list:
            data += str(self.dump_moveit_goal(group))
            data += "----------------------------\n\n"
        data += "--------------------------------\n"
        data += "################################\n"
        return data

    def dump_moveit_goal(self, group):
        '''
        This dumps the current MoveGroupGoal data for debugging and status
        @param group : string for a specific move group or
        '''
        data  = "    group: "+group+"\n"
        data += "    --- MotionPlanRequest ---\n"
        data += "        "+str(ProxyMoveItClient._motion_plan_requests[group])
        data += "\n    ----------------------------\n"
        data += "    --- PlanningOption ---\n"
        data += "        "+str(ProxyMoveItClient._planning_options[group])
        data += "\n    ----------------------------\n"

        return data

    def get_error_msg(self, error_code):
        '''
        Returns error message string based on
        @param error_code : code returned from MoveIt!
        '''
        for key, value in MoveItErrorCodes.__dict__.items():
            if value == error_code.val and key[0] != "_":
                return key + (" (%s)" % str(error_code.val))
        return "unknown error (%s)" % str(error_code.val)


    def is_available(self, action_topic):
        """
        Checks if the client on the given action topic is available.
        @param topic: string The topic of interest.
        """
        if(not action_topic in ProxyMoveItClient._action_clients):
            raise Exception("ProxyMoveItClient - topic %s is not initialized yet!" % (action_topic))

        return ProxyMoveItClient._action_clients[action_topic].is_available(action_topic)

    def get_state(self, action_topic):
        """
        Returns current state of the ActionLib client.
        @param topic: string The topic of interest.
        """
        if(not action_topic in ProxyMoveItClient._action_clients):
            raise Exception("ProxyMoveItClient - topic %s is not initialized yet!" % (action_topic))

        return ProxyMoveItClient._action_clients[action_topic].get_state(action_topic)

    def has_result(self, action_topic):
        """
        Checks if the client on the given action topic has an active result.
        @param topic: string The topic of interest.
        """
        if(not action_topic in ProxyMoveItClient._action_clients):
            raise Exception("ProxyMoveItClient - topic %s is not initialized yet!" % (action_topic))

        return ProxyMoveItClient._action_clients[action_topic].has_result(action_topic)

    def get_result(self, action_topic):
        """
        Gets that latest result from the given action topic.
        @param topic: string The topic of interest.
        """
        if(not action_topic in ProxyMoveItClient._action_clients):
            raise Exception("ProxyMoveItClient - topic %s is not initialized yet!" % (action_topic))

        return ProxyMoveItClient._action_clients[action_topic].get_result(action_topic)

    def cancel(self, action_topic):
        """
        Cancel any active goals on the given action topic
        @param topic: string The topic of interest.
        """
        if(not action_topic in ProxyMoveItClient._action_clients):
            raise Exception("ProxyMoveItClient - topic %s is not initialized yet!" % (action_topic))

        return ProxyMoveItClient._action_clients[action_topic].cancel(action_topic)


    def connect_action_server(self, action_topic, action_type, wait_duration=0.0):
        """
        Checks if the client on the given action topic is available.
        @param action_topic: string The topic of interest.
        @param action_type:  string The interface type
        @param wait_duration : float  How long to wait for a connection
        """
        if not action_topic in ProxyMoveItClient._action_clients:
            raise Exception("ProxyMoveItClient - topic %s is not initialized yet!" % (action_topic))

        ProxyMoveItClient._action_clients[action_topic].setupClient(action_topic, eval(action_type) , wait_duration)
        return ProxyMoveItClient._action_clients[action_topic].is_available(action_topic)

    def is_active(self, action_topic):
        """
        Determines if an action request is already being processed on the given topic.

        @type topic: string
        @param topic: The topic of interest.
        """
        if not action_topic in ProxyMoveItClient._action_clients:
            raise Exception("ProxyMoveItClient - is_active topic %s is not initialized yet!" % (action_topic))

        return ProxyMoveItClient._action_clients[action_topic].is_active(action_topic)


    def reset_motion_plan_request(self, move_group=None):
        '''
        Clears the specified dictionary for move group

        @param move_group     : string or list specifying a particular move group(s) (default: None - change all relevant )
        '''
        flag = False
        if move_group is None:
            for group in self._move_group_list:
                try:
                    ret = self.reset_motion_plan_request(group)
                    if ret:
                        flag=ret
                except:
                    pass
            # Check to see that something was reset
            if flag:
                return flag
            else:
                raise Exception(" Failed to reset dictionary for any relevant topic for move_group=(%s) !" % (str(move_group)))

        elif bool(move_group) and all([isinstance(elem,basestring) for elem in move_group]):
            # List of strings
            for group in move_group:
                try:
                    ret = self.reset_motion_plan_request(group)
                    if ret:
                        flag=ret
                except:
                    pass
            # Check to see that something was reset
            if flag:
                return flag
            else:
                raise Exception(" Failed to reset dictionary for any relevant topic for move_group=(%s) !" % (str(move_group)))
        else:
            # Base case to reset specified motion plan requests options
            try:
                # Reset the planning options for a given move group
                ProxyMoveItClient._motion_plan_requests[move_group] = copy.deepcopy(ProxyMoveItClient._default_motion_plan_requests[move_group])
                return True
            except Exception as e:
                Logger.logerr(" Invalid move group %s for motion plan request  - not configured yet!" % (str(move_group)))
                return False

    def reset_planning_options(self, move_group=None):
        '''
        Reset the planning options to match the defaults
        @param move_group     : string or list specifying a particular move group(s) (default: None - change all relevant )
        '''

        flag = False
        if move_group is None:
            for group in self._move_group_list:
                try:
                    ret = self.reset_planning_options(group)
                    if ret:
                        flag=ret
                except:
                    pass
            # Check to see that something was reset
            if flag:
                return flag
            else:
                raise Exception(" Failed to reset planning options for any relevant topic for move_group=(%s) !" % (str(move_group)))

        elif bool(move_group) and all([isinstance(elem,basestring) for elem in move_group]):
            # List of strings
            for group in move_group:
                try:
                    ret = self.reset_planning_options(group)
                    if ret:
                        flag=ret
                except:
                    pass
            # Check to see that something was reset
            if flag:
                return flag
            else:
                raise Exception(" Failed to reset planning options for any relevant topic for move_group=(%s) !" % (str(move_group)))
        else:
            # Base case to reset specified motion plan requests options
            try:
                # Reset the planning options for a given move group
                ProxyMoveItClient._planning_options[move_group] = copy.deepcopy(ProxyMoveItClient._default_planning_options[move_group])
                return True
            except Exception as e:
                Logger.logerr(" Invalid move group %s for planning options  - not configured yet!" % (str(move_group)))
                return False

    def reset_joint_constraints(self, move_group=None):
        '''
        Reset the planning options to match the defaults
        @param move_group     : string or list specifying a particular move group(s) (default: None - change all relevant )
        '''

        flag = False
        if move_group is None:
            for group in self._move_group_list:
                try:
                    ret = self.reset_joint_constraints(group)
                    if ret:
                        flag=ret
                except:
                    pass
            # Check to see that something was reset
            if flag:
                return flag
            else:
                raise Exception(" Failed to reset joint constraints for any relevant topic for move_group=(%s) !" % (str(move_group)))

        elif bool(move_group) and all([isinstance(elem,basestring) for elem in move_group]):
            # List of strings
            for group in move_group:
                try:
                    ret = self.reset_joint_constraints(group)
                    if ret:
                        flag=ret
                except:
                    pass
            # Check to see that something was reset
            if flag:
                return flag
            else:
                raise Exception(" Failed to reset joint constraints for any relevant topic for move_group=(%s) !" % (str(move_group)))
        else:
            # Base case to reset specified motion plan requests options
            try:
                # Reset the planning options for a given move group
                ProxyMoveItClient._joint_constraints[move_group] = copy.deepcopy(ProxyMoveItClient._default_joint_constraints[move_group])
                return True
            except Exception as e:
                Logger.logerr(" Invalid move group %s for joint constraints - not configured yet!" % (str(move_group)))
                return False

    def reset_position_constraints(self, move_group):
        '''
        Reset the current position constraints to the defaults
        @param move_group           : string specifying a particular move group
        '''
        #@TODO - handle reset all as with joints
        ProxyMoveItClient._position_constraints[move_group] = copy.deepcopy(ProxyMoveItClient._default_position_constraints[move_group])

    def reset_orientation_constraints(self, move_group):
        '''
        Reset the current orientation constraints to the defaults
        @param move_group  : string specifying a particular move group
        '''
        #@TODO - handle reset all as with joints
        ProxyMoveItClient._orientation_constraints[move_group] = copy.deepcopy(ProxyMoveItClient._default_orientation_constraints[move_group])

    def reset_visibility_constraints(self, move_group):
        '''
        Reset the current orientation constraints to the defaults
        @param move_group  : string specifying a particular move group
        '''
        #@TODO - handle reset all as with joints
        ProxyMoveItClient._visibility_constraints[move_group] = copy.deepcopy(ProxyMoveItClient._default_visibility_constraints[move_group])


    def get_goal_joint_constraints(self, move_group, joint_values, joint_names=None):
        '''
        Creates a JointConstraint for  list of goal constraints, and creates a new goal based on list
        of joint constraints for the current plan request specification

        @param move_group   : string specifying a particular move group
        @param joint_values : list of joint values (must match the given joint names)
        @param joint_names  : list of string joint names (default: None uses all joints in group)
        '''

        if joint_names is None:
            joint_names = ProxyMoveItClient._joint_names[move_group]

        if len(joint_names) != len(joint_values):
            Logger.logerr("Amount of given joint values (%d) does not match the amount of joints (%d) in group %s." % (len(joint_values), len(joint_names), move_group))
            raise Exception("Amount of given joint values (%d) does not match the amount of joints (%d) in group %s." % (len(joint_values), len(joint_names), move_group))

        joint_constraints = []
        for i, name in enumerate(joint_names):
            constraint = copy.deepcopy(ProxyMoveItClient._joint_constraints[move_group][name])
            constraint.position = joint_values[i]
            joint_constraints.append( constraint )

        return joint_constraints


    def set_position_constraints(self, move_group, position_constraints):
        '''
        Update the current position constraints for a given move group
        @param move_group           : string specifying a particular move group
        @param position_constraints : list of position constraints
        '''
        ProxyMoveItClient._position_constraints[move_group] = position_constraints

    def set_orientation_constraints(self, move_group, orientation_constraints):
        '''
        Update the current position constraints for a given move group
        @param move_group              : string specifying a particular move group
        @param orientation_constraints : list of orientation constraints
        '''
        ProxyMoveItClient._orientation_constraints[move_group] = orientation_constraints

    def set_visibility_constraints(self, move_group, visibility_constraints):
        '''
        Update the current position constraints for a given move group
        @param move_group              : string specifying a particular move group
        @param visibility_constraints : list of visibility constraints
        '''
        ProxyMoveItClient._visibility_constraints[move_group] = visibility_constraints

    def get_joint_constraints(self, move_group):
        '''
        Retrieve the current joint constraints as a list for a given move group
        @param move_group           : string specifying a particular move group
        '''
        return copy.deepcopy(ProxyMoveItClient._joint_constraints[move_group].values())

    def get_position_constraints(self, move_group):
        '''
        Retrieve the current position constraints for a given move group
        @param move_group           : string specifying a particular move group
        '''
        return copy.deepcopy(ProxyMoveItClient._position_constraints[move_group])

    def get_orientation_constraints(self, move_group):
        '''
        Retrieve the current orientation constraints for a given move group
        @param move_group           : string specifying a particular move group
        '''
        return copy.deepcopy(ProxyMoveItClient._orientation_constraints[move_group])

    def get_visibility_constraints(self, move_group):
        '''
        Retrieve the current visibility constraints for a given move group
        @param move_group           : string specifying a particular move group
        '''
        return copy.deepcopy(ProxyMoveItClient._visibility_constraints[move_group])


    def get_constraints(self, move_group):
        '''
        Creates a Constraints structure for the specified move_group based on
        current stored information.

        @param move_group   : string specifying a particular move group
        '''

        constraints = Constraints()
        constraints.joint_constraints        = self.get_joint_constraints(move_group)
        constraints.position_constraints     = self.get_position_constraints(move_group)
        constraints.orientation_constraints  = self.get_orientation_constraints(move_group)
        constraints.visibility_constraints   = self.get_visibility_constraints(move_group)
        return constraints

    def set_robot_state(self, move_group, robot_state):
        #@TODO - fix this
        Logger.logerr("ProxyMoveItClient: set_robot_state not finished!")
        pass

    def get_robot_state(self, move_group):
        '''
        Retrieve the current robot state structure for this move group
        @param move_group           : string specifying a particular move group
        '''
        return copy.deepcopy(ProxyMoveItClient._robot_states[move_group])

    def set_robot_start_state(self, move_group, robot_state):
        '''
        Store the robot state used at start of trajectory for this move group
        @param move_group           : string specifying a particular move group
        @param robot_state          : robot state data structure
        '''
        ProxyMoveItClient._robot_start_states[move_group] = robot_state


    def get_robot_start_state(self, move_group):
        '''
        Retrieve the current robot state structure for this move group
        @param move_group           : string specifying a particular move group
        '''
        return copy.deepcopy(ProxyMoveItClient._robot_start_states[move_group])

    def send_move_group_joint_space_goal(self, move_group, joint_names, joint_values, plan_only=None):
        '''
        Retrieve the current move group action configuration and send the move request

        @param move_group           : string specifying a particular move group
        @param joint_names          : list of joint names for command
        @param joint_values         : list of joint values for target
        '''

        # We should have already set this pairing up during initialization
        action_topic = ProxyMoveItClient._move_group_clients_[move_group]

        # Get the latest setup of of the MoveGroup goal data
        goal = MoveGroupGoal()
        goal.request          = copy.deepcopy(ProxyMoveItClient._motion_plan_requests[move_group])
        goal.planning_options = copy.deepcopy(ProxyMoveItClient._planning_options[move_group])

        if (plan_only is not None):
            goal.planning_options.plan_only = plan_only

        goal_constraints = Constraints()
        for i in range(len(joint_names)):
                jc = ProxyMoveItClient._joint_constraints[move_group][joint_names[i]]
                goal_constraints.joint_constraints.append(JointConstraint(
                        joint_name=joint_names[i],
                        position=joint_values[i],
                        tolerance_above=jc.tolerance_above,
                        tolerance_below=jc.tolerance_above,
                        weight=jc.weight))

        goal.request.goal_constraints.append(goal_constraints)

        ProxyMoveItClient._action_clients[action_topic].send_goal(action_topic, goal)
