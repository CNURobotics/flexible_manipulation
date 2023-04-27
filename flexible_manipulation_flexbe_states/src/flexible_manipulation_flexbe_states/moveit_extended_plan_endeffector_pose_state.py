#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

from flexbe_core import EventState, Logger
from flexible_manipulation_flexbe_states.proxy import ProxyMoveItClient

'''
Created on 2-Feb-2018

@author: David Conner based on code by Philipp Schillinger
'''

class MoveItPlanEndeffectorPoseState(EventState):
    '''
    Plans trajectory to reach the given pose with the specified endeffector

    This version uses the base MoveIt capabilities.

    -- ignore_collisions     boolean            Should collisions be ignored? Only pass True if you are sure that it is safe.
    -- include_torso         boolean            Should the torso be included in the trajectory or stay fixed?
    -- allowed_collisions    tuple[]            List of 2-tuples (strings) for each pair of links which is allowed to collide.
                                                Set the second value to an empty string if collision to any other link is allowed.

    -- planner_id            string             Sets the ID of the planner to use (MoveIt planner id - default = "RRTConnectkConfigDefault")

    ># move_group            string             Move group (e.g. arm) used for planning
    ># target_pose           PoseStamped        Target pose to reach.

    #> joint_trajectory      JointTrajectory    Trajectory of the end effector to perform the requested motion.

    <= planned                                  Was able to generate a valid motion plan.
    <= failed                                   Failed to create a motion plan at all.
    <= param_error                              Configuration error - did not request plan

    '''

    def __init__(self, ignore_collisions = False, include_torso = False, allowed_collisions = [],
                 planner_id = "RRTConnectkConfigDefault", action_topic="/move_group"):
        '''
        Constructor
        '''
        super(MoveItPlanEndeffectorPoseState, self).__init__(outcomes=['planned', 'failed', 'param_error'],
                                                             input_keys=['move_group', 'target_pose'],
                                                             output_keys=['joint_trajectory'])

        self.client = ProxyMoveItClient(None)
        self.action_topic = action_topic
        self.ignore_collisions = ignore_collisions
        self.include_torso = include_torso
        self.allowed_collisions = allowed_collisions
        self.return_code = None
        self.planner_id = planner_id

        # Logger.logerr("The MoveItPlanEndeffectorPoseState is out of whack")
        # throw "The MoveItPlanEndeffectorPoseState is out of whack"

    def execute(self, userdata):
        '''
        Execute this state
        '''

        if self.return_code is not None:
            return self.return_code

        if self.client.finished():
            #@todo - log status from action client
            userdata.joint_trajectory = self.client.get_plan()
            if self.client.success():
                self.return_code = 'planned'
                return self.return_code
            else:
                Logger.logwarn('MoveIt failed with the following error - %s' % self.client.error_msg())
                self.return_code = 'failed'
                return self.return_code


    def on_enter(self, userdata):

        print(" Userdata in: ", userdata)
        target_pose = PoseStamped()
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'jackal/map'
        target_pose.header = header
        target_pose.pose.position = Point(0, 0, 0.5)
        target_pose.pose.orientation = Quaternion(0, 0, 0, 1)
        print(" pose = ", target_pose)
        #userdata["target_pose"] = pose_stamped
        #print(" Userdata out: ", userdata)



        if not hasattr(userdata, 'target_pose') or userdata.target_pose is None:
            self.return_code = 'param_error'
            Logger.logwarn('Userdata "target_pose" of state %s does not exist or is currently undefined!' % self.name)
            return
        if not hasattr(userdata, 'move_group') or userdata.move_group is None:
            self.return_code = 'param_error'
            Logger.logwarn('Userdata "move_group" of state %s does not exist or is currently undefined!' % self.name)
            return


        self.return_code = None
        try:
            # create the motion goal
            self.client.new_goal(userdata.move_group)
            self.client.add_endeffector_pose(target_pose.pose, target_pose.header.frame_id)
            self.client.set_collision_avoidance(self.ignore_collisions)
            self.client.set_planner_id(self.planner_id)

            for link, target in self.allowed_collisions:
                self.client.set_allowed_collision(link, target)

            # for later use
            #for link, target in self.allowed_collisions:

            Logger.loginfo('Sending planning request to reach point (%f, %f, %f) in frame %s...' % (target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z, target_pose.header.frame_id))
            self.client.start_planning()

        except Exception as e:
            Logger.logwarn('Could not request a plan!\n%s' % str(e))
            self.return_code = 'param_error'


    def on_exit(self, userdata):
        Logger.logwarn(" Need to check %s - exit condition" % self.name)
        #if not self.client.finished():
        #    self.client.cancel()
        #    Logger.loginfo("Cancelled active action goal.")
