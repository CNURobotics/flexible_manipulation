/*********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018
 * Capable Humanitarian Robotics and Intelligent Systems Lab (CHRISLab)
 * Christopher Newport University
 *
 *  Copyright (c) 2016, Michael 'v4hn' Goerner
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Christopher Newport University, CHRISLab,
 *     Willow Garage nor the names of its contributors may be used to endorse
 *     or promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Michael 'v4hn' Goerner and David Conner */

#ifndef FLEXIBLE_MANIPULATION_GROUP_APPLY_PLANNING_SCENE_CAPABILITY_
#define FLEXIBLE_MANIPULATION_GROUP_APPLY_PLANNING_SCENE_CAPABILITY_

#include <actionlib/server/simple_action_server.h>
#include <flexible_manipulation_msgs/ApplyPlanningSceneAction.h>
#include <moveit/move_group/move_group_capability.h>
#include <moveit_msgs/ApplyPlanningScene.h>

namespace flexible_manipulation
{
/**
 * Provides the ability to update the shared planning scene
 * with a remote blocking call using a ROS-Service
 */
class ApplyPlanningSceneCapability : public move_group::MoveGroupCapability
{
public:
  ApplyPlanningSceneCapability();

  void initialize() override;

private:
  // Action interface
  void executeCallback(const flexible_manipulation_msgs::ApplyPlanningSceneGoalConstPtr& goal);
  void preemptCallback();

  std::unique_ptr<actionlib::SimpleActionServer<flexible_manipulation_msgs::ApplyPlanningSceneAction>> action_server_;

  // Service interface
  bool applyScene(moveit_msgs::ApplyPlanningScene::Request& req, moveit_msgs::ApplyPlanningScene::Response& res);

  ros::ServiceServer service_;
};
}

#endif  // FLEXIBLE_MANIPULATION_GROUP_APPLY_PLANNING_SCENE_SERVICE_CAPABILITY_
