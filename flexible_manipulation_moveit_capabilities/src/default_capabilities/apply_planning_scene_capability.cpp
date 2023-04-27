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

/* Author: Michael Goerner and David Conner*/

#include "apply_planning_scene_capability.h"
#include <moveit/move_group/capability_names.h>

namespace flexible_manipulation
{
  ApplyPlanningSceneCapability::ApplyPlanningSceneCapability()
    : move_group::MoveGroupCapability("ApplyPlanningSceneCapability")
{
}

void ApplyPlanningSceneCapability::initialize()
{
  service_ = root_node_handle_.advertiseService(move_group::APPLY_PLANNING_SCENE_SERVICE_NAME,
                                                &flexible_manipulation::ApplyPlanningSceneCapability::applyScene, this);
  // start the move action server
  action_server_.reset(new actionlib::SimpleActionServer<flexible_manipulation_msgs::ApplyPlanningSceneAction>(
      root_node_handle_, move_group::APPLY_PLANNING_SCENE_SERVICE_NAME,
      boost::bind(&flexible_manipulation::ApplyPlanningSceneCapability::executeCallback, this, _1), false));
  action_server_->start();
}

void ApplyPlanningSceneCapability::executeCallback(
    const flexible_manipulation_msgs::ApplyPlanningSceneGoalConstPtr& goal)
{
  ROS_INFO(" Received planning scene update ...");
  flexible_manipulation_msgs::ApplyPlanningSceneResult action_res;
  if (!context_->planning_scene_monitor_)
  {
    action_res.success = false;
    action_server_->setAborted(action_res, "Cannot apply PlanningScene as no scene is monitored.");
    ROS_ERROR_NAMED(getName(), "Cannot apply PlanningScene as no scene is monitored.");
    return;
  }

  context_->planning_scene_monitor_->updateFrameTransforms();
  action_res.success = context_->planning_scene_monitor_->newPlanningSceneMessage(goal->scene);
  if (action_res.success)
  {
    ROS_INFO(" Planning scene update succeeded!");
    action_server_->setSucceeded(action_res, "");
  }
  else
  {
    ROS_ERROR_NAMED(getName(), " Failed to update the planning scene !");
    action_server_->setAborted(action_res, "Failed to apply PlanningScene");
  }
}

bool ApplyPlanningSceneCapability::applyScene(moveit_msgs::ApplyPlanningScene::Request& req,
                                                                     moveit_msgs::ApplyPlanningScene::Response& res)
{
  if (!context_->planning_scene_monitor_)
  {
    ROS_ERROR_NAMED(getName(), "Cannot apply PlanningScene as no scene is monitored.");
    return true;
  }
  context_->planning_scene_monitor_->updateFrameTransforms();
  res.success = context_->planning_scene_monitor_->newPlanningSceneMessage(req.scene);
  return true;
}
} // end of namespace

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(flexible_manipulation::ApplyPlanningSceneCapability, move_group::MoveGroupCapability)
