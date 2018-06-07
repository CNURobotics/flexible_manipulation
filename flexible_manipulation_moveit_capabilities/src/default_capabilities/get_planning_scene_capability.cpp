/*********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018
 * Capable Humanitarian Robotics and Intelligent Systems Lab (CHRISLab)
 * Christopher Newport University
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

/* Author: Ioan Sucan, David Conner, Julie Gates, and Jenny Gu */

#include "get_planning_scene_capability.h"
#include <moveit/move_group/capability_names.h>

flexible_manipulation::GetPlanningSceneCapability::GetPlanningSceneCapability()
  : move_group::MoveGroupCapability("GetPlanningSceneCapability")
{
}

void flexible_manipulation::GetPlanningSceneCapability::initialize()
{
  get_scene_service_ = root_node_handle_.advertiseService(
      move_group::GET_PLANNING_SCENE_SERVICE_NAME,
      &flexible_manipulation::GetPlanningSceneCapability::getPlanningSceneService, this);

  action_server_.reset(new actionlib::SimpleActionServer<flexible_manipulation_msgs::GetPlanningSceneAction>(
      root_node_handle_, move_group::GET_PLANNING_SCENE_SERVICE_NAME,
      boost::bind(&flexible_manipulation::GetPlanningSceneCapability::executeCallback, this, _1), false));
  action_server_->start();
}

void flexible_manipulation::GetPlanningSceneCapability::executeCallback(
    const flexible_manipulation_msgs::GetPlanningSceneGoalConstPtr& goal)
{
  flexible_manipulation_msgs::GetPlanningSceneResult res;
  if (!context_->planning_scene_monitor_)
  {
    action_server_->setAborted(res, "No planning scene monitor");
  }

  if ((goal->components.components & moveit_msgs::PlanningSceneComponents::TRANSFORMS) != 0u)
  {
    context_->planning_scene_monitor_->updateFrameTransforms();
  }

  planning_scene_monitor::LockedPlanningSceneRO ps(context_->planning_scene_monitor_);
  ps->getPlanningSceneMsg(res.scene, goal->components);

  action_server_->setSucceeded(res, "");
  return;
}

bool flexible_manipulation::GetPlanningSceneCapability::getPlanningSceneService(
    moveit_msgs::GetPlanningScene::Request& req, moveit_msgs::GetPlanningScene::Response& res)
{
  if ((req.components.components & moveit_msgs::PlanningSceneComponents::TRANSFORMS) != 0u)
  {
    context_->planning_scene_monitor_->updateFrameTransforms();
  }
  planning_scene_monitor::LockedPlanningSceneRO ps(context_->planning_scene_monitor_);
  ps->getPlanningSceneMsg(res.scene, req.components);
  return true;
}

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(flexible_manipulation::GetPlanningSceneCapability, move_group::MoveGroupCapability)
