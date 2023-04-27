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
 *   * Neither the name of Christopher Newport University, CHRISLab, or
 *     Willow Garage  nor the names of its contributors
 *     may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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

/* Author: Ioan Sucan and David Conner */

#include "move_group_plan_capability.h"
#include <moveit/move_group/capability_names.h>
#include <moveit/planning_pipeline/planning_pipeline.h>

namespace flexible_manipulation
{
  MoveGroupPlanCapability::MoveGroupPlanCapability()
    : move_group::MoveGroupCapability("MoveGroupPlanCapability")
{
}

void MoveGroupPlanCapability::initialize()
{
  plan_service_ = root_node_handle_.advertiseService(
      move_group::PLANNER_SERVICE_NAME, &flexible_manipulation::MoveGroupPlanCapability::computePlanService, this);

  // start the move action server
  action_server_.reset(new actionlib::SimpleActionServer<flexible_manipulation_msgs::GetMotionPlanAction>(
      root_node_handle_, move_group::PLANNER_SERVICE_NAME,
      boost::bind(&flexible_manipulation::MoveGroupPlanCapability::executeCallback, this, _1), false));
  action_server_->start();
}

// Action interface makes use of the service interface
void MoveGroupPlanCapability::executeCallback(
    const flexible_manipulation_msgs::GetMotionPlanGoalConstPtr& goal)
{
  flexible_manipulation_msgs::GetMotionPlanResult action_res;

  ROS_INFO_NAMED(getName(), "Received new planning action request...");
  // before we start planning, ensure that we have the latest robot state
  // received...
  context_->planning_scene_monitor_->waitForCurrentRobotState(ros::Time::now());
  context_->planning_scene_monitor_->updateFrameTransforms();

  bool solved = false;
  planning_scene_monitor::LockedPlanningSceneRO ps(context_->planning_scene_monitor_);
  try
  {
    planning_interface::MotionPlanResponse mp_res;
    context_->planning_pipeline_->generatePlan(ps, goal->motion_plan_request, mp_res);
    mp_res.getMessage(action_res.motion_plan_response);
  }
  catch (std::exception& ex)
  {
    ROS_ERROR_NAMED(getName(), "Planning pipeline threw an exception: %s", ex.what());
    action_res.motion_plan_response.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  }

  if (action_res.motion_plan_response.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    action_server_->setSucceeded(action_res, "");
  }
  else
  {
    action_server_->setAborted(action_res, "Failed to GetMotionPlan");
  }
}

bool MoveGroupPlanCapability::computePlanService(moveit_msgs::GetMotionPlan::Request& req,
                                                                        moveit_msgs::GetMotionPlan::Response& res)
{
  ROS_INFO_NAMED(getName(), "Received new planning service request...");
  // before we start planning, ensure that we have the latest robot state
  // received...
  context_->planning_scene_monitor_->waitForCurrentRobotState(ros::Time::now());
  context_->planning_scene_monitor_->updateFrameTransforms();

  bool solved = false;
  planning_scene_monitor::LockedPlanningSceneRO ps(context_->planning_scene_monitor_);
  try
  {
    planning_interface::MotionPlanResponse mp_res;
    context_->planning_pipeline_->generatePlan(ps, req.motion_plan_request, mp_res);
    mp_res.getMessage(res.motion_plan_response);
  }
  catch (std::exception& ex)
  {
    ROS_ERROR_NAMED(getName(), "Planning pipeline threw an exception: %s", ex.what());
    res.motion_plan_response.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  }

  return true;
}
} // end of namespace

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(flexible_manipulation::MoveGroupPlanCapability, move_group::MoveGroupCapability)
