/*********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018
 * Capable Humanitarian Robotics and Intelligent Systems Lab (CHRISLab)
 * Christopher Newport University
 *
 *  This code is based on code from DARPA Robotics Challenge Team ViGIR
 *  Copyright (c) 2014, Stefan Kohlbrecher, TU Darmstadt ( Team ViGIR )
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
 *   * Neither the name of Christopher Newport University, TU Darmstadt,
 *     Team ViGIR, nor the names of its contributors may be used to endorse
 *     or promote products derived from this software without specific prior
 *     written permission.
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

#include "continuous_replanning_capability.h"

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
#include <moveit_msgs/PlanningScene.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/conversions.h>

namespace flexible_manipulation
{
  ContinuousReplanningCapability::ContinuousReplanningCapability()
  : move_group::MoveGroupCapability("ContinuousReplanningCapability")
{
}

void ContinuousReplanningCapability::initialize()
{
  plan_execution_.reset(new plan_execution::ContinuousPlanExecution(context_));

  trigger_sub_ = root_node_handle_.subscribe("/trigger_cont", 1, &ContinuousReplanningCapability::triggerCb, this);
  abort_sub_ = root_node_handle_.subscribe("/abort_cont", 1, &ContinuousReplanningCapability::abortCb, this);
}

void ContinuousReplanningCapability::triggerCb(const std_msgs::Empty::ConstPtr& /*msg*/)
{
  ROS_INFO_NAMED(getName(), "Received trigger");

  plan_execution_->startExecution();
}

void ContinuousReplanningCapability::abortCb(const std_msgs::Empty::ConstPtr& /*msg*/)
{
  ROS_INFO_NAMED(getName(), "Received abort");

  plan_execution_->stopExecution();
}
}  // namespace flexible_manipulation

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(flexible_manipulation::ContinuousReplanningCapability, move_group::MoveGroupCapability)
