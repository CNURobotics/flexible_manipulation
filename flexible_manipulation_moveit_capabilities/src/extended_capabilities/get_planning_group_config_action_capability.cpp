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

#include "get_planning_group_config_action_capability.h"

#include <moveit/planning_pipeline/planning_pipeline.h>

namespace flexible_manipulation
{
  GetPlanningGroupConfigAction::GetPlanningGroupConfigAction()
  : move_group::MoveGroupCapability("GetPlanningGroupConfigAction")
{
}

void GetPlanningGroupConfigAction::initialize()
{
  gpgc_action_server_.reset(new actionlib::SimpleActionServer<flexible_manipulation_msgs::GetPlanningGroupConfigAction>(
      root_node_handle_, "get_planning_group_config",
      boost::bind(&GetPlanningGroupConfigAction::executeCallback, this, _1), false));
  gpgc_action_server_->start();
}

// @todo - convert to Action server interface
void GetPlanningGroupConfigAction::executeCallback(
    const flexible_manipulation_msgs::GetPlanningGroupConfigGoalConstPtr& goal)
{
  const robot_model::RobotModelConstPtr& robot_model = context_->planning_pipeline_->getRobotModel();

  if (!robot_model->hasJointModelGroup(goal->group_name))
  {
    ROS_ERROR_NAMED(getName(), "Service request for joint states of invalid group %s", goal->group_name.c_str());
    result_.positions.clear();
    gpgc_action_server_->setAborted(result_, "invalid group name " + goal->group_name);
    return;
  }

  const robot_state::JointModelGroup* joint_model_group = robot_model->getJointModelGroup(goal->group_name);

  {
    planning_scene_monitor::LockedPlanningSceneRO ps(context_->planning_scene_monitor_);
    const robot_state::RobotState& curr_state =
        context_->planning_scene_monitor_->getPlanningScene()->getCurrentState();
    curr_state.copyJointGroupPositions(joint_model_group, result_.positions);
  }

  gpgc_action_server_->setSucceeded(result_, "");

  return;
}
}  // namespace flexible_manipulation

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(flexible_manipulation::GetPlanningGroupConfigAction, move_group::MoveGroupCapability)
