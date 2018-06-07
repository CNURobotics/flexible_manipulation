/*********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018
 * Capable Humanitarian Robotics and Intelligent Systems Lab (CHRISLab)
 * Christopher Newport University
 *
 *  Copyright (c) 2016, Kentaro Wada.
 *  All rights reserved.

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

/* Author: Ioan Sucan, Kentaro Wada, and David Conner */

#include "execute_known_trajectory_capability.h"
#include <moveit/move_group/capability_names.h>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>

namespace flexible_manipulation
{
ExecuteKnownTrajectoryCapability::ExecuteKnownTrajectoryCapability()
  : move_group::MoveGroupCapability("ExecuteKnownTrajectoryCapability")
  , callback_queue_()
  , spinner_(1 /* spinner threads */, &callback_queue_)
{
}

ExecuteKnownTrajectoryCapability::~ExecuteKnownTrajectoryCapability()
{
  spinner_.stop();
}

void ExecuteKnownTrajectoryCapability::initialize()
{
  // We need to serve each service request in a thread independent of the main
  // spinner thread.
  // Otherwise, a synchronous execution request (i.e. waiting for the execution
  // to finish) would block
  // execution of the main spinner thread.
  // Hence, we use our own asynchronous spinner listening to our own callback
  // queue.
  ros::AdvertiseServiceOptions ops;
  ops.template init<moveit_msgs::ExecuteKnownTrajectory::Request, moveit_msgs::ExecuteKnownTrajectory::Response>(
      move_group::EXECUTE_SERVICE_NAME,
      boost::bind(&ExecuteKnownTrajectoryCapability::executeTrajectoryService, this, _1, _2));
  ops.callback_queue = &callback_queue_;
  execute_service_ = root_node_handle_.advertiseService(ops);
  spinner_.start();

  // start the move action server
  execute_action_server_.reset(
      new actionlib::SimpleActionServer<flexible_manipulation_msgs::ExecuteKnownTrajectoryAction>(
          root_node_handle_, move_group::EXECUTE_ACTION_NAME,
          boost::bind(&ExecuteKnownTrajectoryCapability::executePathCallback, this, _1), false));
  execute_action_server_->registerPreemptCallback(
      boost::bind(&ExecuteKnownTrajectoryCapability::preemptExecuteKnownTrajectoryCallback, this));
  execute_action_server_->start();
}

// Service interface
bool ExecuteKnownTrajectoryCapability::executeTrajectoryService(moveit_msgs::ExecuteKnownTrajectory::Request& req,
                                                                moveit_msgs::ExecuteKnownTrajectory::Response& res)
{
  ROS_INFO("Received new trajectory execution service request...");
  if (!context_->trajectory_execution_manager_)
  {
    ROS_ERROR("Cannot execute trajectory since ~allow_trajectory_execution was "
              "set to false");
    res.error_code.val = moveit_msgs::MoveItErrorCodes::CONTROL_FAILED;
    return true;
  }

  // Call the comon code to process the request
  flexible_manipulation_msgs::ExecuteKnownTrajectoryResult action_res;
  executePath(req.trajectory, action_res);

  res.error_code.val = action_res.error_code.val;
  return true;
}

// Action server execute goal interface
void ExecuteKnownTrajectoryCapability::executePathCallback(
    const flexible_manipulation_msgs::ExecuteKnownTrajectoryGoalConstPtr& goal)
{
  flexible_manipulation_msgs::ExecuteKnownTrajectoryResult action_res;
  if (!context_->trajectory_execution_manager_)
  {
    const std::string kResponse = "Cannot execute trajectory since "
                                  "~allow_trajectory_execution was set to false";
    action_res.error_code.val = moveit_msgs::MoveItErrorCodes::CONTROL_FAILED;
    execute_action_server_->setAborted(action_res, kResponse);
    return;
  }

  // Call the comon code to process the request
  executePath(goal->trajectory, action_res);

  const std::string kResponse = getActionResultString(action_res.error_code, false, false);
  if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    execute_action_server_->setSucceeded(action_res, kResponse);
  }
  else if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED)
  {
    execute_action_server_->setPreempted(action_res, kResponse);
  }
  else
  {
    execute_action_server_->setAborted(action_res, kResponse);
  }

  setExecuteKnownTrajectoryState(move_group::IDLE);
}

// Action server preempt
void ExecuteKnownTrajectoryCapability::preemptExecuteKnownTrajectoryCallback()
{
  context_->trajectory_execution_manager_->stopExecution(true);
}

// Action server feedback
void ExecuteKnownTrajectoryCapability::setExecuteKnownTrajectoryState(move_group::MoveGroupState state)
{
  flexible_manipulation_msgs::ExecuteKnownTrajectoryFeedback execute_feedback;
  execute_feedback.state = stateToStr(state);
  execute_action_server_->publishFeedback(execute_feedback);
}

// Common execution code
void ExecuteKnownTrajectoryCapability::executePath(const moveit_msgs::RobotTrajectory& trajectory,
                                                   flexible_manipulation_msgs::ExecuteKnownTrajectoryResult& action_res)
{
  ROS_INFO_NAMED(capability_name_, "Execution request received for ExecuteKnownTrajectory action.");

  context_->trajectory_execution_manager_->clear();
  if (context_->trajectory_execution_manager_->push(trajectory))
  {
    setExecuteKnownTrajectoryState(move_group::MONITOR);
    context_->trajectory_execution_manager_->execute();
    moveit_controller_manager::ExecutionStatus status = context_->trajectory_execution_manager_->waitForExecution();
    if (status == moveit_controller_manager::ExecutionStatus::SUCCEEDED)
    {
      action_res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    }
    else if (status == moveit_controller_manager::ExecutionStatus::PREEMPTED)
    {
      action_res.error_code.val = moveit_msgs::MoveItErrorCodes::PREEMPTED;
    }
    else if (status == moveit_controller_manager::ExecutionStatus::TIMED_OUT)
    {
      action_res.error_code.val = moveit_msgs::MoveItErrorCodes::TIMED_OUT;
    }
    else
    {
      action_res.error_code.val = moveit_msgs::MoveItErrorCodes::CONTROL_FAILED;
    }
    ROS_INFO_STREAM_NAMED(capability_name_, "Execution completed: " << status.asString());
  }
  else
  {
    action_res.error_code.val = moveit_msgs::MoveItErrorCodes::CONTROL_FAILED;
  }
}

}  // namespace flexible_manipulation

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(flexible_manipulation::ExecuteKnownTrajectoryCapability, move_group::MoveGroupCapability)
