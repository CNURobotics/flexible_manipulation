/*********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018
 * Capable Humanitarian Robotics and Intelligent Systems Lab (CHRISLab)
 * Christopher Newport University
 *
 *  Copyright (c) 2014, SRI, Inc.
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

/* Author: David Hershberger, David Conner, Julie Gates, and Jenny Gu*/

#include "clear_octomap_capability.h"
#include <moveit/move_group/capability_names.h>

flexible_manipulation::ClearOctomapCapability::ClearOctomapCapability()
  : move_group::MoveGroupCapability("ClearOctomapCapability")
{
}

void flexible_manipulation::ClearOctomapCapability::initialize()
{
  service_ = root_node_handle_.advertiseService(move_group::CLEAR_OCTOMAP_SERVICE_NAME,
                                                &flexible_manipulation::ClearOctomapCapability::clearOctomap, this);

  action_server_.reset(new actionlib::SimpleActionServer<flexible_manipulation_msgs::ClearOctomapAction>(
      root_node_handle_, move_group::CLEAR_OCTOMAP_SERVICE_NAME,
      boost::bind(&flexible_manipulation::ClearOctomapCapability::executeCallback, this, _1), false));
  action_server_->start();
}

void flexible_manipulation::ClearOctomapCapability::executeCallback(
    const flexible_manipulation_msgs::ClearOctomapGoalConstPtr& /*goal*/)
{
  flexible_manipulation_msgs::ClearOctomapResult res;
  if (!context_->planning_scene_monitor_)
  {
    ROS_ERROR("Cannot clear octomap since planning_scene_monitor_ does not exist.");
    action_server_->setAborted(res, "Cannot clear octomap - No planning monitor");
  }

  ROS_INFO("Clearing octomap...");
  context_->planning_scene_monitor_->clearOctomap();
  ROS_INFO("Octomap cleared.");
  action_server_->setSucceeded(res, "Cleared octomap");
}

bool flexible_manipulation::ClearOctomapCapability::clearOctomap(std_srvs::Empty::Request& /*req*/,
                                                                 std_srvs::Empty::Response& /*res*/)
{
  if (!context_->planning_scene_monitor_)
  {
    ROS_ERROR("Cannot clear octomap since planning_scene_monitor_ does not exist.");
    return true;
  }

  ROS_INFO("Clearing octomap...");
  context_->planning_scene_monitor_->clearOctomap();
  ROS_INFO("Octomap cleared.");
  return true;
}

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(flexible_manipulation::ClearOctomapCapability, move_group::MoveGroupCapability)
