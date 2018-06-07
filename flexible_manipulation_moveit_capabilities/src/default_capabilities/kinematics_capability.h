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

#ifndef FLEXIBLE_MANIPULATION_KINEMATICS_SERVICE_CAPABILITY_
#define FLEXIBLE_MANIPULATION_KINEMATICS_SERVICE_CAPABILITY_

#include <actionlib/server/simple_action_server.h>
#include <flexible_manipulation_msgs/GetPositionFKAction.h>
#include <flexible_manipulation_msgs/GetPositionIKAction.h>
#include <moveit/move_group/move_group_capability.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>

namespace flexible_manipulation
{
class KinematicsCapability : public move_group::MoveGroupCapability
{
public:
  KinematicsCapability();

  virtual void initialize();

private:
  // Action interface
  void executeGetPositionIKCallback(const flexible_manipulation_msgs::GetPositionIKGoalConstPtr& goal);
  std::unique_ptr<actionlib::SimpleActionServer<flexible_manipulation_msgs::GetPositionIKAction>> get_ik_action_server_;

  void executeGetPositionFKCallback(const flexible_manipulation_msgs::GetPositionFKGoalConstPtr& goal);
  std::unique_ptr<actionlib::SimpleActionServer<flexible_manipulation_msgs::GetPositionFKAction>> get_fk_action_server_;

  // Service interface
  bool computeIKService(moveit_msgs::GetPositionIK::Request& req, moveit_msgs::GetPositionIK::Response& res);
  bool computeFKService(moveit_msgs::GetPositionFK::Request& req, moveit_msgs::GetPositionFK::Response& res);

  void computeIK(
      moveit_msgs::PositionIKRequest& req, moveit_msgs::RobotState& solution, moveit_msgs::MoveItErrorCodes& error_code,
      robot_state::RobotState& rs,
      const robot_state::GroupStateValidityCallbackFn& constraint = robot_state::GroupStateValidityCallbackFn()) const;

  ros::ServiceServer fk_service_;
  ros::ServiceServer ik_service_;
};
}

#endif
