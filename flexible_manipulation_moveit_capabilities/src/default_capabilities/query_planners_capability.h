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
 *   * Neither the name of Christopher Newport University, CHRISLab, SRI,
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

/* Author: Ioan Sucan, Robert Haschke, David Conner, Julie Gates, and Jenny Gu
 */

#ifndef MOVEIT_MOVE_GROUP_QUERY_PLANNERS_SERVICE_CAPABILITY_
#define MOVEIT_MOVE_GROUP_QUERY_PLANNERS_SERVICE_CAPABILITY_

#include <actionlib/server/simple_action_server.h>
#include <flexible_manipulation_msgs/QueryPlannersAction.h>
#include <moveit/move_group/move_group_capability.h>
#include <moveit_msgs/GetPlannerParams.h>
#include <moveit_msgs/QueryPlannerInterfaces.h>
#include <moveit_msgs/SetPlannerParams.h>

namespace flexible_manipulation
{
class QueryPlannersCapability : public move_group::MoveGroupCapability
{
public:
  QueryPlannersCapability();

  virtual void initialize();

private:
  void executeCallback(const flexible_manipulation_msgs::QueryPlannersGoalConstPtr& goal);

  std::unique_ptr<actionlib::SimpleActionServer<flexible_manipulation_msgs::QueryPlannersAction>> action_server_;

  bool queryInterface(moveit_msgs::QueryPlannerInterfaces::Request& req,
                      moveit_msgs::QueryPlannerInterfaces::Response& res);

  bool getParams(moveit_msgs::GetPlannerParams::Request& req, moveit_msgs::GetPlannerParams::Response& res);
  bool setParams(moveit_msgs::SetPlannerParams::Request& req, moveit_msgs::SetPlannerParams::Response& res);

  ros::ServiceServer query_service_;
  ros::ServiceServer get_service_;
  ros::ServiceServer set_service_;
};
}

#endif
