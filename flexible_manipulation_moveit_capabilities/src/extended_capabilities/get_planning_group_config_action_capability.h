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

#ifndef VIGIR_ROBOT_GET_GROUP_CONFIG_ACTION_CAPABILITY_
#define VIGIR_ROBOT_GET_GROUP_CONFIG_ACTION_CAPABILITY_

#include <actionlib/server/simple_action_server.h>
#include <flexible_manipulation_msgs/GetPlanningGroupConfigAction.h>
#include <moveit/move_group/move_group_capability.h>

namespace flexible_manipulation
{
class GetPlanningGroupConfigAction : public move_group::MoveGroupCapability
{
public:
  GetPlanningGroupConfigAction();

  void initialize() override;

private:
  void executeCallback(const flexible_manipulation_msgs::GetPlanningGroupConfigGoalConstPtr& goal);

  boost::scoped_ptr<actionlib::SimpleActionServer<flexible_manipulation_msgs::GetPlanningGroupConfigAction>>
      gpgc_action_server_;
  flexible_manipulation_msgs::GetPlanningGroupConfigResult result_;
};
}

#endif
