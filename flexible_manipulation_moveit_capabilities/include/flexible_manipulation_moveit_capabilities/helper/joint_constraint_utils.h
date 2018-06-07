/*********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018
 * Capable Humanitarian Robotics and Intelligent Systems Lab (CHRISLab)
 * Christopher Newport University
 *
 *  This code is based on code from DARPA Robotics Challenge Team ViGIR
 *  Copyright (c) 2013, Stefan Kohlbrecher, TU Darmstadt ( Team ViGIR )
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

// Original author: Stefan Kohlbrecher

#ifndef FLEXIBLE_MANIPULATION_JOINT_CONSTRAINT_UTILS_H__
#define FLEXIBLE_MANIPULATION_JOINT_CONSTRAINT_UTILS_H__

#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>

#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model/robot_model.h>

#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_msgs/JointConstraint.h>

#include <eigen_conversions/eigen_msg.h>

#include <flexible_manipulation_msgs/constraints_conversion.h>

namespace joint_constraint_utils
{
static inline bool toMoveitConstraint(const flexible_manipulation_msgs::JointPositionConstraint& input,
                                      const robot_model::RobotModel& robot_model, moveit_msgs::JointConstraint& output)
{
  flexible_manipulation_msgs::toMoveitConstraint(input, output);

  output.joint_name = robot_model.getJointModel(input.joint_index)->getName();
  output.weight = 0.5;

  return true;
}

static inline bool toMoveitConstraint(const std::vector<flexible_manipulation_msgs::JointPositionConstraint>& input,
                                      const robot_model::RobotModel& robot_model,
                                      std::vector<moveit_msgs::JointConstraint>& output)
{
  output.clear();

  output.resize(input.size());

  for (size_t i = 0; i < input.size(); ++i)
  {
    if (!toMoveitConstraint(input[i], robot_model, output[i]))
    {
      output.clear();
      return false;
    }
  }

  return true;
}

static inline bool toExtendedConstraint(const moveit_msgs::JointConstraint& input,
                                        const robot_model::RobotModel& robot_model,
                                        flexible_manipulation_msgs::JointPositionConstraint& output)
{
  flexible_manipulation_msgs::toExtendedConstraint(input, output);

  output.joint_index = robot_model.getJointModel(input.joint_name)->getJointIndex();

  return true;
}

std::vector<std::string> getLockedJoints(const robot_model::JointModelGroup* group,
                                         const std::vector<moveit_msgs::JointConstraint> constraints)
{
  std::vector<std::string> lockedJoints;

  for (size_t i = 0; i < constraints.size(); ++i)
  {
    if (group->hasJointModel(constraints[i].joint_name) &&
        (constraints[i].tolerance_above <= std::numeric_limits<double>::epsilon()))
    {
      lockedJoints.push_back(constraints[i].joint_name);
    }
  }

  return lockedJoints;
}
}

#endif
