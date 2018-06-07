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

/* Original Author: Stefan Kohlbrecher */

#ifndef FLEXIBLE_MANIPULATION_PLANNING_SCENE_UTILS_H__
#define FLEXIBLE_MANIPULATION_PLANNING_SCENE_UTILS_H__

#include <Eigen/Geometry>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

namespace planning_scene_utils
{
static bool get_eef_link(const std::string group_name, std::string& eef_link)
{
  std::string first_char = group_name.substr(0, 1);

  if (first_char == "r")
  {
    eef_link = "r_hand";
  }
  else if (first_char == "l")
  {
    eef_link = "l_hand";
  }
  else
  {
    ROS_ERROR("Group name %s does not start with l or r. Cannot infer "
              "endeffector to use, aborting",
              group_name.c_str());
    return false;
  }
  return true;
}

static bool getEndeffectorTransformOfLink(const std::string& eef_link_name,
                                          const planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
                                          Eigen::Affine3d& transform)
{
  planning_scene_monitor::LockedPlanningSceneRO lscene(planning_scene_monitor);
  const robot_state::RobotState& curr_state = lscene.getPlanningSceneMonitor()->getPlanningScene()->getCurrentState();
  transform = curr_state.getGlobalLinkTransform(eef_link_name);

  return true;
}

static bool getEndeffectorTransform(const std::string& group_name,
                                    const planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
                                    Eigen::Affine3d& transform)
{
  std::string start_pose_link;
  if (!get_eef_link(group_name, start_pose_link))
    return false;

  return getEndeffectorTransformOfLink(start_pose_link, planning_scene_monitor, transform);
}
}

#endif
