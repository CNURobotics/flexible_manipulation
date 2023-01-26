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

/* Original Author: Stefan Kohlbrecher */

#include "octomap_access_capability.h"

#include <moveit/move_group/capability_names.h>

flexible_manipulation::OctomapAccessCapability::OctomapAccessCapability()
  : move_group::MoveGroupCapability("OctomapAccessCapability")
{
}

void flexible_manipulation::OctomapAccessCapability::initialize()
{
  octomap_full_pub_ = node_handle_.advertise<octomap_msgs::Octomap>("octomap", 1, false);

  vis_timer_ = node_handle_.createTimer(ros::Duration(1.0),
                                        &flexible_manipulation::OctomapAccessCapability::visTimerCallback, this, false);
}

void flexible_manipulation::OctomapAccessCapability::visTimerCallback(const ros::TimerEvent& /*event*/)
{
  if (octomap_full_pub_.getNumSubscribers() > 0)
  {
    moveit_msgs::PlanningScene tmp;
    moveit_msgs::PlanningSceneComponents comp;
    std::string octomap_frame_id;
    comp.components = moveit_msgs::PlanningSceneComponents::OCTOMAP;

    {
      planning_scene_monitor::LockedPlanningSceneRO ls(context_->planning_scene_monitor_);
      ls.getPlanningSceneMonitor()->getPlanningScene()->getPlanningSceneMsg(tmp, comp);
      octomap_frame_id = ls.getPlanningSceneMonitor()->getPlanningScene()->getPlanningFrame();
    }

    tmp.world.octomap.octomap.header.frame_id = octomap_frame_id;

    octomap_full_pub_.publish(tmp.world.octomap.octomap);
  }
}

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(flexible_manipulation::OctomapAccessCapability, move_group::MoveGroupCapability)
