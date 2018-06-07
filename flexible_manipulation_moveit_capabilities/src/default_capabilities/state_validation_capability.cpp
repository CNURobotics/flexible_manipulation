/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 * Copyright (c) 2018
 * Capable Humanitarian Robotics and Intelligent Systems Lab (CHRISLab)
 * Christopher Newport University
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

#include "state_validation_capability.h"
#include <eigen_conversions/eigen_msg.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/robot_state/conversions.h>

flexible_manipulation::StateValidationCapability::StateValidationCapability()
  : move_group::MoveGroupCapability("StateValidationCapability")
{
}

void flexible_manipulation::StateValidationCapability::initialize()
{
  validity_service_ = root_node_handle_.advertiseService(
      move_group::STATE_VALIDITY_SERVICE_NAME, &flexible_manipulation::StateValidationCapability::computeService, this);

  action_server_.reset(new actionlib::SimpleActionServer<flexible_manipulation_msgs::StateValidationAction>(
      root_node_handle_, move_group::STATE_VALIDITY_SERVICE_NAME,
      boost::bind(&flexible_manipulation::StateValidationCapability::executeCallback, this, _1), false));
  action_server_->start();
}

void flexible_manipulation::StateValidationCapability::executeCallback(
    const flexible_manipulation_msgs::StateValidationGoalConstPtr& goal)
{
  flexible_manipulation_msgs::StateValidationResult res;
  res.valid = true;

  if (!context_->planning_scene_monitor_)
  {
    res.valid = false;
    action_server_->setAborted(res, "No planning scene monitor");
  }

  planning_scene_monitor::LockedPlanningSceneRO ls(context_->planning_scene_monitor_);
  robot_state::RobotState rs = ls->getCurrentState();
  robot_state::robotStateMsgToRobotState(goal->robot_state, rs);

  // configure collision request
  collision_detection::CollisionRequest creq;
  creq.group_name = goal->group_name;
  creq.cost = true;
  creq.contacts = true;
  creq.max_contacts = ls->getWorld()->size();
  creq.max_cost_sources = creq.max_contacts + ls->getRobotModel()->getLinkModelsWithCollisionGeometry().size();
  creq.max_contacts *= creq.max_contacts;

  collision_detection::CollisionResult cres;
  // check collision
  ls->checkCollision(creq, cres, rs);

  // copy contacts if any
  if (cres.collision)
  {
    ros::Time time_now = ros::Time::now();
    res.contacts.reserve(cres.contact_count);
    res.valid = false;
    for (collision_detection::CollisionResult::ContactMap::const_iterator it = cres.contacts.begin();
         it != cres.contacts.end(); ++it)
    {
      for (const auto& k : it->second)
      {
        res.contacts.resize(res.contacts.size() + 1);
        collision_detection::contactToMsg(k, res.contacts.back());
        res.contacts.back().header.frame_id = ls->getPlanningFrame();
        res.contacts.back().header.stamp = time_now;
      }
    }
  }

  // copy cost sources
  res.cost_sources.reserve(cres.cost_sources.size());
  for (const auto& cost_source : cres.cost_sources)
  {
    res.cost_sources.resize(res.cost_sources.size() + 1);
    collision_detection::costSourceToMsg(cost_source, res.cost_sources.back());
  }

  // evaluate constraints
  if (!kinematic_constraints::isEmpty(goal->constraints))
  {
    kinematic_constraints::KinematicConstraintSet kset(ls->getRobotModel());
    kset.add(goal->constraints, ls->getTransforms());
    std::vector<kinematic_constraints::ConstraintEvaluationResult> kres;
    kinematic_constraints::ConstraintEvaluationResult total_result = kset.decide(rs, kres);
    if (!total_result.satisfied)
    {
      res.valid = false;
    }

    // copy constraint results
    res.constraint_result.resize(kres.size());
    for (std::size_t k = 0; k < kres.size(); ++k)
    {
      res.constraint_result[k].result = kres[k].satisfied;
      res.constraint_result[k].distance = kres[k].distance;
    }
  }

  action_server_->setSucceeded(res, "");
  return;
}

bool flexible_manipulation::StateValidationCapability::computeService(moveit_msgs::GetStateValidity::Request& req,
                                                                      moveit_msgs::GetStateValidity::Response& res)
{
  planning_scene_monitor::LockedPlanningSceneRO ls(context_->planning_scene_monitor_);
  robot_state::RobotState rs = ls->getCurrentState();
  robot_state::robotStateMsgToRobotState(req.robot_state, rs);

  res.valid = true;

  // configure collision request
  collision_detection::CollisionRequest creq;
  creq.group_name = req.group_name;
  creq.cost = true;
  creq.contacts = true;
  creq.max_contacts = ls->getWorld()->size();
  creq.max_cost_sources = creq.max_contacts + ls->getRobotModel()->getLinkModelsWithCollisionGeometry().size();
  creq.max_contacts *= creq.max_contacts;
  collision_detection::CollisionResult cres;

  // check collision
  ls->checkCollision(creq, cres, rs);

  // copy contacts if any
  if (cres.collision)
  {
    ros::Time time_now = ros::Time::now();
    res.contacts.reserve(cres.contact_count);
    res.valid = 0u;
    for (collision_detection::CollisionResult::ContactMap::const_iterator it = cres.contacts.begin();
         it != cres.contacts.end(); ++it)
    {
      for (const auto& k : it->second)
      {
        res.contacts.resize(res.contacts.size() + 1);
        collision_detection::contactToMsg(k, res.contacts.back());
        res.contacts.back().header.frame_id = ls->getPlanningFrame();
        res.contacts.back().header.stamp = time_now;
      }
    }
  }

  // copy cost sources
  res.cost_sources.reserve(cres.cost_sources.size());
  for (const auto& cost_source : cres.cost_sources)
  {
    res.cost_sources.resize(res.cost_sources.size() + 1);
    collision_detection::costSourceToMsg(cost_source, res.cost_sources.back());
  }

  // evaluate constraints
  if (!kinematic_constraints::isEmpty(req.constraints))
  {
    kinematic_constraints::KinematicConstraintSet kset(ls->getRobotModel());
    kset.add(req.constraints, ls->getTransforms());
    std::vector<kinematic_constraints::ConstraintEvaluationResult> kres;
    kinematic_constraints::ConstraintEvaluationResult total_result = kset.decide(rs, kres);
    if (!total_result.satisfied)
    {
      res.valid = false;
    }

    // copy constraint results
    res.constraint_result.resize(kres.size());
    for (std::size_t k = 0; k < kres.size(); ++k)
    {
      res.constraint_result[k].result = kres[k].satisfied;
      res.constraint_result[k].distance = kres[k].distance;
    }
  }

  return true;
}

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(flexible_manipulation::StateValidationCapability, move_group::MoveGroupCapability)
