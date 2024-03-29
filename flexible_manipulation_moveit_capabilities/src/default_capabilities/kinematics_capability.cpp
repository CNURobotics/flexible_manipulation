/*********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018-2023
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

#include "kinematics_capability.h"
#include <tf2_eigen/tf2_eigen.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/utils/message_checks.h>

namespace flexible_manipulation
{
  KinematicsCapability::KinematicsCapability()
    : move_group::MoveGroupCapability("KinematicsCapability")
{
}

void KinematicsCapability::initialize()
{
  fk_service_ = root_node_handle_.advertiseService(
      move_group::FK_SERVICE_NAME, &flexible_manipulation::KinematicsCapability::computeFKService, this);
  ik_service_ = root_node_handle_.advertiseService(
      move_group::IK_SERVICE_NAME, &flexible_manipulation::KinematicsCapability::computeIKService, this);

  get_ik_action_server_.reset(new actionlib::SimpleActionServer<flexible_manipulation_msgs::GetPositionIKAction>(
      root_node_handle_, move_group::IK_SERVICE_NAME,
      boost::bind(&flexible_manipulation::KinematicsCapability::executeGetPositionIKCallback, this, _1), false));
  get_ik_action_server_->start();

  get_fk_action_server_.reset(new actionlib::SimpleActionServer<flexible_manipulation_msgs::GetPositionFKAction>(
      root_node_handle_, move_group::FK_SERVICE_NAME,
      boost::bind(&flexible_manipulation::KinematicsCapability::executeGetPositionFKCallback, this, _1), false));
  get_fk_action_server_->start();
}

namespace
{
bool isIKSolutionValid(const planning_scene::PlanningScene* planning_scene,
                       const kinematic_constraints::KinematicConstraintSet* constraint_set,
                       robot_state::RobotState* state, const robot_model::JointModelGroup* jmg,
                       const double* ik_solution)
{
  state->setJointGroupPositions(jmg, ik_solution);
  state->update();
  return ((planning_scene == nullptr) || !planning_scene->isStateColliding(*state, jmg->getName())) &&
         ((constraint_set == nullptr) || constraint_set->decide(*state).satisfied);
}
}  // namespace

void flexible_manipulation::KinematicsCapability::executeGetPositionIKCallback(
    const flexible_manipulation_msgs::GetPositionIKGoalConstPtr& /*goal*/)
{
  flexible_manipulation_msgs::GetPositionIKResult res;
  if (true)  //! context_->planning_scene_monitor_)
  {
    ROS_ERROR("Cannot get position IK - not implemented for ActionInterface "
              "just yet!");
    get_ik_action_server_->setAborted(res, "Cannot get position IK - not "
                                           "implemented for ActionInterface "
                                           "just yet!");
  }

  get_ik_action_server_->setSucceeded(res, "");
}

void flexible_manipulation::KinematicsCapability::executeGetPositionFKCallback(
    const flexible_manipulation_msgs::GetPositionFKGoalConstPtr& /*goal*/)
{
  flexible_manipulation_msgs::GetPositionFKResult res;
  if (true)  //! context_->planning_scene_monitor_)
  {
    ROS_ERROR("Cannot get position FK - not implemented for ActionInterface "
              "just yet!");
    get_fk_action_server_->setAborted(res, "Cannot get position FK - not "
                                           "implemented for ActionInterface "
                                           "just yet!");
  }

  get_fk_action_server_->setSucceeded(res, "");
}

void KinematicsCapability::computeIK(
    moveit_msgs::PositionIKRequest& req, moveit_msgs::RobotState& solution, moveit_msgs::MoveItErrorCodes& error_code,
    robot_state::RobotState& rs, const robot_state::GroupStateValidityCallbackFn& constraint) const
{
  const robot_state::JointModelGroup* jmg = rs.getJointModelGroup(req.group_name);
  if (jmg != nullptr)
  {
    robot_state::robotStateMsgToRobotState(req.robot_state, rs);
    const std::string& default_frame = context_->planning_scene_monitor_->getRobotModel()->getModelFrame();

    if (req.pose_stamped_vector.empty() || req.pose_stamped_vector.size() == 1)
    {
      geometry_msgs::PoseStamped req_pose =
          req.pose_stamped_vector.empty() ? req.pose_stamped : req.pose_stamped_vector[0];
      std::string ik_link = (!req.pose_stamped_vector.empty()) ?
                                (req.ik_link_names.empty() ? "" : req.ik_link_names[0]) :
                                req.ik_link_name;

      if (performTransform(req_pose, default_frame))
      {
        bool result_ik = false;
        if (ik_link.empty())
        {
          result_ik = rs.setFromIK(jmg, req_pose.pose, req.timeout.toSec(), constraint);
        }
        else
        {
          result_ik = rs.setFromIK(jmg, req_pose.pose, ik_link, req.timeout.toSec(), constraint);
        }

        if (result_ik)
        {
          robot_state::robotStateToRobotStateMsg(rs, solution, false);
          error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
        }
        else
        {
          error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
        }
      }
      else
      {
        error_code.val = moveit_msgs::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE;
      }
    }
    else
    {
      if (req.pose_stamped_vector.size() != req.ik_link_names.size())
      {
        error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_LINK_NAME;
      }
      else
      {
        bool ok = true;
        EigenSTL::vector_Isometry3d req_poses(req.pose_stamped_vector.size());
        for (std::size_t k = 0; k < req.pose_stamped_vector.size(); ++k)
        {
          geometry_msgs::PoseStamped msg = req.pose_stamped_vector[k];
          if (performTransform(msg, default_frame))
          {
            tf2::fromMsg(msg.pose, req_poses[k]);
          }
          else
          {
            error_code.val = moveit_msgs::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE;
            ok = false;
            break;
          }
        }
        if (ok)
        {
          if (rs.setFromIK(jmg, req_poses, req.ik_link_names, req.timeout.toSec(), constraint))
          {
            robot_state::robotStateToRobotStateMsg(rs, solution, false);
            error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
          }
          else
          {
            error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
          }
        }
      }
    }
  }
  else
  {
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
  }
}

bool KinematicsCapability::computeIKService(moveit_msgs::GetPositionIK::Request& req,
                                                                   moveit_msgs::GetPositionIK::Response& res)
{
  context_->planning_scene_monitor_->updateFrameTransforms();

  // check if the planning scene needs to be kept locked; if so, call
  // computeIK() in the scope of the lock
  if ((req.ik_request.avoid_collisions != 0u) || !moveit::core::isEmpty(req.ik_request.constraints))
  {
    planning_scene_monitor::LockedPlanningSceneRO ls(context_->planning_scene_monitor_);
    kinematic_constraints::KinematicConstraintSet kset(ls->getRobotModel());
    robot_state::RobotState rs = ls->getCurrentState();
    kset.add(req.ik_request.constraints, ls->getTransforms());
    computeIK(req.ik_request, res.solution, res.error_code, rs,
              boost::bind(&isIKSolutionValid, req.ik_request.avoid_collisions != 0u ?
                                                  static_cast<const planning_scene::PlanningSceneConstPtr&>(ls).get() :
                                                  nullptr,
                          kset.empty() ? nullptr : &kset, _1, _2, _3));
  }
  else
  {
    // compute unconstrained IK, no lock to planning scene maintained
    robot_state::RobotState rs =
        planning_scene_monitor::LockedPlanningSceneRO(context_->planning_scene_monitor_)->getCurrentState();
    computeIK(req.ik_request, res.solution, res.error_code, rs);
  }

  return true;
}

bool KinematicsCapability::computeFKService(moveit_msgs::GetPositionFK::Request& req,
                                                                   moveit_msgs::GetPositionFK::Response& res)
{
  if (req.fk_link_names.empty())
  {
    ROS_ERROR_NAMED(getName(), "No links specified for FK request");
    res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_LINK_NAME;
    return true;
  }

  context_->planning_scene_monitor_->updateFrameTransforms();

  const std::string& default_frame = context_->planning_scene_monitor_->getRobotModel()->getModelFrame();
  bool do_transform = !req.header.frame_id.empty() &&
                      !robot_state::Transforms::sameFrame(req.header.frame_id, default_frame) &&
                      context_->planning_scene_monitor_->getTFClient();
  bool tf_problem = false;

  robot_state::RobotState rs =
      planning_scene_monitor::LockedPlanningSceneRO(context_->planning_scene_monitor_)->getCurrentState();
  robot_state::robotStateMsgToRobotState(req.robot_state, rs);
  for (std::size_t i = 0; i < req.fk_link_names.size(); ++i)
  {
    if (rs.getRobotModel()->hasLinkModel(req.fk_link_names[i]))
    {
      res.pose_stamped.resize(res.pose_stamped.size() + 1);
      res.pose_stamped.back().pose = tf2::toMsg(rs.getGlobalLinkTransform(req.fk_link_names[i]));
      res.pose_stamped.back().header.frame_id = default_frame;
      res.pose_stamped.back().header.stamp = ros::Time::now();
      if (do_transform)
      {
        if (!performTransform(res.pose_stamped.back(), req.header.frame_id))
        {
          tf_problem = true;
        }
        res.fk_link_names.push_back(req.fk_link_names[i]);
      }
    }
  }
  if (tf_problem)
  {
    res.error_code.val = moveit_msgs::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE;
  }
  else if (res.fk_link_names.size() == req.fk_link_names.size())
  {
    res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  }
  else
  {
    res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_LINK_NAME;
  }
  return true;
}
}  // namespace flexible_manipulation

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(flexible_manipulation::KinematicsCapability, move_group::MoveGroupCapability)
