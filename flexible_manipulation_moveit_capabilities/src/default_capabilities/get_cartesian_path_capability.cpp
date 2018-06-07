/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018
 *  Capable Humanitarian Robotics and Intelligent Systems Lab (CHRISLab)
 *  Christopher Newport University
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

/* Author: Ioan Sucan and David Conner */

#include "get_cartesian_path_capability.h"
#include <eigen_conversions/eigen_msg.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/DisplayTrajectory.h>

flexible_manipulation::GetCartesianPathCapability::GetCartesianPathCapability()
  : move_group::MoveGroupCapability("GetCartesianPathCapability"), display_computed_paths_(true)
{
}

void flexible_manipulation::GetCartesianPathCapability::initialize()
{
  display_path_ = node_handle_.advertise<moveit_msgs::DisplayTrajectory>(
      planning_pipeline::PlanningPipeline::DISPLAY_PATH_TOPIC, 10, true);
  cartesian_path_service_ =
      root_node_handle_.advertiseService(move_group::CARTESIAN_PATH_SERVICE_NAME,
                                         &flexible_manipulation::GetCartesianPathCapability::computeService, this);
  // start the move action server
  action_server_.reset(new actionlib::SimpleActionServer<flexible_manipulation_msgs::GetCartesianPathAction>(
      root_node_handle_, move_group::CARTESIAN_PATH_SERVICE_NAME,
      boost::bind(&flexible_manipulation::GetCartesianPathCapability::executeCallback, this, _1), false));
  action_server_->start();
}

// Action interface makes use of the service interface
void flexible_manipulation::GetCartesianPathCapability::executeCallback(
    const flexible_manipulation_msgs::GetCartesianPathGoalConstPtr& goal)
{
  moveit_msgs::GetCartesianPath::Request req;
  moveit_msgs::GetCartesianPath::Response res;

  req.header = goal->header;
  req.start_state = goal->start_state;
  req.group_name = goal->group_name;
  req.link_name = goal->link_name;
  req.waypoints = goal->waypoints;
  req.max_step = goal->max_step;
  req.jump_threshold = goal->jump_threshold;
  req.avoid_collisions = goal->avoid_collisions;
  req.path_constraints = goal->path_constraints;

  bool ret = computeService(req, res);

  flexible_manipulation_msgs::GetCartesianPathResult action_res;
  action_res.start_state = res.start_state;
  action_res.solution = res.solution;
  action_res.fraction = res.fraction;
  action_res.error_code = res.error_code;

  if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    action_server_->setSucceeded(action_res, "");
  }
  else
  {
    action_server_->setAborted(action_res, "Failed to GetCartesianPath");
  }
}

namespace
{
bool isStateValid(const planning_scene::PlanningScene* planning_scene,
                  const kinematic_constraints::KinematicConstraintSet* constraint_set, robot_state::RobotState* state,
                  const robot_state::JointModelGroup* group, const double* ik_solution)
{
  state->setJointGroupPositions(group, ik_solution);
  state->update();
  return ((planning_scene == nullptr) || !planning_scene->isStateColliding(*state, group->getName())) &&
         ((constraint_set == nullptr) || constraint_set->decide(*state).satisfied);
}
}  // namespace

bool flexible_manipulation::GetCartesianPathCapability::computeService(moveit_msgs::GetCartesianPath::Request& req,
                                                                       moveit_msgs::GetCartesianPath::Response& res)
{
  ROS_INFO("Received request to compute Cartesian path");
  context_->planning_scene_monitor_->updateFrameTransforms();

  robot_state::RobotState start_state =
      planning_scene_monitor::LockedPlanningSceneRO(context_->planning_scene_monitor_)->getCurrentState();
  robot_state::robotStateMsgToRobotState(req.start_state, start_state);
  if (const robot_model::JointModelGroup* jmg = start_state.getJointModelGroup(req.group_name))
  {
    std::string link_name = req.link_name;
    if (link_name.empty() && !jmg->getLinkModelNames().empty())
    {
      link_name = jmg->getLinkModelNames().back();
    }

    bool ok = true;
    EigenSTL::vector_Affine3d waypoints(req.waypoints.size());
    const std::string& default_frame = context_->planning_scene_monitor_->getRobotModel()->getModelFrame();
    bool no_transform = req.header.frame_id.empty() ||
                        robot_state::Transforms::sameFrame(req.header.frame_id, default_frame) ||
                        robot_state::Transforms::sameFrame(req.header.frame_id, link_name);

    for (std::size_t i = 0; i < req.waypoints.size(); ++i)
    {
      if (no_transform)
      {
        tf::poseMsgToEigen(req.waypoints[i], waypoints[i]);
      }
      else
      {
        geometry_msgs::PoseStamped p;
        p.header = req.header;
        p.pose = req.waypoints[i];
        if (performTransform(p, default_frame))
        {
          tf::poseMsgToEigen(p.pose, waypoints[i]);
        }
        else
        {
          ROS_ERROR("Error encountered transforming waypoints to frame '%s'", default_frame.c_str());
          ok = false;
          break;
        }
      }
    }

    if (ok)
    {
      if (req.max_step < std::numeric_limits<double>::epsilon())
      {
        ROS_ERROR("Maximum step to take between consecutive configrations "
                  "along Cartesian path was not specified (this "
                  "value needs to be > 0)");
        res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
      }
      else
      {
        if (!waypoints.empty())
        {
          robot_state::GroupStateValidityCallbackFn constraint_fn;
          std::unique_ptr<planning_scene_monitor::LockedPlanningSceneRO> ls;
          std::unique_ptr<kinematic_constraints::KinematicConstraintSet> kset;
          if ((req.avoid_collisions != 0u) || !kinematic_constraints::isEmpty(req.path_constraints))
          {
            ls.reset(new planning_scene_monitor::LockedPlanningSceneRO(context_->planning_scene_monitor_));
            kset.reset(new kinematic_constraints::KinematicConstraintSet((*ls)->getRobotModel()));
            kset->add(req.path_constraints, (*ls)->getTransforms());
            constraint_fn =
                boost::bind(&isStateValid, req.avoid_collisions != 0u ?
                                               static_cast<const planning_scene::PlanningSceneConstPtr&>(*ls).get() :
                                               nullptr,
                            kset->empty() ? nullptr : kset.get(), _1, _2, _3);
          }
          bool global_frame = !robot_state::Transforms::sameFrame(link_name, req.header.frame_id);
          ROS_INFO("Attempting to follow %u waypoints for link '%s' using a "
                   "step of %lf m and jump threshold %lf (in "
                   "%s reference frame)",
                   (unsigned int)waypoints.size(), link_name.c_str(), req.max_step, req.jump_threshold,
                   global_frame ? "global" : "link");
          std::vector<robot_state::RobotStatePtr> traj;
          res.fraction =
              start_state.computeCartesianPath(jmg, traj, start_state.getLinkModel(link_name), waypoints, global_frame,
                                               req.max_step, req.jump_threshold, constraint_fn);
          robot_state::robotStateToRobotStateMsg(start_state, res.start_state);

          robot_trajectory::RobotTrajectory rt(context_->planning_scene_monitor_->getRobotModel(), req.group_name);
          for (auto& i : traj)
          {
            rt.addSuffixWayPoint(i, 0.0);
          }

          // time trajectory
          // \todo optionally compute timing to move the eef with constant speed
          trajectory_processing::IterativeParabolicTimeParameterization time_param;
          time_param.computeTimeStamps(rt, 1.0);

          rt.getRobotTrajectoryMsg(res.solution);
          ROS_INFO("Computed Cartesian path with %u points (followed %lf%% of "
                   "requested trajectory)",
                   (unsigned int)traj.size(), res.fraction * 100.0);
          if (display_computed_paths_ && rt.getWayPointCount() > 0)
          {
            moveit_msgs::DisplayTrajectory disp;
            disp.model_id = context_->planning_scene_monitor_->getRobotModel()->getName();
            disp.trajectory.resize(1, res.solution);
            robot_state::robotStateToRobotStateMsg(rt.getFirstWayPoint(), disp.trajectory_start);
            display_path_.publish(disp);
          }
        }
        res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
      }
    }
    else
    {
      res.error_code.val = moveit_msgs::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE;
    }
  }
  else
  {
    res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
  }

  return true;
}

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(flexible_manipulation::GetCartesianPathCapability, move_group::MoveGroupCapability)
