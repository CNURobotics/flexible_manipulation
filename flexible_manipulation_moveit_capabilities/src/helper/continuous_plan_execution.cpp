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

#include <flexible_manipulation_moveit_capabilities/helper/continuous_plan_execution.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>

#include <eigen_conversions/eigen_msg.h>

namespace plan_execution
{
ContinuousPlanExecution::ContinuousPlanExecution(const move_group::MoveGroupContextPtr context) : context_(context)
{
  ros::NodeHandle pnh("~/continuous_replanning");

  traj_vis_.reset(new trajectory_utils::TrajectoryVisualization(pnh));
  debug_pose_pub_ = pnh.advertise<geometry_msgs::PoseStamped>("debug_pose", 1, false);
}

void ContinuousPlanExecution::initialize()
{
  // Default
}

void ContinuousPlanExecution::startExecution()
{
  if (continuous_replanning_thread_.get() == nullptr)
  {
    continuous_replanning_thread_.reset(
        new boost::thread(boost::bind(&ContinuousPlanExecution::continuousReplanningThread, this)));
  }
}

void ContinuousPlanExecution::stopExecution()
{
  ROS_INFO("Aborting continuous plan execution");

  stop_continuous_replanning_ = true;

  if (continuous_replanning_thread_.get() != nullptr)
  {
    continuous_replanning_thread_->join();
    continuous_replanning_thread_.reset();
  }
}

void ContinuousPlanExecution::continuousReplanningThread()
{
  stop_continuous_replanning_ = false;

  const robot_model::RobotModelConstPtr& robot_model = context_->planning_pipeline_->getRobotModel();
  const planning_scene::PlanningScenePtr& planning_scene = context_->planning_scene_monitor_->getPlanningScene();

  std::string group_name = "arm";

  planning_interface::MotionPlanRequest motion_plan_request;

  motion_plan_request.allowed_planning_time = 1.0;
  motion_plan_request.group_name = group_name;
  motion_plan_request.goal_constraints.resize(1);
  motion_plan_request.start_state.is_diff = 1u;
  motion_plan_request.max_velocity_scaling_factor = 0.3;

  boost::shared_ptr<planning_interface::MotionPlanResponse> mp_res;
  mp_res.reset(new planning_interface::MotionPlanResponse());

  boost::shared_ptr<planning_interface::MotionPlanResponse> mp_res_prior;
  ros::Time start_exec_time_prior;

  size_t count = 40;
  ros::WallTime start = ros::WallTime::now();

  context_->planning_scene_monitor_->updateFrameTransforms();

  robot_state::RobotState tmp = planning_scene->getCurrentState();

  const robot_state::JointModelGroup* jmg = tmp.getJointModelGroup(group_name);

  if (!jmg->getSolverInstance())
  {
    ROS_ERROR("No IK solver specified for group %s, cannot run continuous "
              "replanning!",
              group_name.c_str());
    return;
  }

  tmp.setToRandomPositions(jmg);

  const Eigen::Isometry3d& target_pose = tmp.getGlobalLinkTransform(jmg->getSolverInstance()->getTipFrame());

  if (debug_pose_pub_.getNumSubscribers() > 0)
  {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = robot_model->getModelFrame();
    pose_stamped.header.stamp = ros::Time::now();
    tf::poseEigenToMsg(target_pose, pose_stamped.pose);
    debug_pose_pub_.publish(pose_stamped);
  }

  for (size_t i = 0; i < count; ++i)
  {
    context_->planning_scene_monitor_->updateFrameTransforms();

    if (stop_continuous_replanning_)
    {
      context_->trajectory_execution_manager_->stopExecution();
      return;
    }

    robot_state::RobotState current_state = planning_scene->getCurrentState();

    const robot_state::JointModelGroup* current_group = current_state.getJointModelGroup(group_name);

    bool ik_solved = current_state.setFromIK(current_group, target_pose);

    if (!ik_solved)
    {
      ROS_ERROR("IK failed!");
    }

    motion_plan_request.goal_constraints[0] =
        kinematic_constraints::constructGoalConstraints(current_state, current_group);

    bool solved = false;

    if (ik_solved)
    {
      planning_scene_monitor::LockedPlanningSceneRO ps(context_->planning_scene_monitor_);
      solved = context_->planning_pipeline_->generatePlan(ps, motion_plan_request, *mp_res);
    }

    if (stop_continuous_replanning_)
    {
      context_->trajectory_execution_manager_->stopExecution();
      return;
    }

    ros::Time start_stamp = ros::Time::now();  // + ros::Duration(0.1);
    ros::Time merge_stamp = start_stamp + ros::Duration(0.4);

    if (solved)
    {
      if (mp_res_prior.get() != nullptr)
      {
        ROS_INFO("Traj points before: %d", static_cast<int>(mp_res->trajectory_->getWayPointCount()));

        robot_trajectory::RobotTrajectory merged_traj(mp_res->trajectory_->getRobotModel(),
                                                      mp_res->trajectory_->getGroupName());

        traj_merger_.mergeTrajectories(*mp_res_prior->trajectory_, *mp_res->trajectory_, start_exec_time_prior,
                                       start_stamp, merge_stamp, motion_plan_request.max_velocity_scaling_factor,
                                       merged_traj);

        *mp_res->trajectory_ = merged_traj;

        ROS_INFO("Traj points after: %d", static_cast<int>(mp_res->trajectory_->getWayPointCount()));
      }

      moveit_msgs::RobotTrajectory robot_traj;
      mp_res->trajectory_->getRobotTrajectoryMsg(robot_traj);
      robot_traj.joint_trajectory.header.stamp = start_stamp;

      //context_->trajectory_execution_manager_->pushAndExecute(robot_traj); //deprecated in Noetic https://github.com/ros-planning/moveit/issues/3252
      ROS_ERROR("pushAndExecute - deprecated in Noetic https://github.com/ros-planning/moveit/issues/3252");
      throw "pushAndExecute - deprecated in Noetic https://github.com/ros-planning/moveit/issues/3252";
      mp_res_prior = mp_res;

      traj_vis_->publishTrajectoryEndeffectorVis(*mp_res->trajectory_);

      start_exec_time_prior = start_stamp;

      ros::Time::sleepUntil(merge_stamp);
    }
    else
    {
      ROS_WARN("Cannot plan to given goal!");
      return;
    }
  }
  ROS_INFO("Elapsed time %f seconds, %f seconds per planning attempt", (ros::WallTime::now() - start).toSec(),
           (ros::WallTime::now() - start).toSec() / static_cast<double>(count));
}
}  // namespace plan_execution
