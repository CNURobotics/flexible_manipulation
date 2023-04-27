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

#ifndef FLEXIBLE_MANIPULATION_MOVE_GROUP_EXTENDED_CAPABILITY_
#define FLEXIBLE_MANIPULATION_MOVE_GROUP_EXTENDED_CAPABILITY_

#include <actionlib/server/simple_action_server.h>
#include <flexible_manipulation_msgs/MoveGroupExtendedAction.h>
#include <moveit/move_group/move_group_capability.h>

#include <flexible_manipulation_moveit_capabilities/helper/continuous_plan_execution.h>
#include <moveit_msgs/GetCartesianPath.h>

#include "tf2_ros/transform_listener.h"

namespace flexible_manipulation
{
class MoveGroupExtendedAction : public move_group::MoveGroupCapability
{
public:
  MoveGroupExtendedAction();

  void initialize() override;

private:
  bool checkGroupStateSelfCollisionFree(robot_state::RobotState* robot_state,
                                        const robot_state::JointModelGroup* joint_group,
                                        const double* joint_group_variable_values);

  void setupEndEffectorData();

  void setCollisionOptions(bool all_env_collision_allow, bool end_effector_collision_allow);

  void executeMoveCallback(const flexible_manipulation_msgs::MoveGroupExtendedGoalConstPtr& goal);
  void executeMoveCallbackPlanAndExecute(const flexible_manipulation_msgs::MoveGroupExtendedGoalConstPtr& goal,
                                         flexible_manipulation_msgs::MoveGroupExtendedResult& action_res);
  void executeMoveCallbackPlanOnly(const flexible_manipulation_msgs::MoveGroupExtendedGoalConstPtr& goal,
                                   flexible_manipulation_msgs::MoveGroupExtendedResult& action_res);

  void executeCartesianMoveCallbackPlanAndExecute(const flexible_manipulation_msgs::MoveGroupExtendedGoalConstPtr& goal,
                                                  flexible_manipulation_msgs::MoveGroupExtendedResult& action_res);
  void startMoveExecutionCallback();
  void startMoveLookCallback();
  void preemptMoveCallback();
  void setMoveState(move_group::MoveGroupState state);
  bool planUsingPlanningPipeline(const planning_interface::MotionPlanRequest& req,
                                 plan_execution::ExecutableMotionPlan& plan);

  // Mostly copy of MoveGroup CartesianPath service with modifications
  bool computeCartesianPath(moveit_msgs::GetCartesianPath::Request& req, moveit_msgs::GetCartesianPath::Response& res,
                            double max_velocity_scaling_factor);

  planning_scene::PlanningSceneConstPtr
  getCollisionSettingsPlanningSceneDiff(const flexible_manipulation_msgs::MoveGroupExtendedGoalConstPtr& goal,
                                        planning_scene_monitor::LockedPlanningSceneRO& lscene) const;

  boost::scoped_ptr<actionlib::SimpleActionServer<flexible_manipulation_msgs::MoveGroupExtendedAction>>
      move_action_server_;
  flexible_manipulation_msgs::MoveGroupExtendedFeedback move_feedback_;

  move_group::MoveGroupState move_state_;

  plan_execution::ContinuousPlanExecutionPtr continuous_plan_execution_;

  boost::shared_ptr<trajectory_utils::TrajectoryVisualization> planned_traj_vis_;
  boost::shared_ptr<trajectory_utils::TrajectoryVisualization> executed_traj_vis_;

  ros::Publisher trajectory_result_display_pub_;
  ros::Publisher circular_target_path_pub_;

  // tf2_ros::TransformListener transform_listener_;

  boost::shared_ptr<trajectory_processing::IterativeParabolicTimeParameterization> time_param_;

  std::vector<std::string> end_effector_links_vector_;

  std::string end_effector_base_link_;
};
}

#endif
