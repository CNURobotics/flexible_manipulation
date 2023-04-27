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

#ifndef FLEXIBLE_MANIPULATION_CONSTRAINED_MOTION_UTILS_H__
#define FLEXIBLE_MANIPULATION_CONSTRAINED_MOTION_UTILS_H__

#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>

#include <eigen_conversions/eigen_msg.h>

namespace constrained_motion_utils
{
/**
 * Generates vector of poses that lie on an arc starting at start_point
 * and rotating around pose given by rotation_center
 * @param rotation_center The pose around which to rotate
 * @param start_point The pose that gets rotated around rotation_center
 * @param poses Vector of poses (output)
 * @param angular_resolution always positive angular resolution in rad
 * @param arc_length signed arc length
 */
static void getCircularArcPoses(const Eigen::Isometry3d& rotation_center, const Eigen::Isometry3d& start_point,
                                std::vector<geometry_msgs::Pose>& poses, double angular_resolution, double arc_length,
                                bool keep_orientation = false, double pitch = 0.0)
{
  double sizef = fabs(arc_length / angular_resolution);

  size_t size = static_cast<size_t>(sizef);
  double direction = arc_length > 0.0 ? 1.0 : -1.0;

  double pitch_increment = pitch / sizef;

  poses.resize(size);

  for (size_t i = 1; i <= size; ++i)
  {
    Eigen::Isometry3d rotation_increment(
        Eigen::AngleAxisd(direction * angular_resolution * static_cast<double>(i), Eigen::Vector3d::UnitX()));

    Eigen::Translation3d translation_increment(pitch_increment * static_cast<double>(i) * Eigen::Vector3d::UnitX());

    Eigen::Isometry3d pose(rotation_center * translation_increment * rotation_increment * rotation_center.inverse() *
                         start_point);

    if (keep_orientation)
    {
      pose.linear().setIdentity(); // Ignore rotation part, and retain translation
      pose *= start_point.rotation();
    }

    tf::poseEigenToMsg(pose, poses[i - 1]);
  }
}
}

#endif
