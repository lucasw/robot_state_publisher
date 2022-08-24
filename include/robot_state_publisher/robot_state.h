/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
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

/* Author: Wim Meeussen */

#ifndef ROBOT_STATE_PUBLISHER_ROBOT_STATE_H
#define ROBOT_STATE_PUBLISHER_ROBOT_STATE_H

#include <map>
#include <string>

#include <kdl/frames.hpp>
#include <kdl/segment.hpp>
#include <kdl/tree.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <urdf/model.h>

#include "robot_state_publisher/robot_state_publisher.h"

namespace robot_state_publisher {

typedef std::map<std::string, urdf::JointMimicSharedPtr> MimicMap;

class RobotState
{
public:
  /** Default constructor.
   */
  RobotState();

  /// Destructor
  ~RobotState(){}

  virtual void setJointState(const sensor_msgs::JointState& joint_state);
  sensor_msgs::JointState getJointStates();

  /** Get transforms given recent joint angles
   * \param joint_positions A map of joint names and joint positions.
   * \param time The time at which the joint positions were recorded
   */
  tf2_msgs::TFMessage getTransforms(const ros::Time& time, const std::string& tf_prefix) const;
  tf2_msgs::TFMessage getFixedTransforms(const ros::Time& time, const std::string& tf_prefix) const;

protected:
  virtual void addChildren(const KDL::SegmentMap::const_iterator segment);

  MimicMap mimic_;
  std::map<std::string, SegmentPair> segments_, segments_fixed_;
  std::map<std::string, double> joint_positions_;
  urdf::Model model_;
};

}  // namespace robot_state_publisher

#endif  // ROBOT_STATE_PUBLISHER_ROBOT_STATE_H
