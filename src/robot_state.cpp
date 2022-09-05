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

#include <map>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/TransformStamped.h>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf2_kdl/tf2_kdl.h>

#include "robot_state_publisher/robot_state.h"

namespace robot_state_publisher {

RobotState::RobotState()
{
  // TODO(lucasw) want to be able to get urdf from any param, or via xml string

  // gets the location of the robot description on the parameter server
  if (!model_.initParam("robot_description")) {
    // TODO(lucasw) throw
    ROS_ERROR("could not load robot_description");
    return;
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model_, tree)) {
    // TODO(lucasw) throw
    ROS_ERROR("Failed to extract kdl tree from xml robot description");
    return;
  }

  for (const auto& joints : model_.joints_) {
    const auto& joint_name = joints.first;
    const auto& joint = joints.second;
    if (joint->mimic) {
      mimic_[joint_name] = joint->mimic;
    }
    if (joint->limits) {
      ROS_INFO_STREAM(joint_name << " limited: " << joint->limits->lower << " - " << joint->limits->upper);
    }
  }
  ROS_INFO_STREAM("mimic joints " << mimic_.size());

  // walk the tree recursively and add segments to segments_
  addChildren(tree.getRootSegment());

  ROS_INFO_STREAM("segments size " << segments_.size()
      << ", segments_fixed size " << segments_fixed_.size());
}

// add children to correct maps
void RobotState::addChildren(const KDL::SegmentMap::const_iterator segment)
{
  const std::string& root = GetTreeElementSegment(segment->second).getName();

  const std::vector<KDL::SegmentMap::const_iterator>& children = GetTreeElementChildren(segment->second);
  for (size_t i = 0; i < children.size(); ++i) {
    const KDL::Segment& child = GetTreeElementSegment(children[i]->second);
    SegmentPair s(GetTreeElementSegment(children[i]->second), root, child.getName());
    if (child.getJoint().getType() == KDL::Joint::None) {
      if (model_.getJoint(child.getJoint().getName()) &&
          model_.getJoint(child.getJoint().getName())->type == urdf::Joint::FLOATING) {
        ROS_WARN("Floating joint. Not adding segment from '%s' to '%s'."
                 "  This TF can not be published based on joint_states info",
                 root.c_str(), child.getName().c_str());
      }
      else {
        segments_fixed_[child.getJoint().getName()] = s;
        ROS_DEBUG("Adding fixed segment from '%s' to '%s'", root.c_str(), child.getName().c_str());
      }
    }
    else {
      segments_[child.getJoint().getName()] = s;
      ROS_DEBUG("Adding moving segment from '%s' to '%s'", root.c_str(), child.getName().c_str());
    }
    addChildren(children[i]);
  }
}

void RobotState::setJointState(const sensor_msgs::JointState& joint_state)
{
  if (joint_state.name.size() != joint_state.position.size()) {
    if (joint_state.position.empty()){
      const int throttleSeconds = 300;
      ROS_WARN_THROTTLE(throttleSeconds,
          "Robot state publisher ignored a JointState message about joint(s) "
          "\"%s\"(,...) whose position member was empty. This message will "
          "not reappear for %d seconds.", joint_state.name[0].c_str(),
          throttleSeconds);
    } else {
      ROS_ERROR("Robot state publisher ignored an invalid JointState message");
    }
    return;
  }

  // update joint positions from joint_stateg message
  // if it doesn't contain all the joints then old positions are used
  for (size_t i = 0; i < joint_state.name.size(); ++i) {
    const auto& joint_name = joint_state.name[i];
    const auto& joint_position = joint_state.position[i];
    // TODO(lucasw) optionally enforce joint limits
    joint_positions_[joint_name] = joint_position;
  }

  for (const auto& mimic_pair : mimic_) {
    const auto& joint_dst_name = mimic_pair.first;
    const auto& joint_src = mimic_pair.second;

    if (joint_positions_.count(joint_src->joint_name) > 0) {
      const double pos = joint_positions_[joint_src->joint_name] * joint_src->multiplier + joint_src->offset;
      joint_positions_[joint_dst_name] = pos;
    }
  }
}

sensor_msgs::JointState RobotState::getJointStates() const
{
  sensor_msgs::JointState js;
  for (const auto& jnt : joint_positions_) {
    js.name.push_back(jnt.first);
    js.position.push_back(jnt.second);
  }

  return js;
}

urdf::JointConstSharedPtr RobotState::getJoint(const std::string& joint_name) const
{
  return model_.getJoint(joint_name);
}

// get moving transforms
tf2_msgs::TFMessage RobotState::getTransforms(const ros::Time& time, const std::string& tf_prefix) const
{
  tf2_msgs::TFMessage tfm;

  // loop over all joints
  for (const auto& jnt : joint_positions_) {
    const auto& joint_name = jnt.first;
    const auto& joint_position = jnt.second;

    if (segments_.count(joint_name) > 0) {
      const auto& seg = segments_.at(joint_name);
      geometry_msgs::TransformStamped tf_transform = tf2::kdlToTransform(seg.segment.pose(joint_position));
      tf_transform.header.stamp = time;
      tf_transform.header.frame_id = prefix_frame(tf_prefix, seg.root);
      tf_transform.child_frame_id = prefix_frame(tf_prefix, seg.tip);
      tfm.transforms.push_back(tf_transform);
    }
    else {
      ROS_WARN_THROTTLE(10, "Joint state with name: \"%s\" was received but not found in URDF", joint_name.c_str());
    }
  }
  return tfm;
}

// get fixed transforms
tf2_msgs::TFMessage RobotState::getFixedTransforms(const ros::Time& time, const std::string& tf_prefix) const
{
  ROS_DEBUG("Getting transforms for fixed joints");
  tf2_msgs::TFMessage tfm;

  // loop over all fixed segments
  for (const auto& seg : segments_fixed_) {
    geometry_msgs::TransformStamped tf_transform = tf2::kdlToTransform(seg.second.segment.pose(0));
    // TODO(lucasw) this time isn't need since static?
    tf_transform.header.stamp = time;
    tf_transform.header.frame_id = prefix_frame(tf_prefix, seg.second.root);
    tf_transform.child_frame_id = prefix_frame(tf_prefix, seg.second.tip);
    tfm.transforms.push_back(tf_transform);
  }

  return tfm;
}

void addTransformsToBuffer(tf2::BufferCore& bc,
                           const tf2_msgs::TFMessage& tfm,
                           const bool is_static)
{
  const std::string authority = is_static ? "robot_state_add_static_transforms" : "robot_state_add_transforms";
  for (const auto& tfs : tfm.transforms) {
    if (tfs.child_frame_id == tfs.header.frame_id) {
      ROS_WARN_STREAM("same parent and child: '" << tfs.child_frame_id << "'");
      continue;
    }
    bc.setTransform(tfs, authority, is_static);
  }
}

void RobotState::toBufferCore(tf2::BufferCore& bc, const ros::Time& stamp,
                              const std::string& tf_prefix) const
{
  addTransformsToBuffer(bc, getFixedTransforms(stamp, tf_prefix), true);
  addTransformsToBuffer(bc, getTransforms(stamp, tf_prefix), false);
}

}  // namespace robot_state_publisher
