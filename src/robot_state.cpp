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
    return;
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model_, tree)) {
    // TODO(lucasw) throw
    ROS_ERROR("Failed to extract kdl tree from xml robot description");
    return;
  }

  for (std::map< std::string, urdf::JointSharedPtr >::iterator i = model_.joints_.begin();
       i != model_.joints_.end(); i++) {
    if (i->second->mimic) {
      mimic_.insert(std::make_pair(i->first, i->second->mimic));
    }
  }

  // walk the tree and add segments to segments_
  addChildren(tree.getRootSegment());
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
        ROS_INFO("Floating joint. Not adding segment from %s to %s."
                 "  This TF can not be published based on joint_states info",
                 root.c_str(), child.getName().c_str());
      }
      else {
        segments_fixed_.insert(make_pair(child.getJoint().getName(), s));
        ROS_DEBUG("Adding fixed segment from %s to %s", root.c_str(), child.getName().c_str());
      }
    }
    else {
      segments_.insert(make_pair(child.getJoint().getName(), s));
      ROS_DEBUG("Adding moving segment from %s to %s", root.c_str(), child.getName().c_str());
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
    joint_positions_.insert(std::make_pair(joint_state.name[i], joint_state.position[i]));
  }

  for (MimicMap::iterator i = mimic_.begin(); i != mimic_.end(); i++) {
    // TODO(lucasw) contains()
    if (joint_positions_.find(i->second->joint_name) != joint_positions_.end()) {
      const double pos = joint_positions_[i->second->joint_name] * i->second->multiplier + i->second->offset;
      joint_positions_.insert(std::make_pair(i->first, pos));
    }
  }
}

// get moving transforms
tf2_msgs::TFMessage RobotState::getTransforms(const ros::Time& time)
{
  tf2_msgs::TFMessage tfm;

  // loop over all joints
  // TODO(lucasw) modernize this for statement
  for (std::map<std::string, double>::const_iterator jnt = joint_positions_.begin();
       jnt != joint_positions_.end(); jnt++) {
    std::map<std::string, SegmentPair>::const_iterator seg = segments_.find(jnt->first);
    if (seg != segments_.end()) {
      geometry_msgs::TransformStamped tf_transform = tf2::kdlToTransform(seg->second.segment.pose(jnt->second));
      tf_transform.header.stamp = time;
      tf_transform.header.frame_id = seg->second.root;
      tf_transform.child_frame_id = seg->second.tip;
      tfm.transforms.push_back(tf_transform);
    }
    else {
      ROS_WARN_THROTTLE(10, "Joint state with name: \"%s\" was received but not found in URDF", jnt->first.c_str());
    }
  }
  return tfm;
}

// publish fixed transforms
tf2_msgs::TFMessage RobotState::getFixedTransforms(const ros::Time& time)
{
  ROS_DEBUG("Publishing transforms for fixed joints");
  tf2_msgs::TFMessage tfm;

  // loop over all fixed segments
  // TODO(lucasw) modernize this for statement
  for (std::map<std::string, SegmentPair>::const_iterator seg = segments_fixed_.begin();
       seg != segments_fixed_.end(); seg++) {
    geometry_msgs::TransformStamped tf_transform = tf2::kdlToTransform(seg->second.segment.pose(0));
    // TODO(lucasw) this time isn't need since static?
    tf_transform.header.stamp = time;
    tf_transform.header.frame_id = seg->second.root;
    tf_transform.child_frame_id = seg->second.tip;
    tfm.transforms.push_back(tf_transform);
  }

  return tfm;
}

}  // namespace robot_state_publisher
