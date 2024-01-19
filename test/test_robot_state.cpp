/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2020, Open Source Robotics Foundation, Inc.
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


#include <string>
#include <utility>

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "robot_state_publisher/robot_state.h"

void printTransforms(robot_state_publisher::RobotState& rs)
{
  const auto stamp = ros::Time::now();
  const auto tfm = rs.getTransforms(stamp);
  ROS_WARN_STREAM(tfm.transforms.size() << " transforms");
  for (const auto& tf : tfm.transforms) {
    ROS_WARN_STREAM(tf);
  }
}

TEST(RobotStatePublisher, printTransforms)
{
  robot_state_publisher::RobotState rs;

  ROS_WARN_STREAM("no joints set");
  printTransforms(rs);
  const auto stamp = ros::Time::now();
  const auto tfm0 = rs.getTransforms(stamp);
  ASSERT_TRUE(tfm0.transforms.size() == 0);

  sensor_msgs::JointState js;
  js.header.stamp = stamp;
  js.name.push_back("joint1");

  double angle = 0.0;
  js.position.push_back(angle);
  const size_t num = 5;
  for (size_t i = 0; i <num; ++i) {
    js.position[0] = angle;
    rs.setJointState(js);
    ROS_WARN_STREAM(i << " angle " << angle);
    printTransforms(rs);
    angle += 1.5707963267948966 / (num - 1);
  }

  const auto tfm1 = rs.getTransforms(stamp);
  ASSERT_TRUE(tfm1.transforms.size() > 0);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "test_robot_state");
  ::testing::InitGoogleTest(&argc, argv);
  ROS_WARN_STREAM("test");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
