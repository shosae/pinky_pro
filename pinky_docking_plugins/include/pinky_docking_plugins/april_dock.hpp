// Copyright (c) 2024 Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PINKY_DOCKING_PLUGINS__APRIL_DOCK_HPP_
#define PINKY_DOCKING_PLUGINS__APRIL_DOCK_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

#include "opennav_docking_core/non_charging_dock.hpp"
#include "pinky_docking_plugins/pose_filter.hpp"

namespace pinky_docking_plugins
{

class AprilDock : public opennav_docking_core::NonChargingDock
{
public:
  AprilDock()
  : NonChargingDock()
  {}

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf) override;

  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}

  geometry_msgs::msg::PoseStamped getStagingPose(
    const geometry_msgs::msg::Pose & pose, const std::string & frame) override;

  bool getRefinedPose(geometry_msgs::msg::PoseStamped & pose, std::string id) override;

  bool isDocked() override;

protected:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr state);

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr dock_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr dock_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr filtered_dock_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr staging_pose_pub_;

  geometry_msgs::msg::PoseStamped detected_dock_pose_;
  geometry_msgs::msg::PoseStamped dock_pose_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  std::vector<std::string> stall_joint_names_;
  double stall_velocity_threshold_{1.0};
  double stall_effort_threshold_{1.0};
  bool is_stalled_{false};

  bool use_external_detection_pose_{false};
  double external_detection_timeout_{1.0};
  tf2::Quaternion external_detection_rotation_;
  double external_detection_translation_x_{-0.20};
  double external_detection_translation_y_{0.0};

  std::shared_ptr<PoseFilter> filter_;

  double docking_threshold_{0.05};
  std::string base_frame_id_;
  double staging_x_offset_{-0.7};
  double staging_yaw_offset_{0.0};

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
};

}  // namespace pinky_docking_plugins

#endif  // PINKY_DOCKING_PLUGINS__APRIL_DOCK_HPP_
