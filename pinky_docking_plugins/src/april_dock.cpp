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

#include <cmath>

#include "nav2_util/node_utils.hpp"
#include "pinky_docking_plugins/april_dock.hpp"

namespace pinky_docking_plugins
{

void AprilDock::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf)
{
  name_ = name;
  tf2_buffer_ = tf;
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".use_external_detection_pose", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_timeout", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_translation_x", rclcpp::ParameterValue(-0.20));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_translation_y", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_rotation_yaw", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_rotation_pitch", rclcpp::ParameterValue(1.57));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_rotation_roll", rclcpp::ParameterValue(-1.57));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".filter_coef", rclcpp::ParameterValue(0.1));

  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".use_stall_detection", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".stall_joint_names", rclcpp::PARAMETER_STRING_ARRAY);
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".stall_velocity_threshold", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".stall_effort_threshold", rclcpp::ParameterValue(1.0));

  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".docking_threshold", rclcpp::ParameterValue(0.05));

  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".staging_x_offset", rclcpp::ParameterValue(-0.7));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".staging_yaw_offset", rclcpp::ParameterValue(0.0));

  node_->get_parameter(name + ".use_external_detection_pose", use_external_detection_pose_);
  node_->get_parameter(name + ".external_detection_timeout", external_detection_timeout_);
  node_->get_parameter(
    name + ".external_detection_translation_x", external_detection_translation_x_);
  node_->get_parameter(
    name + ".external_detection_translation_y", external_detection_translation_y_);
  double yaw, pitch, roll;
  node_->get_parameter(name + ".external_detection_rotation_yaw", yaw);
  node_->get_parameter(name + ".external_detection_rotation_pitch", pitch);
  node_->get_parameter(name + ".external_detection_rotation_roll", roll);
  external_detection_rotation_.setEuler(pitch, roll, yaw);
  node_->get_parameter(name + ".stall_velocity_threshold", stall_velocity_threshold_);
  node_->get_parameter(name + ".stall_effort_threshold", stall_effort_threshold_);
  node_->get_parameter(name + ".docking_threshold", docking_threshold_);
  node_->get_parameter("base_frame", base_frame_id_);
  node_->get_parameter(name + ".staging_x_offset", staging_x_offset_);
  node_->get_parameter(name + ".staging_yaw_offset", staging_yaw_offset_);

  double filter_coef;
  node_->get_parameter(name + ".filter_coef", filter_coef);
  filter_ = std::make_shared<PoseFilter>(filter_coef, external_detection_timeout_);

  if (use_external_detection_pose_) {
    dock_pose_.header.stamp = rclcpp::Time(0);
    dock_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "detected_dock_pose", 1,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
        detected_dock_pose_ = *pose;
      });
  }

  bool use_stall_detection;
  node_->get_parameter(name + ".use_stall_detection", use_stall_detection);
  if (use_stall_detection) {
    is_stalled_ = false;
    node_->get_parameter(name + ".stall_joint_names", stall_joint_names_);
    if (stall_joint_names_.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "stall_joint_names cannot be empty!");
    }
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 1,
      std::bind(&AprilDock::jointStateCallback, this, std::placeholders::_1));
  }

  dock_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("dock_pose", 1);
  filtered_dock_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    "filtered_dock_pose", 1);
  staging_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("staging_pose", 1);
}

geometry_msgs::msg::PoseStamped AprilDock::getStagingPose(
  const geometry_msgs::msg::Pose & pose, const std::string & frame)
{
  if (!use_external_detection_pose_) {
    dock_pose_.header.frame_id = frame;
    dock_pose_.pose = pose;
  }

  const double yaw = tf2::getYaw(pose.orientation);
  geometry_msgs::msg::PoseStamped staging_pose;
  staging_pose.header.frame_id = frame;
  staging_pose.header.stamp = node_->now();
  staging_pose.pose = pose;
  staging_pose.pose.position.x += std::cos(yaw) * staging_x_offset_;
  staging_pose.pose.position.y += std::sin(yaw) * staging_x_offset_;
  tf2::Quaternion orientation;
  orientation.setEuler(0.0, 0.0, yaw + staging_yaw_offset_);
  staging_pose.pose.orientation = tf2::toMsg(orientation);

  staging_pose_pub_->publish(staging_pose);
  return staging_pose;
}

bool AprilDock::getRefinedPose(geometry_msgs::msg::PoseStamped & pose, std::string)
{
  if (!use_external_detection_pose_) {
    dock_pose_pub_->publish(pose);
    dock_pose_ = pose;
    return true;
  }

  geometry_msgs::msg::PoseStamped detected = detected_dock_pose_;
  auto timeout = rclcpp::Duration::from_seconds(external_detection_timeout_);
  if (node_->now() - detected.header.stamp > timeout) {
    RCLCPP_WARN(node_->get_logger(), "Lost detection or did not detect: timeout exceeded");
    return false;
  }

  if (detected.header.frame_id != pose.header.frame_id) {
    try {
      if (!tf2_buffer_->canTransform(
          pose.header.frame_id, detected.header.frame_id,
          detected.header.stamp, rclcpp::Duration::from_seconds(0.2)))
      {
        RCLCPP_WARN(node_->get_logger(), "Failed to transform detected dock pose");
        return false;
      }
      tf2_buffer_->transform(detected, detected, pose.header.frame_id);
    } catch (const tf2::TransformException &) {
      RCLCPP_WARN(node_->get_logger(), "Failed to transform detected dock pose");
      return false;
    }
  }

  detected = filter_->update(detected);
  filtered_dock_pose_pub_->publish(detected);

  geometry_msgs::msg::PoseStamped just_orientation;
  just_orientation.pose.orientation = tf2::toMsg(external_detection_rotation_);
  geometry_msgs::msg::TransformStamped transform;
  transform.transform.rotation = detected.pose.orientation;
  tf2::doTransform(just_orientation, just_orientation, transform);

  tf2::Quaternion orientation;
  orientation.setEuler(0.0, 0.0, tf2::getYaw(just_orientation.pose.orientation));
  dock_pose_.pose.orientation = tf2::toMsg(orientation);

  dock_pose_.header = detected.header;
  dock_pose_.pose.position = detected.pose.position;
  const double yaw = tf2::getYaw(dock_pose_.pose.orientation);
  dock_pose_.pose.position.x += std::cos(yaw) * external_detection_translation_x_ -
    std::sin(yaw) * external_detection_translation_y_;
  dock_pose_.pose.position.y += std::sin(yaw) * external_detection_translation_x_ +
    std::cos(yaw) * external_detection_translation_y_;
  dock_pose_.pose.position.z = 0.0;

  dock_pose_pub_->publish(dock_pose_);
  pose = dock_pose_;
  return true;
}

bool AprilDock::isDocked()
{
  if (joint_state_sub_) {
    return is_stalled_;
  }

  if (dock_pose_.header.frame_id.empty()) {
    return false;
  }

  geometry_msgs::msg::PoseStamped base_pose;
  base_pose.header.stamp = rclcpp::Time(0);
  base_pose.header.frame_id = base_frame_id_;
  base_pose.pose.orientation.w = 1.0;
  try {
    tf2_buffer_->transform(base_pose, base_pose, dock_pose_.header.frame_id);
  } catch (const tf2::TransformException &) {
    return false;
  }

  const double distance = std::hypot(
    base_pose.pose.position.x - dock_pose_.pose.position.x,
    base_pose.pose.position.y - dock_pose_.pose.position.y);
  return distance < docking_threshold_;
}

void AprilDock::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr state)
{
  double velocity = 0.0;
  double effort = 0.0;
  for (size_t i = 0; i < state->name.size(); ++i) {
    for (auto & name : stall_joint_names_) {
      if (state->name[i] == name) {
        velocity += std::abs(state->velocity[i]);
        effort += std::abs(state->effort[i]);
      }
    }
  }

  effort /= stall_joint_names_.size();
  velocity /= stall_joint_names_.size();
  is_stalled_ = (velocity < stall_velocity_threshold_) && (effort > stall_effort_threshold_);
}

}  // namespace pinky_docking_plugins

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(pinky_docking_plugins::AprilDock, opennav_docking_core::ChargingDock)
