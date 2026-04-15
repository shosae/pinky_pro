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

#ifndef PINKY_DOCKING_PLUGINS__POSE_FILTER_HPP_
#define PINKY_DOCKING_PLUGINS__POSE_FILTER_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pinky_docking_plugins
{

class PoseFilter
{
public:
  PoseFilter(double coef, double timeout);

  geometry_msgs::msg::PoseStamped update(const geometry_msgs::msg::PoseStamped & measurement);

protected:
  void filter(double & filt, double meas);

  double coef_{0.0};
  double timeout_{0.0};
  geometry_msgs::msg::PoseStamped pose_;
};

}  // namespace pinky_docking_plugins

#endif  // PINKY_DOCKING_PLUGINS__POSE_FILTER_HPP_
