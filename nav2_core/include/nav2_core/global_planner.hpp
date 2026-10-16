// Copyright (c) 2019 Samsung Research America
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

#ifndef NAV2_CORE__GLOBAL_PLANNER_HPP_
#define NAV2_CORE__GLOBAL_PLANNER_HPP_

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace nav2_core
{

/**
 * @class GlobalPlanner
 * @brief Abstract interface for global planners to adhere to with pluginlib | 使用 pluginlib 的全局规划器需要遵循的抽象接口
 */
class GlobalPlanner
{
public:
  using Ptr = std::shared_ptr<GlobalPlanner>;

  /**
   * @brief Virtual destructor | 虚析构函数
   */
  virtual ~GlobalPlanner() {}

  /**
   * @param  parent pointer to user's node | 指向用户节点的指针
   * @param  name The name of this planner | 此规划器的名称
   * @param  tf A pointer to a TF buffer| 指向 TF 缓冲区的指针
   * @param  costmap_ros A pointer to the costmap | 指向代价地图的指针
   */
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) = 0;

  /**
   * @brief Method to cleanup resources used on shutdown. | 清理关闭时所用资源的方法
   */
  virtual void cleanup() = 0;

  /**
   * @brief Method to active planner and any threads involved in execution. | 激活规划器以及执行过程中所涉及线程的方法
   */
  virtual void activate() = 0;

  /**
   * @brief Method to deactive planner and any threads involved in execution. | 停用规划器以及执行过程中所涉及线程的方法
   */
  virtual void deactivate() = 0;

  /**
   * @brief Method create the plan from a starting and ending goal. | 根据起始位姿和目标位姿创建路径的方法
   * @param start The starting pose of the robot | 机器人的起始位姿
   * @param goal  The goal pose of the robot | 机器人的目标位姿
   * @param cancel_checker Function to check if the action has been canceled | 用于检查动作是否已被取消的函数
   * @return The sequence of poses to get from start to goal, if any | 从起点到目标点的位姿序列（如果存在可行路径）
   */
  virtual nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) = 0;
};

}  // namespace nav2_core

#endif  // NAV2_CORE__GLOBAL_PLANNER_HPP_
