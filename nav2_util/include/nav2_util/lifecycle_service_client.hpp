// Copyright (c) 2019 Intel Corporation
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

#ifndef NAV2_UTIL__LIFECYCLE_SERVICE_CLIENT_HPP_
#define NAV2_UTIL__LIFECYCLE_SERVICE_CLIENT_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_util/service_client.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_util
{

/// Helper functions to interact with a lifecycle node.
class LifecycleServiceClient
{
public:
  /**
   * @brief 构造函数，使用内部自动创建的 ROS 2 节点构造生命周期服务客户端
   *
   * 客户端会连接目标生命周期节点的 change_state 和 get_state 服务，
   * 并创建一个内部节点用于服务通信。
   *
   * @param lifecycle_node_name 目标生命周期节点的名称
   */
  explicit LifecycleServiceClient(const std::string & lifecycle_node_name);
  /**
   * @brief 构造函数，使用已有的父节点构造生命周期服务客户端
   *
   * 客户端会连接目标生命周期节点的 change_state 和 get_state 服务，
   * 并使用 parent_node 进行服务通信。
   *
   * @param lifecycle_node_name 目标生命周期节点的名称
   * @param parent_node 已有 ROS 2 父节点的共享指针
   */
  LifecycleServiceClient(const std::string & lifecycle_node_name, rclcpp::Node::SharedPtr parent_node);


  ~LifecycleServiceClient()
  {
    change_state_.stop();
    get_state_.stop();
  }

  /**
   * @brief Trigger a state change | 在指定超时时间内触发生命周期状态转换
   * @param transition 要执行的生命周期转换 ID
   * @param timeout 等待服务响应的最长时间
   * @return true 服务调用成功
   * @return Throws std::runtime_error on failure | false 服务调用失败
   */
  bool change_state(
    const uint8_t transition,  // takes a lifecycle_msgs::msg::Transition id
    const std::chrono::seconds timeout);

  /// Trigger a state change, returning result
  bool change_state(std::uint8_t transition);

  /**
   * @brief Trigger a state change, returning result | 获取节点当前的生命周期状态 ID
   * @param timeout 等待服务响应的最长时间
   * @return uint8_t 当前生命周期状态 ID
   * @throws Throws std::runtime_error on failure | std::runtime_error 服务不可用或请求超时时抛出
   */
  uint8_t get_state(const std::chrono::seconds timeout = std::chrono::seconds(2));

protected:
  rclcpp::Node::SharedPtr node_;         // 用于创建和运行生命周期服务客户端的 ROS 2 节点
  ServiceClient<lifecycle_msgs::srv::ChangeState> change_state_;    // 用于请求目标节点执行生命周期状态转换
  ServiceClient<lifecycle_msgs::srv::GetState> get_state_;          // 用于查询目标生命周期节点的当前状态
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__LIFECYCLE_SERVICE_CLIENT_HPP_
