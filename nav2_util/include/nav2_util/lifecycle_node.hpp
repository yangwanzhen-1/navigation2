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

#ifndef NAV2_UTIL__LIFECYCLE_NODE_HPP_
#define NAV2_UTIL__LIFECYCLE_NODE_HPP_

#include <memory>
#include <string>
#include <thread>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "nav2_util/node_thread.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "bondcpp/bond.hpp"
#include "bond/msg/constants.hpp"

namespace nav2_util
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @class nav2_util::LifecycleNode
 * @brief A lifecycle node wrapper to enable common Nav2 needs such as manipulating parameters
 */
class LifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @brief A lifecycle node constructor | 生命周期节点构造函数
   * @param node_name Name for the node | 节点名称
   * @param namespace Namespace for the node, if any  | 节点命名空间，没有时使用空字符串
   * @param options Node options | 创建节点时使用的配置选项
   */
  LifecycleNode(
    const std::string & node_name,
    const std::string & ns = "",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~LifecycleNode();

  typedef struct
  {
    double from_value;
    double to_value;
    double step;
  } floating_point_range;

  typedef struct
  {
    int from_value;
    int to_value;
    int step;
  } integer_range;

  /**
   * @brief Declare a parameter that has no integer or floating point range constraints
   * @param node_name Name of parameter
   * @param default_value Default node value to add
   * @param description Node description
   * @param additional_constraints Any additional constraints on the parameters to list
   * @param read_only Whether this param should be considered read only
   */
  void add_parameter(
    const std::string & name, const rclcpp::ParameterValue & default_value,
    const std::string & description = "", const std::string & additional_constraints = "",
    bool read_only = false)
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.name = name;
    descriptor.description = description;
    descriptor.additional_constraints = additional_constraints;
    descriptor.read_only = read_only;

    declare_parameter(descriptor.name, default_value, descriptor);
  }

  /**
   * @brief Declare a parameter that has a floating point range constraint
   * @param node_name Name of parameter
   * @param default_value Default node value to add
   * @param fp_range floating point range
   * @param description Node description
   * @param additional_constraints Any additional constraints on the parameters to list
   * @param read_only Whether this param should be considered read only
   */
  void add_parameter(
    const std::string & name, const rclcpp::ParameterValue & default_value,
    const floating_point_range fp_range,
    const std::string & description = "", const std::string & additional_constraints = "",
    bool read_only = false)
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.name = name;
    descriptor.description = description;
    descriptor.additional_constraints = additional_constraints;
    descriptor.read_only = read_only;
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = fp_range.from_value;
    descriptor.floating_point_range[0].to_value = fp_range.to_value;
    descriptor.floating_point_range[0].step = fp_range.step;

    declare_parameter(descriptor.name, default_value, descriptor);
  }

  /**
   * @brief Declare a parameter that has an integer range constraint
   * @param node_name Name of parameter
   * @param default_value Default node value to add
   * @param integer_range Integer range
   * @param description Node description
   * @param additional_constraints Any additional constraints on the parameters to list
   * @param read_only Whether this param should be considered read only
   */
  void add_parameter(
    const std::string & name, const rclcpp::ParameterValue & default_value,
    const integer_range int_range,
    const std::string & description = "", const std::string & additional_constraints = "",
    bool read_only = false)
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.name = name;
    descriptor.description = description;
    descriptor.additional_constraints = additional_constraints;
    descriptor.read_only = read_only;
    descriptor.integer_range.resize(1);
    descriptor.integer_range[0].from_value = int_range.from_value;
    descriptor.integer_range[0].to_value = int_range.to_value;
    descriptor.integer_range[0].step = int_range.step;

    declare_parameter(descriptor.name, default_value, descriptor);
  }

  /**
   * @brief Get a shared pointer of thiss | 获取指向当前对象的共享指针
   */
  std::shared_ptr<nav2_util::LifecycleNode> shared_from_this()
  {
    return std::static_pointer_cast<nav2_util::LifecycleNode>(
      rclcpp_lifecycle::LifecycleNode::shared_from_this());
  }

  /**
   * @brief Abstracted on_error state transition callback, since unimplemented as of 2020
   * in the managed ROS2 node state machine
   * @param state State prior to error transition
   * @return Return type for success or failed transition to error state
   */
  nav2_util::CallbackReturn on_error(const rclcpp_lifecycle::State & /*state*/)
  {
    RCLCPP_FATAL(
      get_logger(),
      "Lifecycle node %s does not have error state implemented", get_name());
    return nav2_util::CallbackReturn::SUCCESS;
  }

  /**
   * @brief Automatically configure and active the node | 自动配置并激活当前生命周期节点
   */
  void autostart();

  /**
   * @brief Perform preshutdown activities before our Context is shutdown.
   * Note that this is related to our Context's shutdown sequence, not the
   * lifecycle node state machine.
   * 
   * | 在 ROS 2 Context 关闭之前执行预关闭操作。该函数属于 Context 的关闭流程，与生命周期节点状态机无直接关系。
   */
  virtual void on_rcl_preshutdown();

  /**
   * @brief Create bond connection to lifecycle manager | 创建与生命周期管理器之间的 bond 连接
   */
  void createBond();

  /**
   * @brief Destroy bond connection to lifecycle manager | 销毁与生命周期管理器之间的 bond 连接
   */
  void destroyBond();

protected:
  /**
   * @brief Print notifications for lifecycle node | 打印生命周期节点的启动信息和状态转换提示信息
   */
  void printLifecycleNodeNotification();

  /**
   * Register our preshutdown callback for this Node's rcl Context.
   * The callback fires before this Node's Context is shutdown.
   * Note this is not directly related to the lifecycle state machine.
   * 
   * | 为当前节点的 ROS 2 Context 注册预关闭回调函数。该回调会在 Context 正式关闭之前触发，与生命周期节点的状态机没有直接关系。
   */
  void register_rcl_preshutdown_callback();
  std::unique_ptr<rclcpp::PreShutdownCallbackHandle> rcl_preshutdown_cb_handle_{nullptr};

  /**
   * Run some common cleanup steps shared between rcl preshutdown and destruction.| 用于保存和管理已经注册的 ROS 2 Context 预关闭回调
   */
  void runCleanups();

  // Connection to tell that server is still up | 用于通过 Bond 心跳表明当前生命周期节点仍在正常运行
  std::shared_ptr<bond::Bond> bond_{nullptr};
  double bond_heartbeat_period;                     // Bond 心跳消息的发送周期，单位为秒
  rclcpp::TimerBase::SharedPtr autostart_timer_;    // 节点自动启动使用的定时器，用于在构造函数完成后异步执行生命周期状态转换
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__LIFECYCLE_NODE_HPP_
