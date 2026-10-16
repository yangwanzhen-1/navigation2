// Copyright (c) 2019 Intel Corporation
// Copyright (c) 2022 Samsung Research America
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

#ifndef NAV2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_
#define NAV2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_

#include <map>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "nav2_util/lifecycle_service_client.hpp"
#include "nav2_util/node_thread.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "bondcpp/bond.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"


namespace nav2_lifecycle_manager
{
using namespace std::chrono_literals;  // NOLINT

using nav2_msgs::srv::ManageLifecycleNodes;

/// @brief Enum for keeping track of the state of managed nodes | 用于记录受管理节点状态的枚举
enum NodeState
{
  UNCONFIGURED,  // 未配置：节点已创建，但尚未完成配置
  ACTIVE,        // 活跃：节点已激活，正在正常运行
  INACTIVE,      // 非活跃：节点已完成配置，但尚未激活或已停用
  FINALIZED,     // 已终止：节点已关闭
  UNKNOWN,       // 未知：无法确定节点当前的生命周期状态
};

/**
 * @class nav2_lifecycle_manager::LifecycleManager
 * @brief Implements service interface to transition the lifecycle nodes of
 * Nav2 stack. It receives transition request and then uses lifecycle
 * interface to change lifecycle node's state.
 */
class LifecycleManager : public rclcpp::Node
{
public:
  /**
   * @brief A constructor for nav2_lifecycle_manager::LifecycleManager | nav2_lifecycle_manager::LifecycleManager 的构造函数
   * @param options Additional options to control creation of the node. | 用于控制节点创建的附加选项
   */
  explicit LifecycleManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief A destructor for nav2_lifecycle_manager::LifecycleManager
   */
  ~LifecycleManager();

protected:
  // Callback group used by services and timers | 服务和定时器使用的回调组
  rclcpp::CallbackGroup::SharedPtr callback_group_;         // 生命周期管理服务和定时器使用的回调组，用于控制这些回调的执行方式
  std::unique_ptr<nav2_util::NodeThread> service_thread_;   // 独立运行该回调组的节点线程，使管理服务和定时器能够被持续处理

  // The services provided by this node | 本节点提供的 ROS 2 服务
  rclcpp::Service<ManageLifecycleNodes>::SharedPtr manager_srv_;     // 管理生命周期节点状态的服务，用于启动、关闭、重置、暂停和恢复导航系统
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr is_active_srv_;  // 查询受管理节点当前是否处于激活状态的服务
  /**
   * @brief Lifecycle node manager callback function | 生命周期节点管理器回调函数
   * @param request_header Header of the service request | 服务请求的请求头
   * @param request Service request | 服务请求
   * @param response Service response | 服务响应
   */
  void managerCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ManageLifecycleNodes::Request> request,
    std::shared_ptr<ManageLifecycleNodes::Response> response);

  /**
   * @brief Trigger callback function that checks whether the managed nodes are active |
   * 检查受管理节点是否处于激活状态的 Trigger 服务回调函数
   * @param request_header Header of the service request | 服务请求的请求头
   * @param request Service request | 服务请求
   * @param response Service response | 服务响应
   */
  void isActiveCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // Support functions for the service calls | 服务调用的辅助函数
  /**
   * @brief Start up the managed nodes | 启动所有受管理节点
   * @return true if startup succeeds, false otherwise | 启动成功返回 true，否则返回 false
   */
  bool startup();

  /**
   * @brief Configure the managed nodes | 配置所有受管理节点
   * @return true if configuration succeeds, false otherwise | 配置成功返回 true，否则返回 false
   */
  bool configure();

  /**
   * @brief Clean up the managed nodes | 清理所有受管理节点
   * @return true if cleanup succeeds, false otherwise | 清理成功返回 true，否则返回 false
   */
  bool cleanup();

  /**
   * @brief Deactivate, clean up, and shut down all managed nodes | 停用、清理并关闭所有受管理节点
   * @return true if shutdown succeeds, false otherwise | 关闭成功返回 true，否则返回 false
   */
  bool shutdown();

  /**
   * @brief Reset all managed nodes | 重置所有受管理节点
   * @param hard_reset Whether to continue resetting when a state transition fails |
   * 状态转换失败时是否继续重置其他节点
   * @return true if reset succeeds, false otherwise | 重置成功返回 true，否则返回 false
   */
  bool reset(bool hard_reset = false);

  /**
   * @brief Pause all managed nodes | 暂停所有受管理节点
   * @return true if pause succeeds, false otherwise | 暂停成功返回 true，否则返回 false
   */
  bool pause();

  /**
   * @brief Resume all managed nodes | 恢复所有受管理节点
   * @return true if resume succeeds, false otherwise | 恢复成功返回 true，否则返回 false
   */
  bool resume();

  /**
   * @brief Perform preshutdown activities before this node context is shut down |
   * 在本节点的上下文关闭之前执行预关闭操作
   * @details This callback belongs to the context shutdown sequence and is not directly
   * related to the lifecycle state machine or shutdown() |
   * 此回调属于上下文关闭流程，与生命周期状态机或 shutdown() 函数没有直接关系
   */
  void onRclPreshutdown();

  // Support function for creating service clients | 创建服务客户端的辅助函数
  /**
   * @brief Create service clients for all managed lifecycle nodes | 为所有受管理的生命周期节点创建服务客户端
   */
  void createLifecycleServiceClients();

  // Support functions for shutdown | 关闭操作的辅助函数
  /**
   * @brief Shut down all managed lifecycle nodes | 关闭所有受管理的生命周期节点
   */
  void shutdownAllNodes();

  /**
   * @brief Destroy all lifecycle service clients | 销毁所有生命周期服务客户端
   */
  void destroyLifecycleServiceClients();

  // Support function for creating Bond timer | 创建 Bond 定时器的辅助函数
  /**
   * @brief Create the timer that monitors Bond connections | 创建监控 Bond 连接的定时器
   */
  void createBondTimer();

  // Support function for creating Bond connection | 创建 Bond 连接的辅助函数
  /**
   * @brief Create a Bond connection to a managed node | 与受管理节点创建 Bond 连接
   * @param node_name Name of the managed node | 受管理节点的名称
   * @return true if the connection is established, false otherwise |
   * 连接建立成功返回 true，否则返回 false
   */
  bool createBondConnection(const std::string & node_name);

  // Support function for destroying Bond connections | 销毁 Bond 连接的辅助函数
  /**
   * @brief Stop and destroy the Bond monitoring timer | 停止并销毁 Bond 监控定时器
   */
  void destroyBondTimer();

  // Support function for checking Bond connections | 检查 Bond 连接的辅助函数
  /**
   * @brief Check Bond connections and shut down the system if a managed node is unresponsive |
   * 检查 Bond 连接，并在受管理节点无响应时关闭系统
   */
  void checkBondConnections();

  // Support function for checking Bond reconnection after respawn | 检查节点重启后 Bond 是否恢复的辅助函数
  /**
   * @brief Check Bond reconnections and restart the system after managed nodes respawn |
   * 检查 Bond 重连，并在受管理节点重新启动后恢复系统
   */
  void checkBondRespawnConnection();

  /**
   * @brief Transition one managed node to the target state | 将一个受管理节点转换到目标状态
   * @param node_name Name of the managed node | 受管理节点的名称
   * @param transition Lifecycle transition identifier | 生命周期转换标识
   * @return true if the transition succeeds, false otherwise | 转换成功返回 true，否则返回 false
   */
  bool changeStateForNode(
    const std::string & node_name,
    std::uint8_t transition);

  /**
   * @brief Transition every managed node to the target state | 将所有受管理节点转换到目标状态
   * @param transition Lifecycle transition identifier | 生命周期转换标识
   * @param hard_change Whether to continue when a node transition fails |
   * 某个节点转换失败时是否继续转换其他节点
   * @return true if all required transitions succeed, false otherwise |
   * 所有需要执行的转换成功返回 true，否则返回 false
   */
  bool changeStateForAllNodes(std::uint8_t transition, bool hard_change = false);

  // Convenience function to highlight console output | 高亮控制台输出的便捷函数
  /**
   * @brief Print a highlighted message to the console | 在控制台打印高亮消息
   * @param msg Message to print | 要打印的消息
   */
  void message(const std::string & msg);

  // Diagnostics functions | 诊断功能
  /**
   * @brief Update diagnostics with the state of Nav2 nodes | 使用 Nav2 节点状态更新诊断信息
   * @param stat Diagnostic status wrapper to update | 要更新的诊断状态封装对象
   */
  void CreateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /**
   * @brief Register a preshutdown callback for this node rcl context |
   * 为本节点的 rcl 上下文注册预关闭回调
   * @details The callback runs before the context is shut down and is not directly related
   * to the lifecycle state machine or the shutdown() member function |
   * 该回调在上下文关闭前执行，与生命周期状态机或 shutdown() 成员函数没有直接关系
   */
  void registerRclPreshutdownCallback();

  /**
   * @brief Check whether all managed nodes are active | 检查所有受管理节点是否处于激活状态
   * @return true if all managed nodes are active, false otherwise |
   * 所有受管理节点均处于激活状态时返回 true，否则返回 false
   */
  bool isActive();

  // Timer thread to look at bond connections
  rclcpp::TimerBase::SharedPtr init_timer_;          // 延迟执行初始化和自动启动的一次性定时器
  rclcpp::TimerBase::SharedPtr bond_timer_;          // 定期检查受管理节点 Bond 心跳连接的定时器
  rclcpp::TimerBase::SharedPtr bond_respawn_timer_;  // 节点异常退出后，定期检查其是否重新启动的定时器
  std::chrono::milliseconds bond_timeout_;           // Bond 心跳连接的超时时间，单位为毫秒

  // A map of all nodes used to check Bond connections | 用于检查所有节点 Bond 连接的映射
  std::map<std::string, std::shared_ptr<bond::Bond>> bond_map_;  // 节点名称到 Bond 连接对象的映射

  // A map of all nodes to be controlled | 所有受管理节点的映射
  std::map<std::string, std::shared_ptr<nav2_util::LifecycleServiceClient>> node_map_;  // 节点名称到生命周期服务客户端的映射

  std::map<std::uint8_t, std::string> transition_label_map_;    // 保存每种生命周期转换对应的日志文本，用于打印当前正在执行的状态转换

  // A map of the expected transitions to primary states | 生命周期转换编号到预期主状态的映射。保存每种生命周期转换完成后，节点应该到达的目标主状态
  std::unordered_map<std::uint8_t, std::uint8_t> transition_state_map_;

  // The names of the nodes to be managed, in the order of desired bring-up | 需要管理的节点名称，按照期望的启动顺序排列
  std::vector<std::string> node_names_;

  // Whether to automatically start up the system | 是否自动启动系统
  bool autostart_;
  bool attempt_respawn_reconnection_;     // 节点异常退出后，是否尝试等待其重新启动并恢复 Bond 连接

  NodeState managed_nodes_state_{NodeState::UNCONFIGURED};   // 当前所有受管理节点的整体生命周期状态，初始为“未配置”
  diagnostic_updater::Updater diagnostics_updater_;          // ROS 2 诊断信息更新器，用于发布生命周期管理器的健康状态

  rclcpp::Time bond_respawn_start_time_{0};           // 开始等待异常节点重新启动的时间点，0 表示尚未开始等待
  rclcpp::Duration bond_respawn_max_duration_{10s};   // 等待异常节点重新启动并恢复 Bond 连接的最长时间，默认 10 秒
};

}  // namespace nav2_lifecycle_manager

#endif  // NAV2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_
