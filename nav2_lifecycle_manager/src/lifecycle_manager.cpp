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

#include "nav2_lifecycle_manager/lifecycle_manager.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

using lifecycle_msgs::msg::Transition;
using lifecycle_msgs::msg::State;
using nav2_util::LifecycleServiceClient;

namespace nav2_lifecycle_manager
{

LifecycleManager::LifecycleManager(const rclcpp::NodeOptions & options)
: Node("lifecycle_manager", options), diagnostics_updater_(this)
{
  RCLCPP_INFO(get_logger(), "\033[34m Creating. LifecycleManager 构造函数 初始化开始 \033[0m");

  // The list of names is parameterized, allowing this module to be used with a different set of nodes
  // 名称列表已参数化，使该模块可以与不同的一组节点配合使用
  declare_parameter("node_names", rclcpp::PARAMETER_STRING_ARRAY);  // 只指定类型，没有默认内容 // rclcpp::PARAMETER_STRING_ARRAY 表示 ROS 2 参数类型中的“字符串数组”。
  declare_parameter("autostart", rclcpp::ParameterValue(false));    // 表示创建一个 ROS 2 参数值，内容是布尔值 false
  declare_parameter("bond_timeout", 4.0);
  declare_parameter("bond_respawn_max_duration", 10.0);
  declare_parameter("attempt_respawn_reconnection", true);

  registerRclPreshutdownCallback();

  node_names_ = get_parameter("node_names").as_string_array();
  get_parameter("autostart", autostart_);
  double bond_timeout_s;
  get_parameter("bond_timeout", bond_timeout_s);
  // 把以“秒”为单位的 bond_timeout_s 转换成以“毫秒”为单位的 bond_timeout_
  bond_timeout_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(bond_timeout_s));

  // ---------------打印日志------------------------
  // 将节点名称数组转换成便于打印的字符串
  std::string node_names_text;
  for (size_t i = 0; i < node_names_.size(); ++i) {
    node_names_text += node_names_[i];
    if (i + 1 < node_names_.size()) {
      node_names_text += ", ";
    }
  }
  RCLCPP_INFO(get_logger(), "生命周期管理器参数：node_names=[%s]，autostart=%s，bond_timeout=%.2f 秒", 
              node_names_text.c_str(), autostart_ ? "true" : "false", bond_timeout_s);
  // ---------------打印日志------------------------

  double respawn_timeout_s;
  get_parameter("bond_respawn_max_duration", respawn_timeout_s);
  bond_respawn_max_duration_ = rclcpp::Duration::from_seconds(respawn_timeout_s);

  get_parameter("attempt_respawn_reconnection", attempt_respawn_reconnection_);

  RCLCPP_INFO(get_logger(),"当前节点名称(get_name): %s, 完整节点名称：%s", get_name(), get_fully_qualified_name());
  // 当前节点名称(get_name): lifecycle_manager_navfn_example, 完整节点名称：/lifecycle_manager_navfn_example


  // 创建互斥回调组，并禁止自动加入节点执行器，稍后由独立线程中的执行器手动管理，
  // MutuallyExclusive：组内回调互斥执行，表示属于该回调组的回调不能同时执行。一个回调尚未结束时，同组的其他回调必须等待。
  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  manager_srv_ = create_service<ManageLifecycleNodes>(
    get_name() + std::string("/manage_nodes"),
    std::bind(&LifecycleManager::managerCallback, this, _1, _2, _3),
    rclcpp::SystemDefaultsQoS(),
    callback_group_);

  is_active_srv_ = create_service<std_srvs::srv::Trigger>(
    get_name() + std::string("/is_active"),
    std::bind(&LifecycleManager::isActiveCallback, this, _1, _2, _3),
    rclcpp::SystemDefaultsQoS(),
    callback_group_);

  transition_state_map_[Transition::TRANSITION_CONFIGURE] = State::PRIMARY_STATE_INACTIVE;
  transition_state_map_[Transition::TRANSITION_CLEANUP] = State::PRIMARY_STATE_UNCONFIGURED;
  transition_state_map_[Transition::TRANSITION_ACTIVATE] = State::PRIMARY_STATE_ACTIVE;
  transition_state_map_[Transition::TRANSITION_DEACTIVATE] = State::PRIMARY_STATE_INACTIVE;
  transition_state_map_[Transition::TRANSITION_UNCONFIGURED_SHUTDOWN] = State::PRIMARY_STATE_FINALIZED;

  transition_label_map_[Transition::TRANSITION_CONFIGURE] = std::string("Configuring ");
  transition_label_map_[Transition::TRANSITION_CLEANUP] = std::string("Cleaning up ");
  transition_label_map_[Transition::TRANSITION_ACTIVATE] = std::string("Activating ");
  transition_label_map_[Transition::TRANSITION_DEACTIVATE] = std::string("Deactivating ");
  transition_label_map_[Transition::TRANSITION_UNCONFIGURED_SHUTDOWN] = std::string("Shutting down ");

  // 创建立即触发的初始化定时器，使初始化操作在构造函数结束后执行。
  init_timer_ = this->create_wall_timer(
    0s,
    [this]() -> void {
      init_timer_->cancel();              // 创建立即触发的初始化定时器，使初始化操作在构造函数结束后执行。
      createLifecycleServiceClients();    // 为所有受管理的生命周期节点创建状态查询和转换服务客户端。// Note: 这里创建需要启动的节点
      if (autostart_) {                   // 如果启用了自动启动，则创建第二个一次性定时器。
        init_timer_ = this->create_wall_timer(
          0s,
          [this]() -> void {
            init_timer_->cancel();        // 取消第二个定时器，保证 startup() 只执行一次。
            startup();                    // 配置并激活所有受管理节点。
          },
          callback_group_);
      }
      // 创建专门处理生命周期服务和定时器回调的单线程执行器。
      auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      executor->add_callback_group(callback_group_, get_node_base_interface());   // 手动将互斥回调组添加到专用执行器。
      service_thread_ = std::make_unique<nav2_util::NodeThread>(executor);        // 在独立线程中运行执行器
    });
  diagnostics_updater_.setHardwareID("Nav2");
  diagnostics_updater_.add("Nav2 Health", this, &LifecycleManager::CreateDiagnostic);
  RCLCPP_INFO(get_logger(), "\033[34m ending. LifecycleManager 构造函数 初始化结束 \033[0m");
}

LifecycleManager::~LifecycleManager()
{
  RCLCPP_INFO(get_logger(), "Destroying %s", get_name());
  service_thread_.reset();
}

void
LifecycleManager::managerCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<ManageLifecycleNodes::Request> request,
  std::shared_ptr<ManageLifecycleNodes::Response> response)
{
  switch (request->command) {
    case ManageLifecycleNodes::Request::STARTUP:    // 启动导航系统：依次配置并激活所有受管理节点。
      response->success = startup();
      break;
    case ManageLifecycleNodes::Request::CONFIGURE:  // 配置所有受管理节点，使其从 unconfigured 转换为 inactive。
      response->success = configure();
      break;
    case ManageLifecycleNodes::Request::CLEANUP:    // 清理所有受管理节点，使其从 inactive 转换为 unconfigured。
      response->success = cleanup();
      break;
    case ManageLifecycleNodes::Request::RESET:      // 重置所有受管理节点，使导航系统恢复到初始的未配置状态。
      response->success = reset();
      break;
    case ManageLifecycleNodes::Request::SHUTDOWN:   // 按照管理顺序关闭所有受管理节点。
      response->success = shutdown();
      break;
    case ManageLifecycleNodes::Request::PAUSE:      // 暂停导航系统：停用所有受管理节点，使其转换为 inactive。
      response->success = pause();
      break;
    case ManageLifecycleNodes::Request::RESUME:     // 恢复导航系统：重新激活暂停的受管理节点。
      response->success = resume();
      break;
  }
}

inline bool
LifecycleManager::isActive()
{
  return managed_nodes_state_ == NodeState::ACTIVE;
}

void
LifecycleManager::isActiveCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  response->success = isActive();
}

void
LifecycleManager::CreateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  unsigned char error_level;
  std::string message;
  switch (managed_nodes_state_) {
    case NodeState::ACTIVE:
      error_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      message = "Managed nodes are active";
      break;
    case NodeState::INACTIVE:
      error_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      message = "Managed nodes are inactive";
      break;
    case NodeState::UNCONFIGURED:
      error_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      message = "Managed nodes are unconfigured";
      break;
    case NodeState::FINALIZED:
      error_level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      message = "Managed nodes have been shut down";
      break;
    default:  // NodeState::UNKNOWN
      error_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      message = "An error has occurred during a node state transition";
      break;
  }
  stat.summary(error_level, message);
}

void
LifecycleManager::createLifecycleServiceClients()
{
  message("Creating and initializing lifecycle service clients | 正在创建并初始化生命周期服务客户端");
  for (auto & node_name : node_names_) {
    node_map_[node_name] =
      std::make_shared<LifecycleServiceClient>(node_name, shared_from_this());
  }
  message("Create and initialize lifecycle service clients end | 创建并初始化生命周期服务客户端---结束--");
}

void
LifecycleManager::destroyLifecycleServiceClients()
{
  message("Destroying lifecycle service clients");
  for (auto & kv : node_map_) {
    kv.second.reset();
  }
}

bool
LifecycleManager::createBondConnection(const std::string & node_name)
{
  // step: 1 转换超时时间
  const double timeout_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(bond_timeout_).count();
  const double timeout_s = timeout_ns / 1e9;

  // step: 2 判断是否需要创建
  // 必须同时满足：bond_map_ 中还没有这个节点的 Bond；配置的 bond_timeout 大于 0。
  if (bond_map_.find(node_name) == bond_map_.end() && bond_timeout_.count() > 0.0) {
    // 创建 Manager 侧的 Bond
    bond_map_[node_name] = std::make_shared<bond::Bond>("bond", node_name, shared_from_this());
    bond_map_[node_name]->setHeartbeatTimeout(timeout_s);
    bond_map_[node_name]->setHeartbeatPeriod(0.10);
    bond_map_[node_name]->start();
    // 等待双方建立连接
    if (!bond_map_[node_name]->waitUntilFormed(rclcpp::Duration(rclcpp::Duration::from_nanoseconds(timeout_ns / 2))))
    {
      RCLCPP_ERROR(
        get_logger(),
        "Server %s was unable to be reached after %0.2fs by bond. "
        "This server may be misconfigured.",
        node_name.c_str(), timeout_s);
      return false;
    }
    RCLCPP_INFO(get_logger(), "\033[33m Server %s connected with bond.\033[0m", node_name.c_str());
  }

  return true;
}

bool
LifecycleManager::changeStateForNode(const std::string & node_name, std::uint8_t transition)
{
  message(transition_label_map_[transition] + node_name);

  // 状态转换请求是否成功，并且节点是否真正进入了预期状态。
  if (!node_map_[node_name]->change_state(transition) ||
    !(node_map_[node_name]->get_state() == transition_state_map_[transition]))
  {
    RCLCPP_ERROR(get_logger(), "Failed to change state for node: %s", node_name.c_str());
    return false;
  }

  if (transition == Transition::TRANSITION_ACTIVATE) {    // 节点激活后，创建 Bond 心跳连接，用于监控节点是否存活
    return createBondConnection(node_name);
  } else if (transition == Transition::TRANSITION_DEACTIVATE) {   // 节点停用后，删除对应的 Bond 心跳连接
    bond_map_.erase(node_name);
  }

  return true;
}

bool
LifecycleManager::changeStateForAllNodes(std::uint8_t transition, bool hard_change)
{
  // Hard change will continue even if a node fails
  // 判断这次状态切换是不是“配置节点” 或 “激活节点”
  // 配置和激活：正序执行
  if (transition == Transition::TRANSITION_CONFIGURE ||
    transition == Transition::TRANSITION_ACTIVATE)
  {
    for (auto & node_name : node_names_) {
      try {
        if (!changeStateForNode(node_name, transition) && !hard_change) {
          return false;
        }
      } catch (const std::runtime_error & e) {
        RCLCPP_ERROR(
          get_logger(),
          "Failed to change state for node: %s. Exception: %s.", node_name.c_str(), e.what());
        return false;
      }
    }
  } else {    // 其他转换：逆序执行. 例如：DEACTIVATE：停用, CLEANUP：清理, SHUTDOWN：关闭
    std::vector<std::string>::reverse_iterator rit;
    for (rit = node_names_.rbegin(); rit != node_names_.rend(); ++rit) {    // rbegin() 和 rend() 用来对容器进行反向遍历。
      try {
        if (!changeStateForNode(*rit, transition) && !hard_change) {
          return false;
        }
      } catch (const std::runtime_error & e) {
        RCLCPP_ERROR(
          get_logger(),
          "Failed to change state for node: %s. Exception: %s.", (*rit).c_str(), e.what());
        return false;
      }
    }
  }
  return true;
}

void
LifecycleManager::shutdownAllNodes()
{
  message("Deactivate, cleanup, and shutdown nodes");
  managed_nodes_state_ = NodeState::FINALIZED;
  changeStateForAllNodes(Transition::TRANSITION_DEACTIVATE);
  changeStateForAllNodes(Transition::TRANSITION_CLEANUP);
  changeStateForAllNodes(Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
}

bool
LifecycleManager::startup()
{
  message("Starting managed nodes bringup... 开始启动受管理节点...");
  if (!changeStateForAllNodes(Transition::TRANSITION_CONFIGURE) ||
    !changeStateForAllNodes(Transition::TRANSITION_ACTIVATE))
  {
    RCLCPP_ERROR(get_logger(), "Failed to bring up all requested nodes. Aborting bringup.");
    managed_nodes_state_ = NodeState::UNKNOWN;
    return false;
  }
  message("Managed nodes are active | 受管理节点已激活");
  managed_nodes_state_ = NodeState::ACTIVE;
  createBondTimer();
  return true;
}

bool
LifecycleManager::configure()
{
  message("Configuring managed nodes...");
  if (!changeStateForAllNodes(Transition::TRANSITION_CONFIGURE)) {
    RCLCPP_ERROR(get_logger(), "Failed to configure all requested nodes. Aborting bringup.");
    managed_nodes_state_ = NodeState::UNKNOWN;
    return false;
  }
  message("Managed nodes are now configured");
  managed_nodes_state_ = NodeState::INACTIVE;
  return true;
}

bool
LifecycleManager::cleanup()
{
  message("Cleaning up managed nodes...");
  if (!changeStateForAllNodes(Transition::TRANSITION_CLEANUP)) {
    RCLCPP_ERROR(get_logger(), "Failed to cleanup all requested nodes. Aborting cleanup.");
    managed_nodes_state_ = NodeState::UNKNOWN;
    return false;
  }
  message("Managed nodes have been cleaned up");
  managed_nodes_state_ = NodeState::UNCONFIGURED;
  return true;
}

bool
LifecycleManager::shutdown()
{
  destroyBondTimer();

  message("Shutting down managed nodes...");
  shutdownAllNodes();
  destroyLifecycleServiceClients();
  message("Managed nodes have been shut down");
  return true;
}

bool
LifecycleManager::reset(bool hard_reset)
{
  destroyBondTimer();

  message("Resetting managed nodes...");
  // Should transition in reverse order
  if (!changeStateForAllNodes(Transition::TRANSITION_DEACTIVATE, hard_reset) ||
    !changeStateForAllNodes(Transition::TRANSITION_CLEANUP, hard_reset))
  {
    if (!hard_reset) {
      RCLCPP_ERROR(get_logger(), "Failed to reset nodes: aborting reset");
      managed_nodes_state_ = NodeState::UNKNOWN;
      return false;
    }
  }

  message("Managed nodes have been reset");
  managed_nodes_state_ = NodeState::UNCONFIGURED;
  return true;
}

bool
LifecycleManager::pause()
{
  destroyBondTimer();

  message("Pausing managed nodes...");
  if (!changeStateForAllNodes(Transition::TRANSITION_DEACTIVATE)) {
    RCLCPP_ERROR(get_logger(), "Failed to pause nodes: aborting pause");
    managed_nodes_state_ = NodeState::UNKNOWN;
    return false;
  }

  message("Managed nodes have been paused");
  managed_nodes_state_ = NodeState::INACTIVE;
  return true;
}

bool
LifecycleManager::resume()
{
  message("Resuming managed nodes...");
  if (!changeStateForAllNodes(Transition::TRANSITION_ACTIVATE)) {
    RCLCPP_ERROR(get_logger(), "Failed to resume nodes: aborting resume");
    managed_nodes_state_ = NodeState::UNKNOWN;
    return false;
  }

  message("Managed nodes are active");
  managed_nodes_state_ = NodeState::ACTIVE;
  createBondTimer();
  return true;
}

void
LifecycleManager::createBondTimer()
{
  if (bond_timeout_.count() <= 0) {
    return;
  }

  message("Creating bond timer... | 正在创建 bond 定时器...");
  bond_timer_ = this->create_wall_timer(
    200ms,
    std::bind(&LifecycleManager::checkBondConnections, this),
    callback_group_);
}

void
LifecycleManager::destroyBondTimer()
{
  if (bond_timer_) {
    message("Terminating bond timer... | 正在终止 bond 定时器...");
    bond_timer_->cancel();    // 停止定时器运行。
    bond_timer_.reset();      // 释放并清空定时器对象。
  }
}

void
LifecycleManager::onRclPreshutdown()
{
  RCLCPP_INFO(
    get_logger(), "Running Nav2 LifecycleManager rcl preshutdown (%s)",
    this->get_name());

  destroyBondTimer();

  /*
   * Dropping the bond map is what we really need here, but we drop the others
   * to prevent the bond map being used. Likewise, squash the service thread.
   */
  service_thread_.reset();
  node_names_.clear();
  node_map_.clear();
  bond_map_.clear();
}

void
LifecycleManager::registerRclPreshutdownCallback()
{
  rclcpp::Context::SharedPtr context = get_node_base_interface()->get_context();

  context->add_pre_shutdown_callback(
    std::bind(&LifecycleManager::onRclPreshutdown, this)
  );
}

void
LifecycleManager::checkBondConnections()
{
  if (!isActive() || !rclcpp::ok() || bond_map_.empty()) {
    return;
  }

  for (auto & node_name : node_names_) {
    if (!rclcpp::ok()) {
      return;
    }

    if (bond_map_[node_name]->isBroken()) {
      message(
        std::string(
          "Have not received a heartbeat from " + node_name + "."));

      // if one is down, bring them all down
      RCLCPP_ERROR(
        get_logger(),
        "CRITICAL FAILURE: SERVER %s IS DOWN after not receiving a heartbeat for %i ms."
        " Shutting down related nodes.",
        node_name.c_str(), static_cast<int>(bond_timeout_.count()));
      reset(true);  // hard reset to transition all still active down
      // if a server crashed, it won't get cleared due to failed transition, clear manually
      bond_map_.clear();

      // Initialize the bond respawn timer to check if server comes back online
      // after a failure, within a maximum timeout period.
      if (attempt_respawn_reconnection_) {
        bond_respawn_timer_ = this->create_wall_timer(
          1s,
          std::bind(&LifecycleManager::checkBondRespawnConnection, this),
          callback_group_);
      }
      return;
    }
  }
}

void
LifecycleManager::checkBondRespawnConnection()
{
  // First attempt in respawn, start maximum duration to respawn
  if (bond_respawn_start_time_.nanoseconds() == 0) {
    bond_respawn_start_time_ = now();
  }

  // Note: isActive() is inverted since this should be in a failure
  // condition. If another outside user actives the system again, this should not process.
  if (isActive() || !rclcpp::ok() || node_names_.empty()) {
    bond_respawn_start_time_ = rclcpp::Time(0);
    bond_respawn_timer_.reset();
    return;
  }

  // Check number of live connections after a bond failure
  int live_servers = 0;
  const int max_live_servers = node_names_.size();
  for (auto & node_name : node_names_) {
    if (!rclcpp::ok()) {
      return;
    }

    try {
      node_map_[node_name]->get_state();  // Only won't throw if the server exists
      live_servers++;
    } catch (...) {
      break;
    }
  }

  // If all are alive, kill timer and retransition system to active
  // Else, check if maximum timeout has occurred
  if (live_servers == max_live_servers) {
    message("Successfully re-established connections from server respawns, starting back up.");
    bond_respawn_start_time_ = rclcpp::Time(0);
    bond_respawn_timer_.reset();
    startup();
  } else if (now() - bond_respawn_start_time_ >= bond_respawn_max_duration_) {
    message("Failed to re-establish connection from a server crash after maximum timeout.");
    bond_respawn_start_time_ = rclcpp::Time(0);
    bond_respawn_timer_.reset();
  }
}

#define ANSI_COLOR_RESET    "\x1b[0m"
#define ANSI_COLOR_BLUE     "\x1b[34m"

void
LifecycleManager::message(const std::string & msg)
{
  RCLCPP_INFO(get_logger(), ANSI_COLOR_BLUE "\33[1m%s\33[0m" ANSI_COLOR_RESET, msg.c_str());
}

}  // namespace nav2_lifecycle_manager

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_lifecycle_manager::LifecycleManager)
