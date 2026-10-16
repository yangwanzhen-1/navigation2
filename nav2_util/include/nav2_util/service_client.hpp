// Copyright (c) 2018 Intel Corporation
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

#ifndef NAV2_UTIL__SERVICE_CLIENT_HPP_
#define NAV2_UTIL__SERVICE_CLIENT_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"

namespace nav2_util
{

/**
 * @class nav2_util::ServiceClient
 * @brief A simple wrapper on ROS2 services for invoke() and block-style calling |
 * ROS 2 服务客户端的简单封装，提供 invoke() 和同步阻塞式调用功能
 */
template<class ServiceT, typename NodeT = rclcpp::Node::SharedPtr>
class ServiceClient
{
public:
  /**
  * @brief A constructor | 构造函数
  * @param service_name name of the service to call | 需要调用的服务名称
  * @param provided_node Node to create the service client off of | 用于创建服务客户端的 ROS 2 节点
  */
  explicit ServiceClient(
    const std::string & service_name,
    const NodeT & provided_node)
  : service_name_(service_name), node_(provided_node)
  {
    // 创建互斥回调组，并禁止将其自动加入节点的执行器。
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);
    
    // 手动将回调组添加到该服务客户端使用的专用执行器。
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

    // 使用指定的服务类型、服务名称、默认 QoS 和回调组 创建 ROS 2 服务客户端。
    client_ = node_->template create_client<ServiceT>(
      service_name,
      rclcpp::SystemDefaultsQoS(),
      callback_group_);
  }

  using RequestType = typename ServiceT::Request;
  using ResponseType = typename ServiceT::Response;

  /**
  * @brief Invoke the service and block until completed or timed out | 调用服务并阻塞，直到请求完成或等待超时
  * @param request The request object to call the service using | 用于调用服务的请求对象
  * @param timeout Maximum timeout to wait for, default infinite | 等待响应的最长时间，负值表示无限等待
  * @return Response A pointer to the service response from the request| 指向服务响应的共享指针
  */
  typename ResponseType::SharedPtr invoke(
    typename RequestType::SharedPtr & request,
    const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1))
  {
    // step: 1 等待服务出现
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      // 检查 ROS 2 是否仍在运行
      if (!rclcpp::ok()) {
        throw std::runtime_error(
                service_name_ + " service client: interrupted while waiting for service");
      }
      RCLCPP_INFO(
        node_->get_logger(), "%s service client: waiting for service to appear...",
        service_name_.c_str());
    }

    RCLCPP_DEBUG(
      node_->get_logger(), "%s service client: send async request",
      service_name_.c_str());
    
    // step: 2 异步发送服务请求
    auto future_result = client_->async_send_request(request);

    // step: 3 阻塞等待响应
    if (callback_group_executor_.spin_until_future_complete(future_result, timeout) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      // Pending request must be manually cleaned up if execution is interrupted or timed out
      // 清理未完成请求, 如果等待超时或被中断，需要手动从客户端内部删除这个未完成请求。
      client_->remove_pending_request(future_result);
      throw std::runtime_error(service_name_ + " service client: async_send_request failed");
    }

    return future_result.get();
  }

  /**
  * @brief Invoke the service and block until completed | 调用服务并阻塞，直到请求处理完成
  * @param request The request object to call the service using | 用于调用服务的请求对象
  * @param Response A pointer to the service response from the request | 用于接收服务响应的共享指针
  * @return bool Whether it was successfully called | 服务调用成功完成返回 true，否则返回 false
  */
  bool invoke(
    typename RequestType::SharedPtr & request,
    typename ResponseType::SharedPtr & response)
  {
    // step: 1 等待服务出现
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      // 检查 ROS 2 是否仍在运行
      if (!rclcpp::ok()) {
        throw std::runtime_error(
                service_name_ + " service client: interrupted while waiting for service");
      }
      RCLCPP_INFO(
        node_->get_logger(), "%s service client: waiting for service to appear...",
        service_name_.c_str());
    }

    RCLCPP_DEBUG(
      node_->get_logger(), "%s service client: send async request",
      service_name_.c_str());

    // step: 2 异步发送服务请求
    auto future_result = client_->async_send_request(request);

    // step: 3 阻塞等待响应
    if (callback_group_executor_.spin_until_future_complete(future_result) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      // Pending request must be manually cleaned up if execution is interrupted or timed out
      // 清理未完成请求, 如果等待超时或被中断，需要手动从客户端内部删除这个未完成请求。
      client_->remove_pending_request(future_result);
      return false;
    }

    response = future_result.get();
    return response.get();
  }

  /**
  * @brief Block until a service is available or timeout |  阻塞等待服务可用，或者等待时间超过指定的超时时间
  * @param timeout Maximum timeout to wait for, default infinite | 最长等待时间，默认值表示无限等待
  * @return bool true if service is available | 服务可用时返回 true，等待超时或服务不可用时返回 false
  */
  bool wait_for_service(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds::max())
  {
    return client_->wait_for_service(timeout);
  }

  /**
  * @brief Gets the service name | 获取服务名称
  * @return string Service name | 返回服务名称字符串
  */
  std::string getServiceName()
  {
    return service_name_;
  }

  /**
  * @brief Stop any running spin operations on the internal executor  | 停止内部执行器正在运行的 spin 操作
  */
  void stop()
  {
    if (client_) {
      callback_group_executor_.cancel();
    }
  }

protected:
  std::string service_name_;     // 服务名称：保存需要调用的 ROS 2 服务名称
  NodeT node_;                   // 节点民初：保存用于创建和运行服务客户端的 ROS 2 节点
  rclcpp::CallbackGroup::SharedPtr callback_group_;       // 服务响应回调使用的互斥回调组
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_; // 专门处理该服务客户端回调的单线程执行器
  typename rclcpp::Client<ServiceT>::SharedPtr client_;   // 底层 ROS 2 服务客户端的共享指针
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__SERVICE_CLIENT_HPP_
