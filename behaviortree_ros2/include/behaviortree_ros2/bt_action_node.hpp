// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2023 Davide Faconti
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

#pragma once

#include <memory>
#include <string>
#include <optional>
#include <rclcpp/executors.hpp>
#include "behaviortree_cpp/action_node.h"
#include "rclcpp_action/rclcpp_action.hpp"

#include "behaviortree_ros2/ros_node_params.hpp"

namespace BT
{

enum ActionNodeErrorCode
{
  SERVER_UNREACHABLE,
  SEND_GOAL_TIMEOUT,
  GOAL_REJECTED_BY_SERVER,
  ACTION_ABORTED,
  ACTION_CANCELLED,
  INVALID_GOAL
};

inline const char* toStr(const ActionNodeErrorCode& err)
{
  switch(err)
  {
    case SERVER_UNREACHABLE:
      return "SERVER_UNREACHABLE";
    case SEND_GOAL_TIMEOUT:
      return "SEND_GOAL_TIMEOUT";
    case GOAL_REJECTED_BY_SERVER:
      return "GOAL_REJECTED_BY_SERVER";
    case ACTION_ABORTED:
      return "ACTION_ABORTED";
    case ACTION_CANCELLED:
      return "ACTION_CANCELLED";
    case INVALID_GOAL:
      return "INVALID_GOAL";
  }
  return nullptr;
}

/// Adapt a ROS action client to a behavior-tree leaf.
/// Serialize tick, halt, destruction, and dispatch through shared clients.
/// Bounded cancellation does not guarantee that an outstanding goal has stopped.
/// The action_name input overrides BT::RosNodeParams::default_port_value when nonempty.
/// Derived leaves supply goals and interpret results through setGoal() and onResultReceived().
template <class ActionT>
class RosActionNode : public BT::ActionNodeBase
{
public:
  using ActionType = ActionT;
  using ActionClient = typename rclcpp_action::Client<ActionT>;
  using ActionClientPtr = std::shared_ptr<ActionClient>;
  using Goal = typename ActionT::Goal;
  using GoalHandle = typename rclcpp_action::ClientGoalHandle<ActionT>;
  using WrappedResult = typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult;
  using Feedback = typename ActionT::Feedback;

  /// Construct a client using explicit discovery and response budgets.
  /// @throw BT::LogicError if called during callback dispatch for the same action type.
  /// @throw BT::RuntimeError if the action name is empty or the ROS node has expired.
  /// @throw rclcpp::exceptions::RCLError if client creation or discovery fails.
  explicit RosActionNode(const std::string& instance_name, const BT::NodeConfig& conf,
                         const BT::RosNodeParams& params);

  ~RosActionNode() override;

  // Callback bindings require a stable leaf address.
  RosActionNode(const RosActionNode&) = delete;
  RosActionNode& operator=(const RosActionNode&) = delete;
  RosActionNode(RosActionNode&&) = delete;
  RosActionNode& operator=(RosActionNode&&) = delete;

  /// Merge the action_name input with a leaf's ports.
  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = { BT::InputPort<std::string>("action_name", "",
                                                       "Action server name") };
    basic.insert(addition.begin(), addition.end());
    return basic;
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  /// Run once when a running invocation is halted, after bounded cancellation.
  virtual void onHalt()
  {}

  /// Populate a goal; false routes INVALID_GOAL to onFailure without sending.
  virtual bool setGoal(Goal& goal) = 0;

  /// Interpret a terminal result; return SUCCESS or FAILURE.
  virtual BT::NodeStatus onResultReceived(const WrappedResult& result) = 0;

  /// Return RUNNING to continue, or SUCCESS/FAILURE to cancel and complete.
  /// Returning IDLE throws std::logic_error during callback dispatch.
  /// Lifecycle changes on this action type are rejected during dispatch; request completion by return status.
  virtual BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> /*feedback*/)
  {
    return BT::NodeStatus::RUNNING;
  }

  /// Interpret a failure and its result when available; return SUCCESS or FAILURE.
  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error,
                                   const std::optional<WrappedResult>&)
  {
    return onFailure(error);
  }

  /// Handle failures without requiring a result-aware override.
  virtual BT::NodeStatus onFailure(ActionNodeErrorCode /*error*/)
  {
    return BT::NodeStatus::FAILURE;
  }

  /// Attempt bounded cancellation; tick() still delivers a completed result.
  /// Transport failures leave cleanup unconfirmed and available for another attempt.
  /// @throw BT::LogicError if called during callback dispatch for the same action type.
  void cancelGoal();

  /// Cancel retained requests, including after FAILURE, and reset BT status.
  /// An unconfirmed request remains available for another cleanup attempt.
  /// Transport failures are logged rather than propagated through BT::Tree destruction.
  /// @throw BT::LogicError if called during callback dispatch for the same action type.
  void halt() override;

  /// @throw BT::LogicError if called during callback dispatch for the same action type,
  /// or a completion hook does not return SUCCESS or FAILURE.
  /// @throw BT::RuntimeError if no usable action name or ROS node is configured.
  /// @throw std::logic_error if feedback returns IDLE.
  /// @throw rclcpp::exceptions::RCLError if transport fails.
  /// @throw rclcpp_action::exceptions::UnawareGoalHandleError if result retrieval fails.
  BT::NodeStatus tick() override;

  /// Report retained request state; after halt(), true means cleanup is unconfirmed.
  /// Starting a new invocation abandons the old request after one bounded cleanup attempt.
  bool hasUnresolvedGoal() const;

  /// Change the endpoint for future invocations; retained cleanup uses its original client.
  /// @throw BT::LogicError if called during callback dispatch for the same action type.
  /// @throw BT::RuntimeError if the action name is empty or the ROS node has expired.
  /// @throw rclcpp::exceptions::RCLError if client creation or discovery fails.
  void setActionName(const std::string& action_name);

protected:
  struct ActionClientInstance
  {
    ActionClientInstance(std::shared_ptr<rclcpp::Node> node,
                         const std::string& action_name);

    virtual ~ActionClientInstance() = default;

    /// Keep result transport overridable independently of real ROS cancellation.
    /// @throw rclcpp::exceptions::RCLError if transport fails.
    /// @throw rclcpp_action::exceptions::UnknownGoalHandleError if the client no longer tracks the goal.
    /// @throw rclcpp_action::exceptions::UnawareGoalHandleError if result retrieval fails.
    /// @throw std::exception_ptr if the native Jazzy client throws an invalidated goal's stored error.
    virtual std::shared_future<WrappedResult>
    requestResult(typename GoalHandle::SharedPtr handle,
                  typename ActionClient::ResultCallback callback)
    {
      return action_client->async_get_result(std::move(handle), std::move(callback));
    }

    ActionClientPtr action_client;
    rclcpp::CallbackGroup::SharedPtr callback_group;
    rclcpp::executors::SingleThreadedExecutor callback_executor;
  };

  static std::mutex& getMutex()
  {
    static std::mutex action_client_mutex;
    return action_client_mutex;
  }

  rclcpp::Logger logger()
  {
    if(auto node = node_.lock())
    {
      return node->get_logger();
    }
    return rclcpp::get_logger("RosActionNode");
  }

  rclcpp::Time now()
  {
    if(auto node = node_.lock())
    {
      return node->now();
    }
    return rclcpp::Clock(RCL_ROS_TIME).now();
  }

  using ClientsRegistry =
      std::unordered_map<std::string, std::weak_ptr<ActionClientInstance>>;

  static ClientsRegistry& getRegistry()
  {
    static ClientsRegistry action_clients_registry;
    return action_clients_registry;
  }

  std::weak_ptr<rclcpp::Node> node_;
  std::shared_ptr<ActionClientInstance> client_instance_;
  std::string action_name_;
  bool action_name_should_be_checked_ = false;
  const std::chrono::milliseconds server_timeout_;
  const std::chrono::milliseconds wait_for_server_timeout_;
  std::string action_client_key_;

private:
  class DispatchGuard
  {
  public:
    DispatchGuard()
    {
      dispatchActive() = true;
    }

    ~DispatchGuard()
    {
      dispatchActive() = false;
    }

    DispatchGuard(const DispatchGuard&) = delete;
    DispatchGuard& operator=(const DispatchGuard&) = delete;
    DispatchGuard(DispatchGuard&&) = delete;
    DispatchGuard& operator=(DispatchGuard&&) = delete;
  };

  struct Request
  {
    std::shared_ptr<ActionClientInstance> client;
    std::shared_future<typename GoalHandle::SharedPtr> acceptance;
    typename GoalHandle::SharedPtr goal_handle;
    std::shared_future<WrappedResult> result;
    std::exception_ptr result_error;
    rclcpp::Time sent_at;
    BT::NodeStatus feedback_status = BT::NodeStatus::RUNNING;
    // Valid only during serialized dispatch for the live invocation.
    RosActionNode* owner = nullptr;
  };

  // Match the action-type mutex without rejecting dispatch on a different thread as re-entry.
  static bool& dispatchActive()
  {
    static thread_local bool active = false;
    return active;
  }

  /// Reject re-entry before any lifecycle mutation or acquisition of the action-type mutex.
  /// @throw BT::LogicError if the current thread is dispatching this action type.
  static void throwIfDispatching();

  bool createClient(const std::string& action_name);

  /// Normalize Jazzy clients that throw an exception_ptr instead of its contained exception.
  /// @pre An exception is currently being handled.
  static std::exception_ptr currentTransportError() noexcept;

  /// Preserve result failures without interrupting executor dispatch or cancellation.
  static void requestResult(const std::shared_ptr<Request>& request) noexcept;

  /// Only a successfully retrieved result establishes completion.
  static std::optional<WrappedResult> resultIfReady(Request& request) noexcept;

  void logCleanupError(const char* operation, std::exception_ptr error) noexcept;

  /// Return true only after rejection or a terminal result is observed.
  /// Preserve unconfirmed ownership when any transport operation fails.
  bool releaseRequest(std::shared_ptr<Request> request) noexcept;

  std::shared_ptr<Request> request_;
  const std::string default_action_name_;
};

template <class T>
RosActionNode<T>::ActionClientInstance::ActionClientInstance(
    std::shared_ptr<rclcpp::Node> node, const std::string& action_name)
{
  callback_group =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_executor.add_callback_group(callback_group, node->get_node_base_interface());
  action_client = rclcpp_action::create_client<T>(node, action_name, callback_group);
}

template <class T>
inline RosActionNode<T>::RosActionNode(const std::string& instance_name,
                                       const BT::NodeConfig& conf,
                                       const BT::RosNodeParams& params)
  : BT::ActionNodeBase(instance_name, conf)
  , node_(params.nh)
  , server_timeout_(params.server_timeout)
  , wait_for_server_timeout_(params.wait_for_server_timeout)
  , default_action_name_(params.default_port_value)
{
  auto port_it = config().input_ports.find("action_name");
  if(port_it != config().input_ports.end())
  {
    const std::string& bb_service_name = port_it->second;

    if(isBlackboardPointer(bb_service_name))
    {
      action_name_should_be_checked_ = true;
    }
    else if(!bb_service_name.empty())
    {
      createClient(bb_service_name);
    }
  }
  if(!client_instance_ && !params.default_port_value.empty())
  {
    createClient(params.default_port_value);
  }
}

template <class T>
inline RosActionNode<T>::~RosActionNode()
{
  if(request_)
  {
    request_->owner = nullptr;
  }
}

template <class T>
inline void RosActionNode<T>::throwIfDispatching()
{
  if(dispatchActive())
  {
    throw BT::LogicError("RosActionNode: lifecycle changes during callback dispatch for "
                         "the same action type "
                         "are forbidden; return SUCCESS or FAILURE from onFeedback to "
                         "request completion");
  }
}

template <class T>
inline bool RosActionNode<T>::createClient(const std::string& action_name)
{
  throwIfDispatching();
  if(action_name.empty())
  {
    throw BT::RuntimeError("action_name is empty");
  }

  std::unique_lock lk(getMutex());
  auto node = node_.lock();
  if(!node)
  {
    throw BT::RuntimeError("The ROS node went out of scope. BT::RosNodeParams doesn't "
                           "take the "
                           "ownership of the node.");
  }
  action_client_key_ = std::string(node->get_fully_qualified_name()) + "/" + action_name;

  auto& registry = getRegistry();
  auto it = registry.find(action_client_key_);
  if(it == registry.end() || it->second.expired())
  {
    client_instance_ = std::make_shared<ActionClientInstance>(node, action_name);
    registry.insert_or_assign(action_client_key_, client_instance_);
  }
  else
  {
    client_instance_ = it->second.lock();
  }

  action_name_ = action_name;

  bool found =
      client_instance_->action_client->wait_for_action_server(wait_for_server_timeout_);
  if(!found)
  {
    RCLCPP_ERROR(logger(), "%s: Action server with name '%s' is not reachable.",
                 name().c_str(), action_name_.c_str());
  }
  return found;
}

template <class T>
inline void RosActionNode<T>::setActionName(const std::string& action_name)
{
  throwIfDispatching();
  action_name_ = action_name;
  createClient(action_name);
}

template <class T>
// Keep invocation transitions together so request ownership is visible at each return.
// NOLINTNEXTLINE(readability-function-size)
inline BT::NodeStatus RosActionNode<T>::tick()
{
  throwIfDispatching();
  if(!rclcpp::ok())
  {
    halt();
    return BT::NodeStatus::FAILURE;
  }

  auto check_status = [](BT::NodeStatus status) {
    if(!BT::isStatusCompleted(status))
    {
      throw BT::LogicError("RosActionNode: the callback must return either SUCCESS or "
                           "FAILURE");
    }
    return status;
  };

  if(status() == BT::NodeStatus::IDLE)
  {
    if(request_)
    {
      // ponytail: preserve bounded reuse; persistent cleanup needs an external owner.
      releaseRequest(request_);
      request_.reset();
    }
    setStatus(BT::NodeStatus::RUNNING);
    Goal goal;
    if(!setGoal(goal))
    {
      return check_status(onFailure(BT::INVALID_GOAL, {}));
    }

    if(!client_instance_ || action_name_should_be_checked_)
    {
      const auto endpoint = getInput<std::string>("action_name");
      if(!endpoint)
      {
        RCLCPP_ERROR(logger(), "%s: Invalid action_name port: %s", name().c_str(),
                     endpoint.error().c_str());
        return check_status(onFailure(BT::INVALID_GOAL, {}));
      }
      const auto& action_name =
          endpoint.value().empty() ? default_action_name_ : endpoint.value();
      if(!client_instance_ || action_name != action_name_)
      {
        createClient(action_name);
      }
    }
    if(!client_instance_)
    {
      throw BT::RuntimeError("RosActionNode: no client was specified, neither as default "
                             "nor in the ports");
    }
    auto& action_client = client_instance_->action_client;
    if(!action_client->action_server_is_ready())
    {
      return check_status(onFailure(BT::SERVER_UNREACHABLE, {}));
    }

    auto pending = std::make_shared<Request>();
    pending->client = client_instance_;
    pending->sent_at = now();
    pending->owner = this;
    const std::weak_ptr<Request> weak_request = pending;
    typename ActionClient::SendGoalOptions goal_options;
    goal_options.feedback_callback =
        [weak_request](typename GoalHandle::SharedPtr,
                       const std::shared_ptr<const Feedback> feedback) {
          if(auto request = weak_request.lock(); request && request->owner)
          {
            request->feedback_status = request->owner->onFeedback(feedback);
            if(request->feedback_status == BT::NodeStatus::IDLE)
            {
              throw std::logic_error("onFeedback must not return IDLE");
            }
            request->owner->emitWakeUpSignal();
          }
        };
    goal_options.goal_response_callback =
        [weak_request](typename GoalHandle::SharedPtr handle) {
          if(auto request = weak_request.lock())
          {
            request->goal_handle = handle;
            if(handle)
            {
              // Keep the result future before another leaf can dispatch completion.
              requestResult(request);
            }
          }
        };
    pending->acceptance = action_client->async_send_goal(goal, goal_options);
    request_ = std::move(pending);
    return BT::NodeStatus::RUNNING;
  }

  if(status() != BT::NodeStatus::RUNNING)
  {
    return status();
  }

  auto& request = *request_;
  {
    std::unique_lock<std::mutex> lock(getMutex());
    const DispatchGuard dispatch_guard;
    request.client->callback_executor.spin_some();
    if(request.acceptance.valid())
    {
      // spin_some handles one action event; feedback may precede acceptance.
      request.client->callback_executor.spin_until_future_complete(
          request.acceptance, std::chrono::milliseconds::zero());
    }
  }
  if(request.acceptance.valid())
  {
    if(request.acceptance.wait_for(std::chrono::milliseconds::zero()) !=
       std::future_status::ready)
    {
      if(now() - request.sent_at > rclcpp::Duration(server_timeout_))
      {
        request.owner = nullptr;
        return check_status(onFailure(BT::SEND_GOAL_TIMEOUT, {}));
      }
      return BT::NodeStatus::RUNNING;
    }
    const auto handle = request.acceptance.get();
    request.acceptance = {};
    if(!handle)
    {
      request.owner = nullptr;
      request_.reset();
      return check_status(onFailure(BT::GOAL_REJECTED_BY_SERVER, {}));
    }
  }

  if(request.feedback_status != BT::NodeStatus::RUNNING)
  {
    const auto completion = check_status(request.feedback_status);
    if(releaseRequest(request_))
    {
      request_.reset();
    }
    return completion;
  }
  const auto result = resultIfReady(request);
  if(request.result_error)
  {
    request.owner = nullptr;
    std::rethrow_exception(request.result_error);
  }
  if(!result)
  {
    return BT::NodeStatus::RUNNING;
  }
  request.owner = nullptr;
  request_.reset();
  if(result->code == rclcpp_action::ResultCode::ABORTED)
  {
    return check_status(onFailure(BT::ACTION_ABORTED, result));
  }
  if(result->code == rclcpp_action::ResultCode::CANCELED)
  {
    return check_status(onFailure(BT::ACTION_CANCELLED, result));
  }
  return check_status(onResultReceived(*result));
}

template <class T>
inline void RosActionNode<T>::halt()
{
  throwIfDispatching();
  if(request_ && releaseRequest(request_))
  {
    request_.reset();
  }
  if(status() == BT::NodeStatus::RUNNING)
  {
    onHalt();
  }
  resetStatus();
}

template <class T>
inline void RosActionNode<T>::cancelGoal()
{
  throwIfDispatching();
  if(request_)
  {
    releaseRequest(request_);
    if(status() == BT::NodeStatus::RUNNING)
    {
      request_->owner = this;
      emitWakeUpSignal();
    }
  }
}

template <class T>
inline bool RosActionNode<T>::hasUnresolvedGoal() const
{
  return static_cast<bool>(request_);
}

template <class T>
inline std::exception_ptr RosActionNode<T>::currentTransportError() noexcept
{
  try
  {
    throw;
  }
  catch(const std::exception_ptr& error)
  {
    return error ? error : std::current_exception();
  }
  catch(...)
  {
    return std::current_exception();
  }
}

template <class T>
inline void
RosActionNode<T>::requestResult(const std::shared_ptr<Request>& request) noexcept
{
  const std::weak_ptr<Request> weak_request = request;
  try
  {
    request->result = request->client->requestResult(
        request->goal_handle, [weak_request](const WrappedResult&) {
          if(auto completed = weak_request.lock(); completed && completed->owner)
          {
            completed->owner->emitWakeUpSignal();
          }
        });
    request->result_error = nullptr;
  }
  catch(...)
  {
    // Executor dispatch must finish so an accepted goal remains cancellable.
    request->result_error = currentTransportError();
    if(request->owner)
    {
      request->owner->emitWakeUpSignal();
    }
  }
}

template <class T>
inline std::optional<typename RosActionNode<T>::WrappedResult>
RosActionNode<T>::resultIfReady(Request& request) noexcept
{
  try
  {
    if(request.result.valid() &&
       request.result.wait_for(std::chrono::milliseconds::zero()) ==
           std::future_status::ready)
    {
      return request.result.get();
    }
  }
  catch(...)
  {
    request.result_error = currentTransportError();
    request.result = {};
  }
  return {};
}

template <class T>
inline void RosActionNode<T>::logCleanupError(const char* operation,
                                              std::exception_ptr error) noexcept
{
  try
  {
    std::rethrow_exception(std::move(error));
  }
  catch(const std::exception& cause)
  {
    RCLCPP_WARN(logger(), "%s failed during cleanup for [%s]: %s", operation,
                name().c_str(), cause.what());
  }
  catch(...)
  {
    RCLCPP_WARN(logger(),
                "%s failed during cleanup for [%s] with a non-standard exception",
                operation, name().c_str());
  }
}

template <class T>
// Keep acceptance, cancellation, and result waits under one executor lock.
// NOLINTNEXTLINE(readability-function-size)
inline bool RosActionNode<T>::releaseRequest(std::shared_ptr<Request> request) noexcept
{
  request->owner = nullptr;
  try
  {
    std::unique_lock<std::mutex> lock(getMutex());
    const DispatchGuard dispatch_guard;
    auto& executor = request->client->callback_executor;
    constexpr auto SUCCESS = rclcpp::FutureReturnCode::SUCCESS;
    if(request->acceptance.valid())
    {
      if(executor.spin_until_future_complete(request->acceptance, server_timeout_) !=
         SUCCESS)
      {
        RCLCPP_WARN(logger(), "Goal acceptance unresolved during cancellation for [%s]",
                    name().c_str());
        return false;
      }
      request->goal_handle = request->acceptance.get();
      if(!request->goal_handle)
      {
        return true;
      }
      request->acceptance = {};
    }
    if(resultIfReady(*request))
    {
      return true;
    }

    // A result-request failure must not prevent cancellation through the accepted handle.
    try
    {
      const auto cancellation =
          request->client->action_client->async_cancel_goal(request->goal_handle);
      if(executor.spin_until_future_complete(cancellation, server_timeout_) != SUCCESS)
      {
        RCLCPP_WARN(logger(), "Cancellation response timed out for [%s]", name().c_str());
      }
      else
      {
        const auto response = cancellation.get();
        if(response->return_code != ActionClient::CancelResponse::ERROR_NONE ||
           response->goals_canceling.empty())
        {
          RCLCPP_WARN(logger(), "Cancellation was not accepted for [%s]", name().c_str());
        }
      }
    }
    catch(...)
    {
      logCleanupError("Goal cancellation", currentTransportError());
    }

    if(!request->result.valid())
    {
      requestResult(request);
    }
    if(request->result_error)
    {
      logCleanupError("Result retrieval", request->result_error);
      return false;
    }
    if(executor.spin_until_future_complete(request->result, server_timeout_) != SUCCESS)
    {
      RCLCPP_WARN(logger(), "Terminal result unresolved during cancellation for [%s]",
                  name().c_str());
      return false;
    }
    const bool completed = resultIfReady(*request).has_value();
    if(request->result_error)
    {
      logCleanupError("Result retrieval", request->result_error);
    }
    return completed;
  }
  catch(...)
  {
    // BT::Tree also invokes halt during destruction.
    logCleanupError("Action cleanup", currentTransportError());
    return false;
  }
}

}  // namespace BT
