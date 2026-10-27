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

#include <behaviortree_ros2/bt_action_node.hpp>
#include "btcpp_ros2_interfaces/action/sleep.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/exceptions.h>
#include <behaviortree_cpp/tree_node.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <gtest/gtest.h>
#include <gtest/internal/gtest-port.h>
#include <rclcpp/context.hpp>
#include <rclcpp/contexts/default_context.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/client_goal_handle.hpp>
#include <rclcpp_action/client_goal_handle_impl.hpp>
#include <rclcpp_action/create_client.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/exceptions.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <rclcpp_action/types.hpp>

#include <chrono>
#include <compare>  // IWYU pragma: keep
#include <condition_variable>
#include <cstddef>  // IWYU pragma: keep
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <functional>
#include <future>
#include <initializer_list>
#include <memory>
#include <mutex>
#include <optional>
#include <ostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace BT
{
namespace
{
using Sleep = btcpp_ros2_interfaces::action::Sleep;
using ServerGoal = rclcpp_action::ServerGoalHandle<Sleep>;
using Base = RosActionNode<Sleep>;

// Local discovery/scheduling allowance in seconds; synchronization establishes ordering.
constexpr auto TEST_TIMEOUT = std::chrono::seconds(5);
// Poll and response budgets in milliseconds, chosen to keep deliberate stalls short.
constexpr auto TEST_POLL = std::chrono::milliseconds(1);
constexpr auto RESPONSE_TIMEOUT = std::chrono::milliseconds(100);
// Nominal request duration in milliseconds; the fake server completes only on a test signal.
constexpr std::int32_t GOAL_DURATION_MS = 1;
// A negative feedback cycle is a transport handshake, separate from tested application feedback.
constexpr std::int32_t HANDSHAKE_CYCLE = -1;
// Distinct, dimensionless feedback counters make reordered or collapsed delivery observable.
constexpr std::int32_t FIRST_FEEDBACK_CYCLE = 2;
constexpr std::int32_t SECOND_FEEDBACK_CYCLE = 3;

enum class ResultFailure
{
  none,
  request_exception,
  stored_exception,
  exceptional_future
};

constexpr auto RESULT_FAILURE_MESSAGE = "Injected result request failure";

struct Hooks
{
  bool valid_goal = true;
  bool default_feedback = false;
  bool handshake_received = false;
  ResultFailure result_failure = ResultFailure::none;
  BT::NodeStatus feedback_status = BT::NodeStatus::RUNNING;
  BT::NodeStatus result_status = BT::NodeStatus::SUCCESS;
  BT::NodeStatus failure_status = BT::NodeStatus::FAILURE;
  std::function<void()> feedback_callback;
  std::vector<std::int32_t> feedback;
  std::size_t results = 0;
  std::size_t failures = 0;
  std::size_t legacy_failures = 0;
  std::size_t halts = 0;
  std::optional<ActionNodeErrorCode> error;
  std::optional<Base::WrappedResult> result;
};

class SleepNode : public Base
{
public:
  SleepNode(const std::string& name, const BT::NodeConfig& config,
            const BT::RosNodeParams& params, std::shared_ptr<Hooks> hooks)
    : Base(name, config, params), hooks_(std::move(hooks))
  {
    if(hooks_->result_failure != ResultFailure::none)
    {
      client_instance_ =
          std::make_shared<ResultFailureClient>(node_.lock(), action_name_, hooks_);
      EXPECT_TRUE(client_instance_->action_client->wait_for_action_server(
          params.wait_for_server_timeout));
    }
  }

  bool setGoal(Goal& goal) override
  {
    goal.msec_timeout = GOAL_DURATION_MS;
    return hooks_->valid_goal;
  }

  BT::NodeStatus onResultReceived(const WrappedResult& result) override
  {
    ++hooks_->results;
    hooks_->result = result;
    return result.result->done ? hooks_->result_status : BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override
  {
    if(feedback->cycle == HANDSHAKE_CYCLE)
    {
      hooks_->handshake_received = true;
      return Base::onFeedback(feedback);
    }
    hooks_->feedback.push_back(feedback->cycle);
    if(hooks_->feedback_callback)
    {
      hooks_->feedback_callback();
    }
    return hooks_->default_feedback ? Base::onFeedback(feedback) :
                                      hooks_->feedback_status;
  }

  BT::NodeStatus onFailure(ActionNodeErrorCode error,
                           const std::optional<WrappedResult>& result) override
  {
    ++hooks_->failures;
    hooks_->result = result;
    return Base::onFailure(error, result);
  }

  BT::NodeStatus onFailure(ActionNodeErrorCode error) override
  {
    ++hooks_->legacy_failures;
    hooks_->error = error;
    return hooks_->failure_status;
  }

  void onHalt() override
  {
    ++hooks_->halts;
  }

  void spinCallbacks()
  {
    std::unique_lock<std::mutex> lock(getMutex());
    client_instance_->callback_executor.spin_some();
  }

  bool sharesClient(const SleepNode& other) const
  {
    return client_instance_ == other.client_instance_;
  }

private:
  class ResultFailureClient : public ActionClientInstance
  {
  public:
    ResultFailureClient(const std::shared_ptr<rclcpp::Node>& node,
                        const std::string& action_name, std::shared_ptr<Hooks> hooks)
      : ActionClientInstance(node, action_name), hooks_(std::move(hooks))
    {}

    std::shared_future<WrappedResult> requestResult(
        GoalHandle::SharedPtr handle, ActionClient::ResultCallback callback) override
    {
      const auto error = std::make_exception_ptr(
          rclcpp_action::exceptions::UnawareGoalHandleError(RESULT_FAILURE_MESSAGE));
      switch(hooks_->result_failure)
      {
        case ResultFailure::none:
          return ActionClientInstance::requestResult(std::move(handle),
                                                     std::move(callback));
        case ResultFailure::request_exception:
          std::rethrow_exception(error);
        case ResultFailure::stored_exception:
          throw std::exception_ptr(error);
        case ResultFailure::exceptional_future: {
          std::promise<WrappedResult> promise;
          promise.set_exception(error);
          return promise.get_future().share();
        }
      }
      throw std::logic_error("Unknown result failure mode");
    }

  private:
    std::shared_ptr<Hooks> hooks_;
  };

  std::shared_ptr<Hooks> hooks_;
};

class RosActionNodeFixture : public testing::Test
{
public:
  RosActionNodeFixture()
  {
    context_ = rclcpp::contexts::get_global_default_context();
    context_->init(0, nullptr);
    client_node_ = std::make_shared<rclcpp::Node>("action_node_test_client");
    server_node_ = std::make_shared<rclcpp::Node>("action_node_test_server");
    client_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    server_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    client_executor_->add_node(client_node_);
    server_executor_->add_node(server_node_);
    server_ = makeServer("sleep_test");
    other_server_ = makeServer("other_sleep_test");
    cancel_timer_ = server_node_->create_wall_timer(TEST_POLL, [this] {
      std::lock_guard<std::mutex> lock(mutex_);
      if(finish_cancel_)
      {
        for(const auto& goal : goals_)
        {
          if(goal->is_canceling())
          {
            goal->canceled(std::make_shared<Sleep::Result>());
          }
        }
      }
    });
    server_thread_ = std::thread([this] { server_executor_->spin(); });
    for(const auto* endpoint : { "sleep_test", "other_sleep_test" })
    {
      auto discovery = rclcpp_action::create_client<Sleep>(client_node_, endpoint);
      EXPECT_TRUE(discovery->wait_for_action_server(TEST_TIMEOUT));
    }
    params_.nh = client_node_;
    params_.default_port_value = "sleep_test";
    params_.server_timeout = RESPONSE_TIMEOUT;
    params_.wait_for_server_timeout = TEST_TIMEOUT;
    blackboard_ = BT::Blackboard::create();
    blackboard_->set("endpoint", std::string("sleep_test"));
  }

  void TearDown() override
  {
    releaseAcceptance();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      refuse_cancel_ = false;
      finish_cancel_ = true;
    }
    tree_.haltTree();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for(const auto& goal : goals_)
      {
        if(goal->is_active() && !goal->is_canceling())
        {
          goal->abort(std::make_shared<Sleep::Result>());
        }
      }
    }
    server_executor_->cancel();
    server_thread_.join();
    tree_ = BT::Tree{};
    context_->shutdown("test complete");
  }

protected:
  BT::Tree makeTree(const std::shared_ptr<Hooks>& hooks, const std::string& endpoint = "")
  {
    BT::BehaviorTreeFactory factory;
    // Sleep is a built-in BT node, so the ROS leaf needs a distinct registration ID.
    factory.registerNodeType<SleepNode>("RosSleepTest", params_, hooks);
    return factory.createTreeFromText("<root BTCPP_format=\"4\"><BehaviorTree "
                                      "ID=\"Test\">"
                                      "<RosSleepTest action_name=\"" +
                                          endpoint +
                                          "\"/>"
                                          "</BehaviorTree></root>",
                                      blackboard_);
  }

  void createTree(const std::string& endpoint = "")
  {
    tree_ = makeTree(hooks_, endpoint);
  }

  SleepNode& leaf()
  {
    return dynamic_cast<SleepNode&>(*tree_.rootNode());
  }

  bool waitUntil(const std::function<bool()>& predicate)
  {
    const auto deadline = std::chrono::steady_clock::now() + TEST_TIMEOUT;
    while(std::chrono::steady_clock::now() < deadline)
    {
      client_executor_->spin_some();
      if(predicate())
      {
        return true;
      }
      client_executor_->spin_once(TEST_POLL);
    }
    return false;
  }

  testing::AssertionResult startGoal()
  {
    const auto previous = goalCount();
    auto status = tree_.tickExactlyOnce();
    if(status != BT::NodeStatus::RUNNING)
    {
      return testing::AssertionFailure()
             << "Goal submission returned " << BT::toStr(status);
    }
    if(!waitUntil([this, previous] { return goalCount() > previous; }))
    {
      return testing::AssertionFailure() << "The server did not accept the goal";
    }
    hooks_->handshake_received = false;
    if(!waitUntil([this, &status] {
         publishFeedback(HANDSHAKE_CYCLE, goalCount() - 1);
         status = tree_.tickExactlyOnce();
         return status != BT::NodeStatus::RUNNING || hooks_->handshake_received;
       }))
    {
      return testing::AssertionFailure() << "Feedback handshake was not delivered after "
                                            "server acceptance";
    }
    if(status != BT::NodeStatus::RUNNING)
    {
      return testing::AssertionFailure()
             << "Action returned " << BT::toStr(status)
             << " before the feedback handshake; error: "
             << (hooks_->error ? BT::toStr(*hooks_->error) : "none");
    }
    return testing::AssertionSuccess();
  }

  void expectFeedbackReentryRejected(const std::function<void(SleepNode&)>& operation)
  {
    createTree();
    ASSERT_TRUE(startGoal());
    const auto checked = std::make_shared<bool>(false);
    hooks_->feedback_callback = [this, operation, checked] {
      EXPECT_THROW(operation(leaf()), BT::LogicError);
      *checked = true;
    };
    publishFeedback(FIRST_FEEDBACK_CYCLE, goalCount() - 1);
    const bool received = waitUntil([this, checked] {
      // Dispatch through the public lifecycle path that holds the action-type mutex.
      tree_.tickExactlyOnce();
      return *checked;
    });
    hooks_->feedback_callback = {};
    EXPECT_TRUE(received);
    EXPECT_EQ(leaf().status(), BT::NodeStatus::RUNNING);
    EXPECT_TRUE(leaf().hasUnresolvedGoal());
    EXPECT_EQ(hooks_->halts, 0U);
    publishFeedback(SECOND_FEEDBACK_CYCLE, goalCount() - 1);
    EXPECT_TRUE(waitUntil([this] {
      EXPECT_EQ(tree_.tickExactlyOnce(), BT::NodeStatus::RUNNING);
      return !hooks_->feedback.empty() &&
             hooks_->feedback.back() == SECOND_FEEDBACK_CYCLE;
    }));
    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_TRUE(cancellations_.empty());
  }

  void expectPeerFeedbackReentryRejected(const std::string& endpoint, bool shared_client)
  {
    const auto peer =
        std::make_shared<BT::Tree>(makeTree(std::make_shared<Hooks>(), endpoint));
    EXPECT_EQ(peer->tickExactlyOnce(), BT::NodeStatus::RUNNING);
    ASSERT_TRUE(waitUntil([this] { return goalCount() == 1; }));
    expectFeedbackReentryRejected([peer, shared_client](SleepNode& node) {
      auto& peer_leaf = dynamic_cast<SleepNode&>(*peer->rootNode());
      EXPECT_EQ(node.sharesClient(peer_leaf), shared_client);
      peer_leaf.cancelGoal();
    });
    auto& peer_leaf = dynamic_cast<SleepNode&>(*peer->rootNode());
    EXPECT_EQ(peer_leaf.status(), BT::NodeStatus::RUNNING);
    EXPECT_TRUE(peer_leaf.hasUnresolvedGoal());
    peer->haltTree();
  }

  void releaseAcceptance()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      hold_acceptance_ = false;
    }
    acceptance_condition_.notify_all();
  }

  std::size_t goalCount()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return goals_.size();
  }

  std::size_t requestCount()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return endpoints_.size();
  }

  void finish(bool abort = false)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto result = std::make_shared<Sleep::Result>();
    result->done = true;
    if(abort)
    {
      goals_.back()->abort(result);
    }
    else
    {
      goals_.back()->succeed(result);
    }
  }

  void publishFeedback(std::int32_t cycle, std::size_t index = 0)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto feedback = std::make_shared<Sleep::Feedback>();
    feedback->cycle = cycle;
    goals_.at(index)->publish_feedback(feedback);
  }

  rclcpp_action::Server<Sleep>::SharedPtr makeServer(const std::string& endpoint)
  {
    return rclcpp_action::create_server<Sleep>(
        server_node_, endpoint,
        [this, endpoint](const rclcpp_action::GoalUUID&,
                         const std::shared_ptr<const Sleep::Goal>&) {
          std::unique_lock<std::mutex> lock(mutex_);
          endpoints_.push_back(endpoint);
          acceptance_condition_.wait(lock, [this] { return !hold_acceptance_; });
          return reject_goal_ ? rclcpp_action::GoalResponse::REJECT :
                                rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](const std::shared_ptr<ServerGoal>& goal) {
          std::lock_guard<std::mutex> lock(mutex_);
          cancellations_.push_back(goal->get_goal_id());
          return refuse_cancel_ ? rclcpp_action::CancelResponse::REJECT :
                                  rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<ServerGoal>& goal) {
          std::lock_guard<std::mutex> lock(mutex_);
          goals_.push_back(goal);
        });
  }

  BT::RosNodeParams params_;
  BT::Tree tree_;
  BT::Blackboard::Ptr blackboard_;
  std::shared_ptr<Hooks> hooks_ = std::make_shared<Hooks>();
  std::mutex mutex_;
  bool hold_acceptance_ = false;
  bool reject_goal_ = false;
  bool refuse_cancel_ = false;
  bool finish_cancel_ = true;
  std::vector<std::shared_ptr<ServerGoal>> goals_;
  std::vector<std::string> endpoints_;
  std::vector<rclcpp_action::GoalUUID> cancellations_;

private:
  std::shared_ptr<rclcpp::Context> context_;
  std::shared_ptr<rclcpp::Node> client_node_;
  std::shared_ptr<rclcpp::Node> server_node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> client_executor_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> server_executor_;
  rclcpp_action::Server<Sleep>::SharedPtr server_;
  rclcpp_action::Server<Sleep>::SharedPtr other_server_;
  rclcpp::TimerBase::SharedPtr cancel_timer_;
  std::thread server_thread_;
  std::condition_variable acceptance_condition_;
};

TEST_F(RosActionNodeFixture, rejects_invalid_goals_without_sending)
{
  hooks_->valid_goal = false;
  createTree();
  EXPECT_EQ(tree_.tickExactlyOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(hooks_->error, BT::INVALID_GOAL);
  EXPECT_EQ(hooks_->failures, 1U);
  EXPECT_EQ(hooks_->legacy_failures, 1U);
  EXPECT_FALSE(hooks_->result.has_value());
  EXPECT_FALSE(leaf().hasUnresolvedGoal());
  EXPECT_EQ(requestCount(), 0U);
}

TEST_F(RosActionNodeFixture, reports_an_unavailable_server_without_sending)
{
  params_.default_port_value = "missing_sleep_test";
  params_.wait_for_server_timeout = RESPONSE_TIMEOUT;
  createTree();
  EXPECT_EQ(tree_.tickExactlyOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(hooks_->error, BT::SERVER_UNREACHABLE);
  EXPECT_FALSE(leaf().hasUnresolvedGoal());
  EXPECT_EQ(requestCount(), 0U);
}

TEST_F(RosActionNodeFixture, rejects_an_unset_dynamic_endpoint_without_sending)
{
  createTree("{missing_endpoint}");
  EXPECT_EQ(tree_.tickExactlyOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(hooks_->error, BT::INVALID_GOAL);
  EXPECT_FALSE(leaf().hasUnresolvedGoal());
  EXPECT_EQ(requestCount(), 0U);
}

TEST_F(RosActionNodeFixture, routes_success_and_abort_with_the_result_payload)
{
  createTree();
  for(const bool abort : { false, true })
  {
    ASSERT_TRUE(startGoal());
    finish(abort);
    const auto expected = abort ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    ASSERT_TRUE(
        waitUntil([this, expected] { return tree_.tickExactlyOnce() == expected; }));
    ASSERT_TRUE(hooks_->result.has_value());
    EXPECT_TRUE(hooks_->result->result->done);
    EXPECT_EQ(hooks_->result->code, abort ? rclcpp_action::ResultCode::ABORTED :
                                            rclcpp_action::ResultCode::SUCCEEDED);
    EXPECT_FALSE(leaf().hasUnresolvedGoal());
    tree_.haltTree();
  }
  EXPECT_EQ(hooks_->results, 1U);
  EXPECT_EQ(hooks_->failures, 1U);
  EXPECT_EQ(hooks_->legacy_failures, 1U);
  EXPECT_EQ(hooks_->error, BT::ACTION_ABORTED);
}

TEST_F(RosActionNodeFixture, direct_cancellation_routes_the_cancelled_result)
{
  createTree();
  leaf().cancelGoal();
  ASSERT_TRUE(startGoal());
  tree_.sleep(std::chrono::milliseconds::zero());
  leaf().cancelGoal();
  EXPECT_TRUE(tree_.sleep(std::chrono::milliseconds::zero()));
  ASSERT_EQ(tree_.tickExactlyOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(hooks_->error, BT::ACTION_CANCELLED);
  ASSERT_TRUE(hooks_->result.has_value());
  EXPECT_EQ(hooks_->result->code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_FALSE(leaf().hasUnresolvedGoal());
  EXPECT_EQ(hooks_->halts, 0U);
  leaf().cancelGoal();
  std::lock_guard<std::mutex> lock(mutex_);
  EXPECT_EQ(cancellations_.size(), 1U);
}

TEST_F(RosActionNodeFixture, cancels_the_second_goal_before_consuming_its_acceptance)
{
  createTree();
  ASSERT_TRUE(startGoal());
  finish();
  ASSERT_TRUE(
      waitUntil([this] { return tree_.tickExactlyOnce() == BT::NodeStatus::SUCCESS; }));
  tree_.haltTree();
  ASSERT_EQ(tree_.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  // One completed request and its successor must both have reached the server.
  ASSERT_TRUE(waitUntil([this] { return goalCount() == 2U; }));
  tree_.haltTree();
  EXPECT_FALSE(leaf().hasUnresolvedGoal());
  EXPECT_EQ(leaf().status(), BT::NodeStatus::IDLE);
  std::lock_guard<std::mutex> lock(mutex_);
  ASSERT_EQ(cancellations_.size(), 1U);
  EXPECT_EQ(cancellations_.front(), goals_.back()->get_goal_id());
  EXPECT_FALSE(goals_.back()->is_active());
}

TEST_F(RosActionNodeFixture, halt_handles_rejection_before_consuming_acceptance)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    reject_goal_ = true;
  }
  createTree();
  ASSERT_EQ(tree_.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  ASSERT_TRUE(waitUntil([this] { return requestCount() == 1U; }));
  tree_.haltTree();
  EXPECT_FALSE(leaf().hasUnresolvedGoal());
  EXPECT_EQ(leaf().status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(hooks_->halts, 1U);
  std::lock_guard<std::mutex> lock(mutex_);
  EXPECT_TRUE(cancellations_.empty());
}

TEST_F(RosActionNodeFixture, reports_rejection_after_direct_cancellation)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    reject_goal_ = true;
  }
  createTree();
  ASSERT_EQ(tree_.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  ASSERT_TRUE(waitUntil([this] { return requestCount() == 1U; }));
  leaf().cancelGoal();
  EXPECT_EQ(tree_.tickExactlyOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(hooks_->error, BT::GOAL_REJECTED_BY_SERVER);
  EXPECT_EQ(hooks_->failures, 1U);
  EXPECT_EQ(hooks_->legacy_failures, 1U);
  EXPECT_FALSE(hooks_->result.has_value());
  EXPECT_FALSE(leaf().hasUnresolvedGoal());
}

TEST_F(RosActionNodeFixture, halt_cancels_a_goal_accepted_after_timeout)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    hold_acceptance_ = true;
  }
  createTree();
  ASSERT_EQ(tree_.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  ASSERT_TRUE(waitUntil([this] { return requestCount() == 1U; }));
  ASSERT_TRUE(
      waitUntil([this] { return tree_.tickExactlyOnce() == BT::NodeStatus::FAILURE; }));
  EXPECT_EQ(hooks_->error, BT::SEND_GOAL_TIMEOUT);
  EXPECT_TRUE(leaf().hasUnresolvedGoal());
  releaseAcceptance();
  ASSERT_TRUE(waitUntil([this] { return goalCount() == 1U; }));
  tree_.haltTree();
  EXPECT_FALSE(leaf().hasUnresolvedGoal());
  std::lock_guard<std::mutex> lock(mutex_);
  ASSERT_EQ(cancellations_.size(), 1U);
  EXPECT_FALSE(goals_.back()->is_active());
}

TEST_F(RosActionNodeFixture,
       repeated_halt_recovers_late_acceptance_without_repeating_the_hook)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    hold_acceptance_ = true;
  }
  createTree();
  ASSERT_EQ(tree_.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  ASSERT_TRUE(waitUntil([this] { return requestCount() == 1U; }));
  tree_.haltTree();
  EXPECT_TRUE(leaf().hasUnresolvedGoal());
  EXPECT_EQ(leaf().status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(hooks_->halts, 1U);
  releaseAcceptance();
  ASSERT_TRUE(waitUntil([this] { return goalCount() == 1U; }));
  tree_.haltTree();
  EXPECT_FALSE(leaf().hasUnresolvedGoal());
  EXPECT_EQ(hooks_->halts, 1U);
  std::lock_guard<std::mutex> lock(mutex_);
  EXPECT_FALSE(goals_.back()->is_active());
}

TEST_F(RosActionNodeFixture, retains_refused_cancellation_and_retries_it)
{
  createTree();
  ASSERT_TRUE(startGoal());
  {
    std::lock_guard<std::mutex> lock(mutex_);
    refuse_cancel_ = true;
  }
  tree_.haltTree();
  EXPECT_TRUE(leaf().hasUnresolvedGoal());
  {
    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_FALSE(cancellations_.empty());
    EXPECT_TRUE(goals_.back()->is_active());
    refuse_cancel_ = false;
  }
  tree_.haltTree();
  EXPECT_FALSE(leaf().hasUnresolvedGoal());
  std::lock_guard<std::mutex> lock(mutex_);
  EXPECT_FALSE(goals_.back()->is_active());
}

TEST_F(RosActionNodeFixture, retains_a_missing_terminal_result_until_the_next_halt)
{
  createTree();
  ASSERT_TRUE(startGoal());
  {
    std::lock_guard<std::mutex> lock(mutex_);
    finish_cancel_ = false;
  }
  tree_.haltTree();
  EXPECT_TRUE(leaf().hasUnresolvedGoal());
  {
    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_TRUE(goals_.back()->is_canceling());
    finish_cancel_ = true;
  }
  tree_.haltTree();
  EXPECT_FALSE(leaf().hasUnresolvedGoal());
}

TEST_F(RosActionNodeFixture, handles_an_early_result_dispatched_by_a_sibling)
{
  createTree();
  auto probe_hooks = std::make_shared<Hooks>();
  auto probe = makeTree(probe_hooks);
  auto& probe_leaf = dynamic_cast<SleepNode&>(*probe.rootNode());
  ASSERT_TRUE(leaf().sharesClient(probe_leaf));
  ASSERT_EQ(tree_.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  ASSERT_TRUE(waitUntil([this] { return goalCount() == 1U; }));
  finish();
  ASSERT_TRUE(waitUntil([this, &probe_leaf] {
    probe_leaf.spinCallbacks();
    return tree_.sleep(std::chrono::milliseconds::zero());
  }));
  EXPECT_EQ(hooks_->results, 0U);
  EXPECT_EQ(tree_.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(hooks_->results, 1U);
  EXPECT_FALSE(leaf().hasUnresolvedGoal());
}

TEST_F(RosActionNodeFixture, cancellation_after_result_dispatch_is_safe)
{
  createTree();
  ASSERT_EQ(tree_.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  ASSERT_TRUE(waitUntil([this] { return goalCount() == 1U; }));
  finish();
  ASSERT_TRUE(waitUntil([this] {
    leaf().spinCallbacks();
    return tree_.sleep(std::chrono::milliseconds::zero());
  }));
  leaf().cancelGoal();
  ASSERT_TRUE(
      waitUntil([this] { return tree_.tickExactlyOnce() == BT::NodeStatus::SUCCESS; }));
  tree_.haltTree();
  std::lock_guard<std::mutex> lock(mutex_);
  EXPECT_TRUE(cancellations_.empty());
}

TEST_F(RosActionNodeFixture, delivers_feedback_and_wakeups_through_a_shared_client)
{
  createTree();
  ASSERT_TRUE(startGoal());
  auto probe = makeTree(std::make_shared<Hooks>());
  auto& probe_leaf = dynamic_cast<SleepNode&>(*probe.rootNode());
  tree_.sleep(std::chrono::milliseconds::zero());
  for(const auto cycle : { FIRST_FEEDBACK_CYCLE, SECOND_FEEDBACK_CYCLE })
  {
    publishFeedback(cycle);
    ASSERT_TRUE(waitUntil([this, &probe_leaf, cycle] {
      probe_leaf.spinCallbacks();
      return !hooks_->feedback.empty() && hooks_->feedback.back() == cycle;
    }));
    EXPECT_TRUE(tree_.sleep(std::chrono::milliseconds::zero()));
  }
  EXPECT_EQ(hooks_->feedback,
            (std::vector<std::int32_t>{ FIRST_FEEDBACK_CYCLE, SECOND_FEEDBACK_CYCLE }));
}

TEST_F(RosActionNodeFixture, preserves_default_feedback_and_feedback_requested_completion)
{
  createTree();
  hooks_->default_feedback = true;
  ASSERT_TRUE(startGoal());
  publishFeedback(0);
  ASSERT_TRUE(waitUntil([this] {
    leaf().spinCallbacks();
    return !hooks_->feedback.empty();
  }));
  EXPECT_EQ(tree_.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  tree_.haltTree();
  hooks_->default_feedback = false;
  for(const auto completion : { BT::NodeStatus::SUCCESS, BT::NodeStatus::FAILURE })
  {
    hooks_->feedback_status = BT::NodeStatus::RUNNING;
    ASSERT_TRUE(startGoal());
    hooks_->feedback_status = completion;
    publishFeedback(0, goalCount() - 1);
    ASSERT_TRUE(
        waitUntil([this, completion] { return tree_.tickExactlyOnce() == completion; }));
    EXPECT_FALSE(leaf().hasUnresolvedGoal());
    tree_.haltTree();
  }
  std::lock_guard<std::mutex> lock(mutex_);
  EXPECT_EQ(
      cancellations_.size(),
      3U);  // Default feedback and both completion statuses each cancel one request.
  EXPECT_FALSE(goals_.back()->is_active());
}

TEST_F(RosActionNodeFixture, rejects_feedback_reentry_into_tick)
{
  expectFeedbackReentryRejected([](SleepNode& node) { node.tick(); });
}

TEST_F(RosActionNodeFixture, rejects_feedback_reentry_into_halt)
{
  expectFeedbackReentryRejected([](SleepNode& node) { node.halt(); });
}

TEST_F(RosActionNodeFixture, rejects_feedback_reentry_into_cancellation)
{
  expectFeedbackReentryRejected([](SleepNode& node) { node.cancelGoal(); });
}

TEST_F(RosActionNodeFixture, rejects_feedback_reentry_into_endpoint_change)
{
  expectFeedbackReentryRejected(
      [](SleepNode& node) { node.setActionName("other_sleep_test"); });
}

TEST_F(RosActionNodeFixture, rejects_feedback_cancelling_a_peer_on_the_shared_client)
{
  expectPeerFeedbackReentryRejected("sleep_test", true);
}

TEST_F(RosActionNodeFixture, rejects_feedback_cancelling_a_peer_on_another_client)
{
  expectPeerFeedbackReentryRejected("other_sleep_test", false);
}

TEST_F(RosActionNodeFixture, rejects_feedback_reentry_during_peer_halt)
{
  createTree();
  ASSERT_TRUE(startGoal());
  {
    std::lock_guard<std::mutex> lock(mutex_);
    hold_acceptance_ = true;
  }
  auto peer = makeTree(std::make_shared<Hooks>());
  auto& peer_leaf = dynamic_cast<SleepNode&>(*peer.rootNode());
  ASSERT_TRUE(leaf().sharesClient(peer_leaf));
  EXPECT_EQ(peer.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  ASSERT_TRUE(waitUntil([this] { return requestCount() > goalCount(); }));

  const auto checked = std::make_shared<bool>(false);
  hooks_->feedback_callback = [this, checked] {
    EXPECT_THROW(leaf().cancelGoal(), BT::LogicError);
    *checked = true;
    releaseAcceptance();
  };
  publishFeedback(FIRST_FEEDBACK_CYCLE);
  const bool received = waitUntil([&peer_leaf, checked] {
    // Pending acceptance forces cleanup to dispatch the live sibling's feedback.
    peer_leaf.halt();
    return *checked;
  });
  hooks_->feedback_callback = {};
  releaseAcceptance();
  EXPECT_TRUE(received);
  EXPECT_EQ(tree_.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_TRUE(leaf().hasUnresolvedGoal());
  EXPECT_EQ(hooks_->halts, 0U);
}

TEST_F(RosActionNodeFixture, rejects_invalid_callback_statuses)
{
  createTree();
  ASSERT_TRUE(startGoal());
  hooks_->feedback_status = BT::NodeStatus::IDLE;
  publishFeedback(0);
  bool rejected = false;
  ASSERT_TRUE(waitUntil([this, &rejected] {
    try
    {
      leaf().tick();
    }
    catch(const std::logic_error&)
    {
      rejected = true;
    }
    return rejected;
  }));
  EXPECT_NO_THROW(tree_.haltTree());
  hooks_->feedback_status = BT::NodeStatus::RUNNING;
  hooks_->result_status = BT::NodeStatus::RUNNING;
  ASSERT_TRUE(startGoal());
  finish();
  rejected = false;
  ASSERT_TRUE(waitUntil([this, &rejected] {
    try
    {
      tree_.tickExactlyOnce();
    }
    catch(const BT::NodeExecutionError& error)
    {
      EXPECT_EQ(error.originalMessage(), "RosActionNode: the callback must return either "
                                         "SUCCESS or FAILURE");
      rejected = true;
    }
    return rejected;
  }));
  tree_.haltTree();
  hooks_->valid_goal = false;
  hooks_->failure_status = BT::NodeStatus::RUNNING;
  EXPECT_THROW(tree_.tickExactlyOnce(), BT::NodeExecutionError);
}

TEST_F(RosActionNodeFixture,
       reuse_after_refusal_is_allowed_without_stale_feedback_or_results)
{
  createTree();
  ASSERT_TRUE(startGoal());
  {
    std::lock_guard<std::mutex> lock(mutex_);
    refuse_cancel_ = true;
  }
  tree_.haltTree();
  ASSERT_TRUE(leaf().hasUnresolvedGoal());
  ASSERT_TRUE(startGoal());
  EXPECT_EQ(goalCount(),
            2U);  // Both the abandoned request and its replacement reached the server.
  hooks_->feedback_status = BT::NodeStatus::FAILURE;
  publishFeedback(0);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto result = std::make_shared<Sleep::Result>();
    result->done = true;
    goals_.front()->succeed(result);
  }
  hooks_->feedback_status = BT::NodeStatus::RUNNING;
  finish();
  ASSERT_TRUE(
      waitUntil([this] { return tree_.tickExactlyOnce() == BT::NodeStatus::SUCCESS; }));
  EXPECT_TRUE(hooks_->feedback.empty());
  EXPECT_EQ(hooks_->results, 1U);
  EXPECT_FALSE(leaf().hasUnresolvedGoal());
}

TEST_F(RosActionNodeFixture, selects_literal_dynamic_fallback_and_programmatic_endpoints)
{
  createTree("other_sleep_test");
  ASSERT_TRUE(startGoal());
  tree_.haltTree();
  createTree("{endpoint}");
  blackboard_->set("endpoint", std::string("other_sleep_test"));
  ASSERT_TRUE(startGoal());
  tree_.haltTree();
  blackboard_->set("endpoint", std::string());
  ASSERT_TRUE(startGoal());
  tree_.haltTree();
  createTree();
  leaf().setActionName("other_sleep_test");
  ASSERT_TRUE(startGoal());
  tree_.haltTree();
  std::lock_guard<std::mutex> lock(mutex_);
  EXPECT_EQ(endpoints_, (std::vector<std::string>{ "other_sleep_test", "other_sleep_test",
                                                   "sleep_test", "other_sleep_test" }));
}

TEST_F(RosActionNodeFixture, endpoint_change_keeps_cleanup_on_the_original_client)
{
  createTree();
  ASSERT_TRUE(startGoal());
  leaf().setActionName("other_sleep_test");
  tree_.haltTree();
  EXPECT_FALSE(leaf().hasUnresolvedGoal());
  std::lock_guard<std::mutex> lock(mutex_);
  ASSERT_EQ(cancellations_.size(), 1U);
  EXPECT_EQ(cancellations_.front(), goals_.front()->get_goal_id());
  EXPECT_FALSE(goals_.front()->is_active());
}

class LateDispatchFixture : public RosActionNodeFixture
{
public:
  // One subprocess retains the shared client across timeout and destruction scenarios.
  // NOLINTNEXTLINE(readability-function-size)
  void TestBody() override
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      hold_acceptance_ = true;
    }
    createTree();
    ASSERT_EQ(tree_.tickExactlyOnce(), BT::NodeStatus::RUNNING);
    ASSERT_TRUE(waitUntil([this] { return requestCount() == 1U; }));
    ASSERT_TRUE(
        waitUntil([this] { return tree_.tickExactlyOnce() == BT::NodeStatus::FAILURE; }));
    auto probe_hooks = std::make_shared<Hooks>();
    auto probe = makeTree(probe_hooks);
    auto& probe_leaf = dynamic_cast<SleepNode&>(*probe.rootNode());
    ASSERT_TRUE(leaf().sharesClient(probe_leaf));
    // The timeout regression also retains the original leaf during dispatch.
    releaseAcceptance();
    ASSERT_TRUE(waitUntil([this] { return goalCount() == 1U; }));
    finish();
    GTEST_LOG_(INFO) << "Late dispatch checkpoint";
    ASSERT_EQ(probe.tickExactlyOnce(), BT::NodeStatus::RUNNING);
    // The timed-out request and first probe request have reached the server.
    ASSERT_TRUE(waitUntil([this] { return goalCount() == 2U; }));
    finish();
    ASSERT_TRUE(waitUntil(
        [&probe] { return probe.tickExactlyOnce() == BT::NodeStatus::SUCCESS; }));
    probe.haltTree();
    // Destroy a running leaf after a bounded halt, retaining its shared client in the probe.
    tree_.haltTree();
    createTree();
    ASSERT_TRUE(startGoal());
    {
      std::lock_guard<std::mutex> lock(mutex_);
      finish_cancel_ = false;
    }
    tree_.haltTree();
    EXPECT_TRUE(leaf().hasUnresolvedGoal());
    publishFeedback(0, goalCount() - 1);
    tree_ = BT::Tree{};
    const auto old_halts = hooks_->halts;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      finish_cancel_ = true;
    }
    ASSERT_EQ(probe.tickExactlyOnce(), BT::NodeStatus::RUNNING);
    // Each late-dispatch scenario sends one original request and one probe request.
    ASSERT_TRUE(waitUntil([this] { return goalCount() == 4U; }));
    finish();
    ASSERT_TRUE(waitUntil(
        [&probe] { return probe.tickExactlyOnce() == BT::NodeStatus::SUCCESS; }));
    EXPECT_EQ(hooks_->halts, old_halts);
    EXPECT_TRUE(hooks_->feedback.empty());
    EXPECT_EQ(probe_hooks->results,
              2U);  // One successful probe result per late-dispatch scenario.
    probe.haltTree();
  }
};

class ResultFailureFixture : public RosActionNodeFixture
{
public:
  // Keep cancellation, recovery, and destruction in the same isolated ROS process.
  // NOLINTNEXTLINE(readability-function-size)
  void TestBody() override
  {
    for(const auto failure :
        { ResultFailure::request_exception, ResultFailure::stored_exception,
          ResultFailure::exceptional_future })
    {
      hooks_->result_failure = failure;
      createTree();
      const auto goal_index = goalCount();
      const auto previous_halts = hooks_->halts;
      ASSERT_EQ(tree_.tickExactlyOnce(), BT::NodeStatus::RUNNING);
      ASSERT_TRUE(waitUntil([this, goal_index] { return goalCount() > goal_index; }));

      // The result request fails while cancellation consumes the pending acceptance.
      EXPECT_NO_THROW(leaf().cancelGoal());
      EXPECT_TRUE(waitUntil([this, goal_index] {
        std::lock_guard<std::mutex> lock(mutex_);
        return !goals_.at(goal_index)->is_active();
      }));
      {
        std::lock_guard<std::mutex> lock(mutex_);
        EXPECT_FALSE(cancellations_.empty());
        if(!cancellations_.empty())
        {
          EXPECT_EQ(cancellations_.back(), goals_.at(goal_index)->get_goal_id());
        }
      }
      EXPECT_TRUE(leaf().hasUnresolvedGoal());
      EXPECT_EQ(leaf().status(), BT::NodeStatus::RUNNING);
      try
      {
        tree_.tickExactlyOnce();
        ADD_FAILURE() << "Result retrieval failure was not reported by tick";
      }
      catch(const BT::NodeExecutionError& error)
      {
        EXPECT_EQ(error.originalMessage(), RESULT_FAILURE_MESSAGE);
      }
      catch(...)
      {
        ADD_FAILURE() << "Tick did not normalize the transport exception";
      }
      EXPECT_NO_THROW(tree_.haltTree());
      EXPECT_EQ(leaf().status(), BT::NodeStatus::IDLE);
      EXPECT_TRUE(leaf().hasUnresolvedGoal());
      EXPECT_EQ(hooks_->halts, previous_halts + 1);

      hooks_->result_failure = ResultFailure::none;
      EXPECT_NO_THROW(tree_.haltTree());
      EXPECT_FALSE(leaf().hasUnresolvedGoal());
      EXPECT_EQ(hooks_->halts, previous_halts + 1);

      hooks_->result_failure = failure;
      createTree();
      const auto destroyed_goal_index = goalCount();
      ASSERT_EQ(tree_.tickExactlyOnce(), BT::NodeStatus::RUNNING);
      ASSERT_TRUE(waitUntil(
          [this, destroyed_goal_index] { return goalCount() > destroyed_goal_index; }));
      GTEST_LOG_(INFO) << "Result-failure cleanup checkpoint";
      {
        // BT::Tree destruction must cancel even when result tracking remains unavailable.
        auto destroyed_tree = std::move(tree_);
      }
      EXPECT_TRUE(waitUntil([this, destroyed_goal_index] {
        std::lock_guard<std::mutex> lock(mutex_);
        return !goals_.at(destroyed_goal_index)->is_active();
      }));
      {
        std::lock_guard<std::mutex> lock(mutex_);
        EXPECT_FALSE(cancellations_.empty());
        if(!cancellations_.empty())
        {
          EXPECT_EQ(cancellations_.back(),
                    goals_.at(destroyed_goal_index)->get_goal_id());
        }
      }
    }
  }
};

template <class Fixture>
void runFixtureSubprocess()
{
  Fixture fixture;
  // Preserve teardown so exceptions cannot destroy a joinable server thread.
  try
  {
    fixture.TestBody();
  }
  catch(const std::exception& error)
  {
    ADD_FAILURE() << error.what();
  }
  catch(...)
  {
    ADD_FAILURE() << "Non-standard exception escaped the fixture body";
  }
  fixture.TearDown();
  const auto* result = testing::UnitTest::GetInstance()->current_test_info()->result();
  for(int index = 0; index < result->total_part_count(); ++index)
  {
    const auto& part = result->GetTestPartResult(index);
    if(part.failed())
    {
      GTEST_LOG_(ERROR) << (part.file_name() ? part.file_name() : "unknown file") << ":"
                        << part.line_number() << ": " << part.message();
    }
  }
  std::_Exit(testing::Test::HasFailure() ? EXIT_FAILURE : EXIT_SUCCESS);
}

TEST(RosActionNodeDeathTest, late_dispatch_after_timeout_and_destruction_is_safe)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(runFixtureSubprocess<LateDispatchFixture>(),
              testing::ExitedWithCode(EXIT_SUCCESS), "Late dispatch checkpoint");
}

TEST(RosActionNodeDeathTest, result_failures_preserve_cancellation_and_safe_destruction)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(runFixtureSubprocess<ResultFailureFixture>(),
              testing::ExitedWithCode(EXIT_SUCCESS), "Result-failure cleanup checkpoint");
}

}  // namespace
}  // namespace BT
