# ROS Behavior Wrappers

A base class is used to implement each Behavior type for a ROS Action, Service or Topic Publisher / Subscriber.

Users are expected to create a derived class and can implement the following methods.

# ROS Action Behavior Wrapper

### bool setGoal(Goal& goal)

Required callback that allows the user to set the goal message.
Return false if the request should not be sent. In that case, RosActionNode::onFailure(INVALID_GOAL) will be called.

### BT::NodeStatus onResultReceived(const WrappedResult& result)

Required callback invoked when the result is received by the server.
It is up to the user to define if the action returns SUCCESS or FAILURE.

### BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)

Optional callback invoked when the feedback is received.
It generally returns RUNNING, but the user can also use this callback to cancel the current action and return SUCCESS or FAILURE.

### BT::NodeStatus onFailure(ActionNodeErrorCode error)

Optional callback invoked when something goes wrong.
It must return either SUCCESS or FAILURE.

### void onHalt()

Optional callback executed when the node is halted.
Note that cancelGoal() is done automatically.

### Action lifecycle and cancellation

Each invocation retains its own acceptance and result futures, goal handle, and originating client.
Late callbacks cannot access a detached or destroyed leaf or change a later invocation.
Serialize tick, halt, destruction, and dispatch through shared clients, and keep the ROS node and default context alive through halt.
During callback dispatch, lifecycle changes for the same action type throw `BT::LogicError`, including calls to peers on other endpoints.
Request completion from `onFeedback()` by returning `SUCCESS` or `FAILURE`.
Feedback completion requests are not latched; later feedback can replace the status before the owning leaf ticks.

`cancelGoal()` attempts bounded cleanup and leaves a completed result for the next tick.
`halt()` retries retained cleanup regardless of BT status, calls `onHalt()` once for a running invocation, and resets the status.
Transport errors are logged during cleanup, while result retrieval errors remain available for tick to report.
The result-aware `onFailure(ActionNodeErrorCode, const std::optional<WrappedResult>&)` overload receives a result when available and otherwise delegates to the single-argument hook.

`hasUnresolvedGoal()` reports whether this leaf retains an invocation, including after acceptance timeout or unconfirmed halt.
A later halt retries cleanup; each acceptance, cancellation, and result wait has its own `RosNodeParams::server_timeout` budget.
A new invocation attempts cleanup once and may abandon an unresolved request before sending another goal.
The query does not track abandoned requests, and bounded cancellation does not establish that an action has stopped.
There is no background cleanup timer or overall execution deadline; acceptance timeouts use the ROS node clock.

`setActionName()` changes the endpoint for future invocations; retained cleanup uses its original client.
The `action_name` input takes precedence when nonempty; an empty input uses `RosNodeParams::default_port_value`.

# ROS Service Behavior Wrapper

### setRequest(typename Request::SharedPtr& request)

Required callback that allows the user to set the request message (ServiceT::Request).

### BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response)

Required callback invoked when the response is received by the server.
It is up to the user to define if this returns SUCCESS or FAILURE.

### BT::NodeStatus onFailure(ServiceNodeErrorCode error)

Optional callback invoked when something goes wrong; you can override it.
It must return either SUCCESS or FAILURE.

# ROS Topic Publisher Wrapper

### bool setMessage(TopicT& msg)

Required callback invoked in tick to allow the user to pass the message to be published.

# ROS Topic Subscriber Wrapper

### NodeStatus onTick(const std::shared_ptr<TopicT>& last_msg)

Required callback invoked in the tick. You must return either SUCCESS of FAILURE.

### bool latchLastMessage()

Optional callback to latch the message that has been processed.
If returns false and no new message is received, before next call there will be no message to process.
If returns true, the next call will process the same message again, if no new message received.
