# behaviortree_ros2

## Overview

Provide ROS action, service, topic, and tree-execution wrappers for BehaviorTree.CPP.
The library supplies transport and tree integration; derived leaves supply application behavior.

## Usage

Link `behaviortree_ros2::behaviortree_ros2` and include the required wrapper header.
See [ROS Behavior Wrappers](ros_behavior_wrappers.md) for ports and callbacks and [TreeExecutionServer](tree_execution_server.md) for the executor API and configuration.
Action clients use `behaviortree_ros2/bt_action_node.hpp` and derive from `BT::RosActionNode<ActionT>`.
Implement `setGoal()` and `onResultReceived()`, and merge ports through `providedBasicPorts()`.
Cancellation, endpoint changes, and unresolved-request inspection use `cancelGoal()`, `setActionName()`, and `hasUnresolvedGoal()`.

## Dependencies

Runtime dependencies include `rclcpp`, `rclcpp_action`, `ament_index_cpp`, `behaviortree_cpp`, `btcpp_ros2_interfaces`, and `generate_parameter_library`.
The tests additionally require `ament_cmake_gtest`.

## Testing

`test/test_ros_action_node.cpp` provides 29 fake Sleep-action regressions through the `test_ros_action_node` target.
Coverage includes goal rejection, acceptance timeouts, cancellation identity and retries, endpoint selection, shared-client feedback and wake-ups, and callback validation.
Subprocess regressions cover late dispatch after destruction and result-transport failures during cancellation and tree destruction.
Seven feedback re-entry cases cover lifecycle calls on the leaf and peers using shared or separate clients.

## Known limitations

Serialize tick, halt, destruction, and shared-client dispatch, retaining the ROS node and default context through halt.
Bounded cancellation and `hasUnresolvedGoal()` do not establish physical stopping or track abandoned goals.
See the [action lifecycle contract](ros_behavior_wrappers.md#action-lifecycle-and-cancellation) for reuse, timeout, and feedback limits.
