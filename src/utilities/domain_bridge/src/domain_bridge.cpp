// Copyright 2021, Open Source Robotics Foundation, Inc.
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

#include <memory>
#include <thread>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

#include "domain_bridge/component_manager.hpp"
#include "domain_bridge/domain_bridge.hpp"
#include "domain_bridge/parse_domain_bridge_yaml_config.hpp"
#include "domain_bridge/process_cmd_line_arguments.hpp"

int main(int argc, char ** argv)
{
  auto arguments = rclcpp::init_and_remove_ros_arguments(argc, argv);

  auto config_rc_pair = domain_bridge::process_cmd_line_arguments(arguments);
  if (!config_rc_pair.first || 0 != config_rc_pair.second) {
    return config_rc_pair.second;
  }
  domain_bridge::DomainBridge domain_bridge(*config_rc_pair.first);

  // Each domain bridge node has its own rclcpp::Context (one per bridged domain
  // ID). rclcpp now enforces that an executor and the nodes it spins share a
  // context, so we cannot put them all on a single executor. Group nodes by
  // context and give each context its own executor and worker thread.
  std::unordered_map<
    rclcpp::Context::SharedPtr,
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor>> bridge_executors;

  for (const auto & node : domain_bridge.get_bridge_nodes()) {
    auto context = node->get_node_base_interface()->get_context();
    auto & executor = bridge_executors[context];
    if (!executor) {
      rclcpp::ExecutorOptions exec_options;
      exec_options.context = context;
      executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(exec_options);
    }
    executor->add_node(node);
  }

  std::vector<std::thread> bridge_threads;
  bridge_threads.reserve(bridge_executors.size());
  for (auto & ctx_executor_pair : bridge_executors) {
    auto executor = ctx_executor_pair.second;
    bridge_threads.emplace_back([executor]() {executor->spin();});
  }

  // The ComponentManager uses the default context, so spin it on the main
  // thread with its own executor.
  auto component_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto component_node = std::make_shared<domain_bridge::ComponentManager>(component_executor);
  component_executor->add_node(component_node);
  component_executor->spin();

  for (auto & ctx_executor_pair : bridge_executors) {
    ctx_executor_pair.second->cancel();
  }
  for (auto & thread : bridge_threads) {
    if (thread.joinable()) {
      thread.join();
    }
  }

  rclcpp::shutdown();
  return 0;
}
