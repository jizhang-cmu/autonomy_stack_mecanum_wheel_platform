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

#ifndef DOMAIN_BRIDGE__DOMAIN_BRIDGE_HPP_
#define DOMAIN_BRIDGE__DOMAIN_BRIDGE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/executor.hpp"

#include "domain_bridge/domain_bridge_config.hpp"
#include "domain_bridge/domain_bridge_options.hpp"
#include "domain_bridge/topic_bridge.hpp"
#include "domain_bridge/topic_bridge_options.hpp"
#include "domain_bridge/service_bridge_options.hpp"
#include "domain_bridge/visibility_control.hpp"

namespace domain_bridge
{

class DomainBridgeImpl;

/// Bridge ROS entities across different domains.
/**
 * This class is responsible for forwarding ROS messages across different ROS domains.
 * Topics can be bridged from one domain to another by calling `bridge_topic()`.
 *
 * Internally, the domain bridge creates a ROS node for each domain ID involved in the bridge.
 * It is up to the user to provide the execution model, which is used for processing subscription
 * callbacks for example.
 * A user can add the bridge to an executor of their choice with `add_to_executor()`.
 * Optionally, the user can configure callback groups per bridged topic via `TopicBridgeOptions`
 * when calling `bridge_topic()`.
 *
 * Example usage:
 *
 * ```c++
 * // Create a domain bridge
 * domain_bridge::DomainBridge domain_bridge;
 *
 * // Bridge one or more topics
 * domain_bridge.bridge_topic("/clock", "rosgraph_msgs/msg/Clock", 5, 10);
 * domain_bridge.bridge_topic("image", "sensor_msgs/msg/Image", 5, 10);
 * // ...
 *
 * // Create an executor and add the bridge nodes to it
 * rclcpp::executors::SingleThreadedExecutor executor;
 * domain_bridge.add_to_executor(executor);
 *
 * // Start executing callbacks, hence forwarding messages across the bridge
 * executor.spin();
 * ```
 */
class DomainBridge
{
public:
  /// Constructor.
  DOMAIN_BRIDGE_PUBLIC
  explicit DomainBridge(const DomainBridgeOptions & options = DomainBridgeOptions());

  /// Constructor from DomainBridgeConfig.
  /**
   * Constructor that also creates bridges based on the passed configuration.
   */
  DOMAIN_BRIDGE_PUBLIC
  explicit DomainBridge(const DomainBridgeConfig & config);

  /// Move constructor.
  DOMAIN_BRIDGE_PUBLIC
  DomainBridge(DomainBridge && other);

  /// Move assignment operator.
  DOMAIN_BRIDGE_PUBLIC
  DomainBridge &
  operator=(DomainBridge && other) = default;

  /// Destructor.
  DOMAIN_BRIDGE_PUBLIC
  ~DomainBridge();

  DOMAIN_BRIDGE_PUBLIC
  /// Get the options for this domain bridge.
  DomainBridgeOptions
  get_domain_bridge_options() const;

  /// Add the bridge to an executor.
  /**
   * When the executor is running ROS traffic will be bridged between domains.
   * Only topics registered with \ref bridge_topic() will have their messages forwarded.
   *
   * This method will add all nodes associated with this bridge to the provided executor.
   *
   * \param executor: The executor to add this domain bridge to.
   */
  DOMAIN_BRIDGE_PUBLIC
  void add_to_executor(rclcpp::Executor & executor);

  /// Get the bridge nodes.
  /**
   * Each bridged domain has its own node (with its own context). Callers that
   * need to spin the bridge with per-context executors can use this to enumerate
   * the nodes.
   */
  DOMAIN_BRIDGE_PUBLIC
  std::vector<std::shared_ptr<rclcpp::Node>> get_bridge_nodes() const;

  /// Bridge a topic from one domain to another.
  /**
   * \param topic: Name of the topic to bridge.
   * \param type: Name of the topic type (e.g. "example_interfaces/msg/String")
   * \param from_domain_id: Domain ID the bridge will use to subscribe to the topic.
   * \param to_domain_id: Domain ID the bridge will use to publish to the topic.
   * \param options: Options for bridging the topic.
   */
  DOMAIN_BRIDGE_PUBLIC
  void bridge_topic(
    const std::string & topic,
    const std::string & type,
    size_t from_domain_id,
    size_t to_domain_id,
    const TopicBridgeOptions & options = TopicBridgeOptions());

  /// Bridge a topic from one domain to another.
  /**
   * \param topic_bridge: Struct containing info about the topic to bridge.
   * \param options: Options for bridging the topic.
   */
  DOMAIN_BRIDGE_PUBLIC
  void bridge_topic(
    const TopicBridge & topic_bridge,
    const TopicBridgeOptions & options = TopicBridgeOptions());

  /// Bridge a service from one domain to another.
  /**
   * \param service: Name of the service to be bridged.
   * \param from_domain_id: Domain id where there is a service server to be bridged.
   * \param to_domain_id: Domain id where we want to make request to the bridged service.
   * \param options: Options for bridging the topic.
   */
  template<typename ServiceT>
  void bridge_service(
    const std::string & service,
    size_t from_domain_id,
    size_t to_domain_id,
    const ServiceBridgeOptions & options = ServiceBridgeOptions());

  /// Get bridged topics.
  /**
   * \return Topic bridges created by `bridge_topic()`.
   */
  DOMAIN_BRIDGE_PUBLIC
  std::vector<TopicBridge> get_bridged_topics() const;

private:
  std::unique_ptr<DomainBridgeImpl> impl_;
};  // class DomainBridge

}  // namespace domain_bridge

// Implementation of the bridge_service() template method.
#include "domain_bridge/service_bridge_impl.inc"

#endif  // DOMAIN_BRIDGE__DOMAIN_BRIDGE_HPP_
