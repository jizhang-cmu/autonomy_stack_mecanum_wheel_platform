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

#include "domain_bridge/domain_bridge.hpp"

#include <zstd.h>

#include <cstddef>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/executor.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"
#include "rclcpp/generic_publisher.hpp"
#include "rclcpp/generic_subscription.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/typesupport_helpers.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rmw/types.h"

#include "domain_bridge/compress_messages.hpp"
#include "domain_bridge/domain_bridge_options.hpp"
#include "domain_bridge/topic_bridge.hpp"
#include "domain_bridge/topic_bridge_options.hpp"
#include "domain_bridge/msg/compressed_msg.hpp"

#include "wait_for_graph_events.hpp"

namespace domain_bridge
{

/// \internal
/**
 * A hack, because PublisherBase doesn't support publishing serialized messages and because
 * GenericPublisher cannot be created with a typesupport handle :/
 */
class SerializedPublisher
{
public:
  template<typename MessageT, typename AllocatorT>
  explicit SerializedPublisher(std::shared_ptr<rclcpp::Publisher<MessageT, AllocatorT>> impl)
  : impl_(std::move(impl)),
    publish_method_pointer_(static_cast<decltype(publish_method_pointer_)>(
        &rclcpp::Publisher<MessageT, AllocatorT>::publish))
  {}

  explicit SerializedPublisher(std::shared_ptr<rclcpp::GenericPublisher> impl)
  : impl_(std::move(impl)),
    publish_method_pointer_(static_cast<decltype(publish_method_pointer_)>(
        &rclcpp::GenericPublisher::publish))
  {}

  void publish(const rclcpp::SerializedMessage & message)
  {
    ((*impl_).*publish_method_pointer_)(message);  // this is a bit horrible, but it works ...
  }

private:
  std::shared_ptr<rclcpp::PublisherBase> impl_;
  using PointerToMemberMethod =
    void (rclcpp::PublisherBase::*)(const rclcpp::SerializedMessage & message);
  PointerToMemberMethod publish_method_pointer_;
};

[[noreturn]] static void unreachable()
{
#if defined(__has_builtin)
#if __has_builtin(__builtin_unreachable)
  __builtin_unreachable();
#endif
#elif (__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 5))
  __builtin_unreachable();
#endif
  throw std::logic_error("This code should be unreachable.");
}

/// Implementation of \ref DomainBridge.
class DomainBridgeImpl
{
public:
  using NodeMap = std::unordered_map<std::size_t, std::shared_ptr<rclcpp::Node>>;
  using TopicBridgeMap = std::map<
    TopicBridge,
    std::pair<
      std::shared_ptr<SerializedPublisher>,
      std::shared_ptr<rclcpp::SubscriptionBase>>>;
  using ServiceBridgeMap = std::map<
    detail::ServiceBridge,
    std::pair<std::shared_ptr<rclcpp::ServiceBase>, std::shared_ptr<rclcpp::ClientBase>>>;
  using TypesupportMap = std::unordered_map<
    std::string, std::shared_ptr<rcpputils::SharedLibrary>>;

  explicit DomainBridgeImpl(const DomainBridgeOptions & options)
  : options_(options)
  {
    switch (options.mode()) {
      case DomainBridgeOptions::Mode::Decompress:
        dctx_ = std::unique_ptr<ZSTD_DCtx, size_t (*)(ZSTD_DCtx *)>(
          ZSTD_createDCtx(),
          ZSTD_freeDCtx);
        break;
      case DomainBridgeOptions::Mode::Compress:
        cctx_ = std::unique_ptr<ZSTD_CCtx, size_t (*)(ZSTD_CCtx *)>(
          ZSTD_createCCtx(),
          ZSTD_freeCCtx);
      default:
        break;
    }
  }

  ~DomainBridgeImpl() = default;

  rclcpp::Context::SharedPtr create_context_with_domain_id(std::size_t domain_id)
  {
    auto context = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions options;
    options.auto_initialize_logging(false).set_domain_id(domain_id);
    context->init(0, nullptr, options);
    return context;
  }

  rclcpp::NodeOptions create_node_options(rclcpp::Context::SharedPtr context)
  {
    rclcpp::NodeOptions options;
    return options.context(context)
           .use_global_arguments(false)
           .start_parameter_services(false)
           .start_parameter_event_publisher(false);
  }

  rclcpp::Node::SharedPtr get_node_for_domain(std::size_t domain_id)
  {
    auto domain_id_node_pair = node_map_.find(domain_id);

    // If we don't already have a node for the domain, create one
    if (node_map_.end() == domain_id_node_pair) {
      if (options_.on_new_domain_callback_) {
        options_.on_new_domain_callback_(domain_id);
      }
      auto context = create_context_with_domain_id(domain_id);
      auto node_options = create_node_options(context);
      std::ostringstream oss;
      oss << options_.name() << "_" << std::to_string(domain_id);
      auto node = std::make_shared<rclcpp::Node>(oss.str(), node_options);
      node_map_[domain_id] = node;
      return node;
    }

    return domain_id_node_pair->second;
  }

  bool
  is_bridging_service(const detail::ServiceBridge & service_bridge) const
  {
    std::lock_guard guard{data_mutex_};
    return bridged_services_.find(service_bridge) != bridged_services_.end();
  }

  void
  add_service_bridge(
    const rclcpp::Node::SharedPtr & node,
    detail::ServiceBridge service_bridge,
    std::function<std::shared_ptr<rclcpp::ServiceBase>()> create_service,
    std::shared_ptr<rclcpp::ClientBase> client)
  {
    std::pair<ServiceBridgeMap::iterator, bool> it_emplaced_pair;
    {
      std::lock_guard guard{data_mutex_};
      it_emplaced_pair = bridged_services_.try_emplace(
        std::move(service_bridge), nullptr, client);
    }
    wait_for_graph_events_.register_on_server_ready_callback(
      std::move(client),
      node,
      [
        this,
        & service = std::get<0>(it_emplaced_pair.first->second),
        create_service = std::move(create_service)]()
      {
        std::lock_guard guard{data_mutex_};
        service = create_service();
      }
    );
  }

  /// Load typesupport library into a cache.
  void load_typesupport_library(std::string type)
  {
    if (loaded_typesupports_.find(type) != loaded_typesupports_.end()) {
      // Typesupport library already loaded
      return;
    }
    loaded_typesupports_[type] = rclcpp::get_typesupport_library(
      type, "rosidl_typesupport_cpp");
  }

  std::shared_ptr<SerializedPublisher>
  create_publisher(
    rclcpp::Node::SharedPtr node,
    const std::string & topic_name,
    const std::string & type,
    const rclcpp::QoS & qos,
    rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> & options)
  {
    std::shared_ptr<SerializedPublisher> publisher;
    if (options_.mode() != DomainBridgeOptions::Mode::Compress) {
      publisher = std::make_shared<SerializedPublisher>(
        node->create_generic_publisher(topic_name, type, qos, options));
    } else {
      publisher = std::make_shared<SerializedPublisher>(
        node->create_publisher<domain_bridge::msg::CompressedMsg>(topic_name, qos, options));
    }
    return publisher;
  }

  std::shared_ptr<rclcpp::SubscriptionBase> create_subscription(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<SerializedPublisher> publisher,
    const std::string & topic_name,
    const std::string & type,
    const rclcpp::QoS & qos,
    rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> & options)
  {
    std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback;
    switch (options_.mode()) {
      case DomainBridgeOptions::Mode::Compress:
        callback = [
          serializer = rclcpp::Serialization<domain_bridge::msg::CompressedMsg>{},
          publisher,
          cctx = cctx_.get()
          ](std::shared_ptr<rclcpp::SerializedMessage> msg)
          {
            // Publish message into the other domain
            domain_bridge::msg::CompressedMsg compressed_msg;
            compressed_msg.data = domain_bridge::compress_message(cctx, std::move(*msg));
            rclcpp::SerializedMessage serialized_compressed_msg;
            serializer.serialize_message(&compressed_msg, &serialized_compressed_msg);
            publisher->publish(serialized_compressed_msg);
          };
        break;
      case DomainBridgeOptions::Mode::Decompress:
        callback = [
          serializer = rclcpp::Serialization<domain_bridge::msg::CompressedMsg>{},
          publisher,
          dctx = dctx_.get()
          ](std::shared_ptr<rclcpp::SerializedMessage> serialized_compressed_msg)
          {
            // Publish message into the other domain
            domain_bridge::msg::CompressedMsg compressed_msg;
            serializer.deserialize_message(serialized_compressed_msg.get(), &compressed_msg);
            rclcpp::SerializedMessage msg = domain_bridge::decompress_message(
              dctx, std::move(compressed_msg.data));
            publisher->publish(msg);
          };
        break;
      default:  // fallthrough
      case DomainBridgeOptions::Mode::Normal:
        callback = [publisher](std::shared_ptr<rclcpp::SerializedMessage> msg) {
            // Publish message into the other domain
            publisher->publish(*msg);
          };
        break;
    }
    if (options_.mode() != DomainBridgeOptions::Mode::Decompress) {
      // Create subscription
      return node->create_generic_subscription(
        topic_name,
        type,
        qos,
        callback,
        options);
    }
    return node->create_subscription<domain_bridge::msg::CompressedMsg>(
      topic_name,
      qos,
      callback,
      options);
  }

  void bridge_topic(
    const TopicBridge & topic_bridge,
    const TopicBridgeOptions & topic_options)
  {
    // Validate topic name
    const std::string & topic = rclcpp::expand_topic_or_service_name(
      topic_bridge.topic_name, options_.name(), "/");

    // If a remap name is provided, then validate it
    // otherwise "remap" to the same name
    std::string topic_remapped = topic;
    if (!topic_options.remap_name().empty()) {
      topic_remapped = rclcpp::expand_topic_or_service_name(
        topic_options.remap_name(), options_.name(), "/");
    }

    const std::string & type = topic_bridge.type_name;
    std::size_t from_domain_id = topic_bridge.from_domain_id;
    std::size_t to_domain_id = topic_bridge.to_domain_id;

    if (topic_options.reversed()) {
      std::swap(to_domain_id, from_domain_id);
    }

    // Ensure 'to' domain and 'from' domain are not equal
    if (to_domain_id == from_domain_id) {
      std::cerr << "Cannot bridge topic '" << topic << "' from domain " <<
        std::to_string(from_domain_id) << " to domain " << std::to_string(to_domain_id) <<
        ". Domain IDs must be different." << std::endl;
      return;
    }

    // Validate type name by loading library support (if not already loaded)
    // Front-loading lets us fail early on invalid type names
    load_typesupport_library(type);

    {
      std::lock_guard guard{data_mutex_};
      // Check if already bridged
      if (bridged_topics_.find(topic_bridge) != bridged_topics_.end()) {
        std::cerr << "Topic '" << topic << "' with type '" << type << "'" <<
          " already bridged from domain " << std::to_string(from_domain_id) <<
          " to domain " << std::to_string(to_domain_id) << ", ignoring" << std::endl;
        return;
      }

      // Create a null entry to avoid duplicate bridges
      bridged_topics_[topic_bridge] = {nullptr, nullptr};
    }

    rclcpp::Node::SharedPtr from_domain_node = get_node_for_domain(from_domain_id);
    rclcpp::Node::SharedPtr to_domain_node = get_node_for_domain(to_domain_id);

    auto create_bridge =
      [this, topic, topic_remapped, topic_bridge, topic_options, from_domain_node, to_domain_node]
        (const QosMatchInfo & qos_match)
      {
        {
          std::lock_guard guard{data_mutex_};
          if (
            nullptr != std::get<0>(bridged_topics_[topic_bridge]) &&
            nullptr != std::get<1>(bridged_topics_[topic_bridge]))
          {
            return;  // bridge already running
          }
        }
        const std::string & type = topic_bridge.type_name;

        // Apply QoS overrides
        const auto & qos_options = topic_options.qos_options();
        rclcpp::QoS qos(qos_options.depth());
        qos.history(qos_options.history());
        if (qos_options.reliability()) {
          qos.reliability(*qos_options.reliability());
        } else {
          qos.reliability(qos_match.qos.reliability());
        }
        if (qos_options.durability()) {
          qos.durability(*qos_options.durability());
        } else {
          qos.durability(qos_match.qos.durability());
        }
        if (qos_options.deadline()) {
          const auto deadline_ns = *qos_options.deadline();
          if (deadline_ns < 0) {
            qos.deadline(
              rclcpp::Duration::from_nanoseconds(std::numeric_limits<std::int64_t>::max()));
          } else {
            qos.deadline(rclcpp::Duration::from_nanoseconds(deadline_ns));
          }
        } else {
          qos.deadline(qos_match.qos.deadline());
        }
        if (qos_options.lifespan()) {
          const auto lifespan_ns = *qos_options.lifespan();
          if (lifespan_ns < 0) {
            qos.lifespan(
              rclcpp::Duration::from_nanoseconds(std::numeric_limits<std::int64_t>::max()));
          } else {
            qos.lifespan(rclcpp::Duration::from_nanoseconds(lifespan_ns));
          }
        } else {
          qos.lifespan(qos_match.qos.lifespan());
        }
        if (qos_options.liveliness()) {
          qos.liveliness(*qos_options.liveliness());
        } else {
          qos.liveliness(qos_match.qos.liveliness());
        }
        if (qos_options.liveliness_lease_duration()) {
          const auto liveliness_lease_duration_ns = *qos_options.liveliness_lease_duration();
          if (liveliness_lease_duration_ns < 0) {
            qos.liveliness_lease_duration(
              rclcpp::Duration::from_nanoseconds(std::numeric_limits<std::int64_t>::max()));
          } else {
            qos.liveliness_lease_duration(
              rclcpp::Duration::from_nanoseconds(liveliness_lease_duration_ns));
          }
        } else {
          qos.liveliness_lease_duration(qos_match.qos.liveliness_lease_duration());
        }

        // Print any match warnings
        for (const auto & warning : qos_match.warnings) {
          std::cerr << warning << std::endl;
        }

        rclcpp::PublisherOptions publisher_options;
        rclcpp::SubscriptionOptions subscription_options;

        // Prevent endless looping on a bidirectional bridge
        // Note: this may not be supported by all rmw implementations
        subscription_options.ignore_local_publications = true;

        publisher_options.callback_group = topic_options.callback_group();
        subscription_options.callback_group = topic_options.callback_group();

        // Create publisher for the 'to_domain'
        // The publisher should be created first so it is available to the subscription callback
        auto publisher = this->create_publisher(
          to_domain_node,
          topic_remapped,
          type,
          qos,
          publisher_options);

        // Create subscription for the 'from_domain'
        auto subscription = this->create_subscription(
          from_domain_node,
          publisher,
          topic,
          type,
          qos,
          subscription_options);

        {
          std::lock_guard guard{data_mutex_};
          this->bridged_topics_[topic_bridge] = {publisher, subscription};
        }
      };
    auto opt_case = static_cast<int>(topic_options.wait_for_subscription()) * 2 +
      static_cast<int>(topic_options.wait_for_publisher());
    switch (opt_case) {
      case 0:  // DON'T WAIT
        {
          domain_bridge::QosMatchInfo qos_match;  // with default qos
          create_bridge(qos_match);
        }
        break;
      case 1:  // WAIT FOR ONLY PUBLISHER, this is the default
        wait_for_graph_events_.set_delay(topic_options.delay());
        wait_for_graph_events_.register_on_publisher_qos_ready_callback(
          topic, from_domain_node, create_bridge);
        break;
      case 2:  // WAIT FOR ONLY SUBSCRIPTION
        wait_for_graph_events_.set_delay(topic_options.delay());
        wait_for_graph_events_.register_on_subscription_qos_ready_callback(
          topic, to_domain_node, create_bridge);
        break;
      case 3:  // WAIT FOR SUBSCRIPTION, THEN FOR PUBLISHER
        wait_for_graph_events_.set_delay(topic_options.delay());
        wait_for_graph_events_.register_on_subscription_qos_ready_callback(
          topic,
          to_domain_node,
          [this, create_bridge, topic, from_domain_node](const QosMatchInfo &) {
            this->wait_for_graph_events_.register_on_publisher_qos_ready_callback(
              topic, from_domain_node, create_bridge);
          });
        break;
      default:
        unreachable();
    }
    if (
      domain_bridge::TopicBridgeOptions::AutoRemove::OnNoPublisher ==
      topic_options.auto_remove() ||
      domain_bridge::TopicBridgeOptions::AutoRemove::OnNoPublisherOrSubscription ==
      topic_options.auto_remove())
    {
      this->wait_for_graph_events_.register_on_no_publisher_callback(
        topic, from_domain_node, [this, topic_bridge]()
        {
          std::lock_guard guard{data_mutex_};
          this->bridged_topics_[topic_bridge] = {nullptr, nullptr};
        }
      );
    }
    if (
      domain_bridge::TopicBridgeOptions::AutoRemove::OnNoSubscription ==
      topic_options.auto_remove() ||
      domain_bridge::TopicBridgeOptions::AutoRemove::OnNoPublisherOrSubscription ==
      topic_options.auto_remove())
    {
      this->wait_for_graph_events_.register_on_no_subscription_callback(
        topic, to_domain_node, [this, topic_bridge]()
        {
          std::lock_guard guard{data_mutex_};
          this->bridged_topics_[topic_bridge] = {nullptr, nullptr};
        }
      );
    }
  }

  void add_to_executor(rclcpp::Executor & executor)
  {
    for (const auto & domain_id_node_pair : node_map_) {
      executor.add_node(domain_id_node_pair.second);
    }
  }

  std::vector<TopicBridge> get_bridged_topics() const
  {
    std::vector<TopicBridge> result;
    std::lock_guard guard{data_mutex_};
    for (const auto & bridge : bridged_topics_) {
      result.push_back(bridge.first);
    }
    return result;
  }

  DomainBridgeOptions options_;

  /// Map of domain IDs to ROS nodes
  NodeMap node_map_;

  /// Mutex used to sync all data shared with the WaitForGraphEvents instance.
  mutable std::mutex data_mutex_;

  /// Set of bridged topics
  TopicBridgeMap bridged_topics_;

  /// Set of bridged services
  ServiceBridgeMap bridged_services_;

  /// Cache of typesupport libraries
  TypesupportMap loaded_typesupports_;

  /// QoS event handler
  WaitForGraphEvents wait_for_graph_events_;

  // Warn: The destruction order does matter here!
  // The subscription are capturing the raw pointer!
  std::unique_ptr<ZSTD_DCtx, size_t (*)(ZSTD_DCtx *)> dctx_{nullptr, &ZSTD_freeDCtx};
  std::unique_ptr<ZSTD_CCtx, size_t (*)(ZSTD_CCtx *)> cctx_{nullptr, &ZSTD_freeCCtx};
};  // class DomainBridgeImpl

namespace detail
{
rclcpp::Node::SharedPtr
get_node_for_domain(DomainBridgeImpl & impl, std::size_t domain_id)
{
  return impl.get_node_for_domain(domain_id);
}

bool
is_bridging_service(const DomainBridgeImpl & impl, const detail::ServiceBridge & service_bridge)
{
  return impl.is_bridging_service(service_bridge);
}

void
add_service_bridge(
  DomainBridgeImpl & impl,
  const rclcpp::Node::SharedPtr & node,
  ServiceBridge service_bridge,
  std::function<std::shared_ptr<rclcpp::ServiceBase>()> create_service,
  std::shared_ptr<rclcpp::ClientBase> client)
{
  return impl.add_service_bridge(
    node, std::move(service_bridge), std::move(create_service), std::move(client));
}

const std::string &
get_node_name(const DomainBridgeImpl & impl)
{
  return impl.options_.name();
}
}  // namespace detail

DomainBridge::DomainBridge(const DomainBridgeOptions & options)
: impl_(std::make_unique<DomainBridgeImpl>(options))
{}

DomainBridge::DomainBridge(const DomainBridgeConfig & config)
: impl_(std::make_unique<DomainBridgeImpl>(config.options))
{
  for (const auto & topic_bridge_pair : config.topics) {
    bridge_topic(topic_bridge_pair.first, topic_bridge_pair.second);

    // if topic is bidrectional, bridge it again with the domain IDs swapped
    if (topic_bridge_pair.second.bidirectional()) {
      auto reversed_topic_bridge_pair = topic_bridge_pair;
      std::swap(
        reversed_topic_bridge_pair.first.from_domain_id,
        reversed_topic_bridge_pair.first.to_domain_id);
      bridge_topic(reversed_topic_bridge_pair.first, reversed_topic_bridge_pair.second);
    }
  }
}

DomainBridge::DomainBridge(DomainBridge && other) = default;

DomainBridge::~DomainBridge() = default;

DomainBridgeOptions
DomainBridge::get_domain_bridge_options() const
{
  return impl_->options_;
}

void DomainBridge::add_to_executor(rclcpp::Executor & executor)
{
  impl_->add_to_executor(executor);
}

std::vector<std::shared_ptr<rclcpp::Node>> DomainBridge::get_bridge_nodes() const
{
  std::vector<std::shared_ptr<rclcpp::Node>> nodes;
  nodes.reserve(impl_->node_map_.size());
  for (const auto & domain_id_node_pair : impl_->node_map_) {
    nodes.push_back(domain_id_node_pair.second);
  }
  return nodes;
}

void DomainBridge::bridge_topic(
  const std::string & topic,
  const std::string & type,
  std::size_t from_domain_id,
  std::size_t to_domain_id,
  const TopicBridgeOptions & options)
{
  impl_->bridge_topic({topic, type, from_domain_id, to_domain_id}, options);
}

void DomainBridge::bridge_topic(
  const TopicBridge & topic_bridge,
  const TopicBridgeOptions & options)
{
  impl_->bridge_topic(topic_bridge, options);
}

std::vector<TopicBridge> DomainBridge::get_bridged_topics() const
{
  return impl_->get_bridged_topics();
}

}  // namespace domain_bridge
