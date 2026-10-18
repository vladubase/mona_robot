// Copyright 2026 vladubase
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


#ifndef MONA_PERCEPTION__LIDAR_MERGER_NODE_HPP_
#define MONA_PERCEPTION__LIDAR_MERGER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/bool.hpp"

#include "laser_geometry/laser_geometry.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/time_synchronizer.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

namespace mona_perception
{
// Alias for standard lifecycle return types
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class LidarMergerNode : public rclcpp_lifecycle::LifecycleNode {
public:
	explicit LidarMergerNode(const rclcpp::NodeOptions &options);
	virtual ~LidarMergerNode();

	// Lifecycle transition overrides
	CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
	CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
	CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
	CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
	CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;
	CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;

private:
	// Callbacks
	void power_callback(const std_msgs::msg::Bool::SharedPtr msg);

	void fused_callback(
		const sensor_msgs::msg::LaserScan::ConstSharedPtr &front,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr &back,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr &left,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr &right);

	void concatenate_pointclouds(
		sensor_msgs::msg::PointCloud2 &base,
		const sensor_msgs::msg::PointCloud2 &add);

	// State variables
	bool has_power_{true};
	std::string target_frame_{};
	double transform_timeout_{0.0};
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr power_sub_;

	// TF and projections
	laser_geometry::LaserProjection projector_;
	std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

	// message_filters subscribers bound to the LifecycleNode
	message_filters::Subscriber<sensor_msgs::msg::LaserScan, rclcpp_lifecycle::LifecycleNode>
	sub_front_, sub_back_, sub_left_, sub_right_;

	// Synchronization policy
	typedef message_filters::sync_policies::ApproximateTime<
			sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan,
			sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan> SyncPolicy;

	std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
	rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};
}  // namespace mona_perception

#endif  // MONA_PERCEPTION__LIDAR_MERGER_NODE_HPP_
