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

// Points per scan:    FOV/res = 150/0.17 ~ 880 points per scan
// Point size:         PointCloud2 - float32 - 4 bytes - 16 bytes
// Volume of scan:     880 * 16 ~ 14200 bytes ~ 13.8 kB
// Update rate:        15 Hz
// Bandwidth:          (13.8 * 15) * 4 = 827 kB/s = 6.6 Mbit/s


#include "mona_perception/lidar_merger_node.hpp"

namespace mona_perception
{
LidarMergerNode::LidarMergerNode(const rclcpp::NodeOptions &options)
	: rclcpp_lifecycle::LifecycleNode("mona_lidar_merger", options) {
}

LidarMergerNode::~LidarMergerNode() {
	// CRITICAL FAULT MITIGATION: Terminate the message_filters synchronizer and TF listener.
	// Prevents background threading from triggering fused_callback() with late
	// sensor messages while the node's memory is actively being reclaimed.
	if (sync_) {
		sync_.reset();
	}
	if (tf_listener_) {
		tf_listener_.reset();
	}
	if (tf_buffer_) {
		tf_buffer_.reset();
	}
}

// 1. CONFIGURE: Allocate memory, initialize TF and subscribers
CallbackReturn LidarMergerNode::on_configure(const rclcpp_lifecycle::State &) {
	RCLCPP_INFO(get_logger(), "Configuring Lidar Merger Component...");

	// Safely declare parameters if they don't exist yet
	if (!this->has_parameter("target_frame")) {
		this->declare_parameter("target_frame", "base_link");
	}
	if (!this->has_parameter("transform_timeout")) {
		this->declare_parameter("transform_timeout", 0.1);
	}

	// Read the parameters into class variables
	target_frame_       = this->get_parameter("target_frame").as_string();
	transform_timeout_  = this->get_parameter("transform_timeout").as_double();

	// Fallback protection just in case
	if (target_frame_.empty()) {
		target_frame_ = "base_link";
		RCLCPP_WARN(this->get_logger(), "target_frame was empty! Defaulting to 'base_link'");
	}

	// Initialize TF
	tf_buffer_      = std::make_shared<tf2_ros::Buffer>(this->get_clock());
	tf_listener_    = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	// Configure QoS for compatibility with LiDAR drivers (Best Effort)
	rmw_qos_profile_t custom_qos = rmw_qos_profile_sensor_data;

	// --- PUBLISHERS --- //
	// Create Lifecycle Publisher
	publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
		"perception/combined_cloud", 10);

	// --- SUBSCRIPTIONS --- //
	power_sub_ = this->create_subscription<std_msgs::msg::Bool>(
		"hardware/power/perception_pc", 10,
		std::bind(&LidarMergerNode::power_callback, this, std::placeholders::_1)
	);

	// Initialize subscribers
	// Pass 'this' (the node), topic name, and QoS profile
	sub_front_.subscribe(this, "lidar_front/scan", custom_qos);
	sub_back_.subscribe(this,  "lidar_back/scan",  custom_qos);
	sub_left_.subscribe(this,  "lidar_left/scan",  custom_qos);
	sub_right_.subscribe(this, "lidar_right/scan", custom_qos);

	// Initialize synchronizer
	// Queue size of 10 messages. This acts as the buffer
	// where the algorithm searches for matching timestamps.
	sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
		SyncPolicy(10), sub_front_, sub_back_, sub_left_, sub_right_);
	sync_->registerCallback(
		std::bind(
			&LidarMergerNode::fused_callback, this,
			std::placeholders::_1, std::placeholders::_2,
			std::placeholders::_3, std::placeholders::_4));

	RCLCPP_INFO(this->get_logger(), "Lidar Merger CONFIGURED. Target frame: %s", target_frame_.c_str());
	return CallbackReturn::SUCCESS;
}

// 2. ACTIVATE: Enable publisher
CallbackReturn LidarMergerNode::on_activate(const rclcpp_lifecycle::State &) {
	RCLCPP_INFO(get_logger(), "Activating Lidar Merger Component...");

	// Lifecycle publishers are muted by default. They must be explicitly activated.
	publisher_->on_activate();

	return CallbackReturn::SUCCESS;
}

// 3. DEACTIVATE: Disable publisher
CallbackReturn LidarMergerNode::on_deactivate(const rclcpp_lifecycle::State &) {
	RCLCPP_INFO(get_logger(), "Deactivating Lidar Merger Component...");

	publisher_->on_deactivate();

	return CallbackReturn::SUCCESS;
}

// 4. CLEANUP: Free memory and resources
CallbackReturn LidarMergerNode::on_cleanup(const rclcpp_lifecycle::State &) {
	RCLCPP_INFO(get_logger(), "Cleaning up Lidar Merger Component...");

	// Reset smart pointers and unsubscribe
	sync_.reset();
	tf_listener_.reset();
	tf_buffer_.reset();
	publisher_.reset();

	sub_front_.unsubscribe();
	sub_back_.unsubscribe();
	sub_left_.unsubscribe();
	sub_right_.unsubscribe();

	return CallbackReturn::SUCCESS;
}

// 5. SHUTDOWN
CallbackReturn LidarMergerNode::on_shutdown(const rclcpp_lifecycle::State &) {
	RCLCPP_INFO(get_logger(), "Shutting down Lidar Merger Component...");

	sync_.reset();
	publisher_.reset();

	return CallbackReturn::SUCCESS;
}

CallbackReturn LidarMergerNode::on_error(const rclcpp_lifecycle::State &previous_state) {
	RCLCPP_ERROR(get_logger(), "Error handling transition from %s", previous_state.label().c_str());

	// Clear resources if necessary
	publisher_.reset();

	return CallbackReturn::FAILURE;  // Transition the node to the FINALIZED state (termination)
}

void LidarMergerNode::power_callback(const std_msgs::msg::Bool::SharedPtr msg) {
	// Prevent redundant processing and I/O log spam
	if (has_power_ == msg->data) {
		return;
	}

	// Commit the validation state.
	has_power_ = msg->data;

	// Execute reaction policies based on the new hardware state.
	if (!has_power_) {
		RCLCPP_ERROR(this->get_logger(), "POWER CUT! Perception module is going blind...");
	} else {
		RCLCPP_INFO(this->get_logger(), "POWER RESTORED! Perception module booting up...");
	}
}

void LidarMergerNode::fused_callback(
	const sensor_msgs::msg::LaserScan::ConstSharedPtr &front,
	const sensor_msgs::msg::LaserScan::ConstSharedPtr &back,
	const sensor_msgs::msg::LaserScan::ConstSharedPtr &left,
	const sensor_msgs::msg::LaserScan::ConstSharedPtr &right) {
	// Ignore incoming data if the node is not in the ACTIVE state
	if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
		return;
	}
	if (!has_power_) {
		return;  // Simulate power loss - drop all incoming data!
	}

	auto combined_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
	std::vector<sensor_msgs::msg::LaserScan::ConstSharedPtr> scans = {front, back, left, right};

	// Project each LiDAR scan into a 3D point cloud
	// within the target coordinate frame (target_frame_)
	// Automatically account for individual ray timestamps to correct motion distortion
	for (const auto &scan : scans) {
		sensor_msgs::msg::PointCloud2 cloud_out;
		try {
			// In ROS 2, using TimePointZero guarantees fetching the latest TF without blocking.
			// Since the LiDAR is fixed relative to base_link,
			// this TF does not change over time (Static TF).
			// The transformLaserScanToPointCloud function extracts the TF automatically.
			projector_.transformLaserScanToPointCloud(
				target_frame_,  // Using dynamic frame from parameters
				*scan,
				cloud_out,
				*tf_buffer_
			);

			if (combined_cloud->data.empty()) {
				*combined_cloud = cloud_out;
			} else {
				concatenate_pointclouds(*combined_cloud, cloud_out);
			}
		} catch (const std::exception &e) {
			// Use THROTTLE to prevent console spam if the TF tree actually disconnects
			RCLCPP_WARN_THROTTLE(
				this->get_logger(), *this->get_clock(), 1000,
				"Projection error for %s: %s", scan->header.frame_id.c_str(), e.what());
		}
	}

	combined_cloud->header.stamp    = this->get_clock()->now();
	combined_cloud->header.frame_id = target_frame_;  // Assigning dynamic target frame to header

	publisher_->publish(*combined_cloud);
}

void LidarMergerNode::concatenate_pointclouds(
	sensor_msgs::msg::PointCloud2 &base,
	const sensor_msgs::msg::PointCloud2 &add) {
	// Store the current data size
	size_t original_size = base.data.size();

	// Allocate memory for the combined buffer once
	// This prevents unnecessary memory reallocations (realloc)
	base.data.resize(original_size + add.data.size());

	// Perform fast, hardware-level memory block copying
	std::memcpy(
		base.data.data() + original_size,   // Pointer to the end of the existing data
		add.data.data(),                    // Pointer to the start of the new data
		add.data.size()                     // Number of bytes to copy
	);

	base.width      += add.width;
	base.row_step   = base.width * base.point_step;
	base.is_dense   = base.is_dense && add.is_dense;
}
}  // namespace mona_perception

// Register the component within the ROS 2 infrastructure
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mona_perception::LidarMergerNode)
