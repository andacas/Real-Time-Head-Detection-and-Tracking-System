#include "pointcloud_concatenate/pointcloud_concatenate.hpp"
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>

PointcloudConcatenate::PointcloudConcatenate()
: Node("pointcloud_concatenate")
{
  handleParams();

  // TF2
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Subscribers
  sub_cloud_in1_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud_in1", rclcpp::QoS(1),
    std::bind(&PointcloudConcatenate::subCallbackCloudIn1, this, std::placeholders::_1));
  sub_cloud_in2_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud_in2", rclcpp::QoS(1),
    std::bind(&PointcloudConcatenate::subCallbackCloudIn2, this, std::placeholders::_1));
  sub_cloud_in3_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud_in3", rclcpp::QoS(1),
    std::bind(&PointcloudConcatenate::subCallbackCloudIn3, this, std::placeholders::_1));
  sub_cloud_in4_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud_in4", rclcpp::QoS(1),
    std::bind(&PointcloudConcatenate::subCallbackCloudIn4, this, std::placeholders::_1));

  // Publisher
  pub_cloud_out_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "cloud_out", rclcpp::QoS(1));

  // Timer
  timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / hz_)),
    std::bind(&PointcloudConcatenate::update, this));
}

void PointcloudConcatenate::handleParams()
{
  declare_parameter<std::string>("target_frame", "base_link");
  get_parameter("target_frame", target_frame_);

  declare_parameter<int>("clouds", 2);
  get_parameter("clouds", clouds_);

  declare_parameter<double>("hz", 10.0);
  get_parameter("hz", hz_);

  RCLCPP_INFO(get_logger(),
    "Params — target_frame: %s, clouds: %d, hz: %.1f",
    target_frame_.c_str(), clouds_, hz_);
}

void PointcloudConcatenate::subCallbackCloudIn1(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  cloud_in1_ = *msg;
  cloud_in1_received_ = true;
  cloud_in1_received_recent_ = true;
}

void PointcloudConcatenate::subCallbackCloudIn2(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  cloud_in2_ = *msg;
  cloud_in2_received_ = true;
  cloud_in2_received_recent_ = true;
}

void PointcloudConcatenate::subCallbackCloudIn3(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  cloud_in3_ = *msg;
  cloud_in3_received_ = true;
  cloud_in3_received_recent_ = true;
}

void PointcloudConcatenate::subCallbackCloudIn4(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  cloud_in4_ = *msg;
  cloud_in4_received_ = true;
  cloud_in4_received_recent_ = true;
}

void PointcloudConcatenate::update()
{
  if (pub_cloud_out_->get_subscription_count() == 0 || clouds_ < 1) {
    return;
  }

  // Wait for first message
  if (!cloud_in1_received_ && !cloud_in2_received_ &&
      !cloud_in3_received_ && !cloud_in4_received_) {
    RCLCPP_WARN(get_logger(), "No pointclouds received yet");
    return;
  }

  // Start with cloud1
  sensor_msgs::msg::PointCloud2 output = cloud_in1_;
  if (!cloud_in1_received_recent_) {
    RCLCPP_WARN(get_logger(), "Reusing last cloud1");
  }
  cloud_in1_received_recent_ = false;
  try {
    pcl_ros::transformPointCloud(target_frame_, cloud_in1_, output, *tf_buffer_);
  } catch (...) {
    RCLCPP_WARN(get_logger(), "Transform cloud1 failed");
    return;
  }

  // Helper to process and concat clouds 2-4
  auto process = [&](const sensor_msgs::msg::PointCloud2 &in,
                     bool received, bool &recent) {
    if (!received) return;
    if (!recent) {
      RCLCPP_WARN(get_logger(), "Reusing last cloud");
    }
    recent = false;
    sensor_msgs::msg::PointCloud2 tmp;
    pcl_ros::transformPointCloud(target_frame_, in, tmp, *tf_buffer_);
    pcl::concatenatePointCloud(output, tmp, output);
  };

  if (clouds_ >= 2) process(cloud_in2_, cloud_in2_received_, cloud_in2_received_recent_);
  if (clouds_ >= 3) process(cloud_in3_, cloud_in3_received_, cloud_in3_received_recent_);
  if (clouds_ >= 4) process(cloud_in4_, cloud_in4_received_, cloud_in4_received_recent_);

  // Publish
  output.header.stamp = now();
  pub_cloud_out_->publish(output);
}

void PointcloudConcatenate::publishPointcloud(sensor_msgs::msg::PointCloud2 &cloud)
{
  // (unused since we publish directly in update())
}
