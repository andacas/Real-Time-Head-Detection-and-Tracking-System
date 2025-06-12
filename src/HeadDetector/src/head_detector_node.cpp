/******************************************************************************
 *  head_detector_node.cpp  –  ROS 2 node that runs a GPU-accelerated
 *                             3-D Haar-cascade head detector in real-time.
 *
 *  ▸ Subscribes to a PointCloud2 stream (default “cloud_concatenated”).
 *  ▸ Transforms every cloud into a fixed frame (default “world”) using TF2.
 *  ▸ **Keeps only points whose XY-radius ≤ max_distance AND Z ≥ min_z.**
 *  ▸ Runs the detector on the GPU every Nth frame for throughput.
 *  ▸ Publishes a semi-transparent CUBE marker at the best head pose.
 *
 *  Parameters (CLI / YAML):
 *      cloud_topic   [cloud_concatenated] – PointCloud2 input topic
 *      target_frame  [world]              – TF frame for processing/output
 *      detect_every  [5]                  – run detector every N frames
 *      tf_timeout    [0.05]               – TF lookup timeout (s)
 *      max_distance  [1.5]                – XY-plane radius filter (m)
 *      min_z         [1.0]                – discard points lower than this (m)
 *
 *  Build : colcon build --packages-select head_detector
 *  Run   : ros2 run head_detector head_detector_node \
 *              --ros-args -p min_z:=1.2 -p max_distance:=2.0
 ******************************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tf2/LinearMath/Quaternion.h>

/* ─── tf2_sensor_msgs header moved from .h → .hpp in Jazzy ─────────────── */
#if __has_include(<tf2_sensor_msgs/tf2_sensor_msgs.hpp>)
  #include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#elif __has_include(<tf2_sensor_msgs/tf2_sensor_msgs.h>)
  #include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#else
  #error "Cannot find <tf2_sensor_msgs/tf2_sensor_msgs.{h,hpp}> – "\
         "please install ros-${ROS_DISTRO}-tf2-sensor-msgs"
#endif

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

#include "detection/Detector.h"   // GPU Haar cascade

using std::placeholders::_1;

/* ────────────────────────────────────────────────────────────────────────── */
class HeadDetectorNode : public rclcpp::Node
{
public:
    HeadDetectorNode();

private:
    /* callback ---------------------------------------------------------- */
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    /* I/O --------------------------------------------------------------- */
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr  pub_;

    /* TF --------------------------------------------------------------- */
    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    /* detector + state -------------------------------------------------- */
    detection::Detector detector_;
    std::size_t frame_count_{0};

    /* parameters -------------------------------------------------------- */
    std::string cloud_topic_;
    std::string target_frame_;
    int         detect_every_;
    double      tf_timeout_;          // seconds
    double      max_distance_;        // metres (XY radius)
    double      max_distance_sq_;     // metres² (pre-squared for speed)
    double      min_z_;               // metres (reject points below)
};

/* ======================================================================= */
HeadDetectorNode::HeadDetectorNode()
: Node("head_detector_node")
{
    /* ── declare parameters ──────────────────────────────────────────── */
    declare_parameter<std::string>("cloud_topic",  "cloud_concatenated");
    declare_parameter<std::string>("target_frame", "world");
    declare_parameter<int>("detect_every",         1);
    declare_parameter<double>("tf_timeout",        0.05);
    declare_parameter<double>("max_distance",      2.5);  // metres (XY)
    declare_parameter<double>("min_z",             1.4);  // metres (Z)

    /* ── fetch parameters ────────────────────────────────────────────── */
    get_parameter("cloud_topic",   cloud_topic_);
    get_parameter("target_frame",  target_frame_);
    get_parameter("detect_every",  detect_every_);
    get_parameter("tf_timeout",    tf_timeout_);
    get_parameter("max_distance",  max_distance_);
    get_parameter("min_z",         min_z_);

    max_distance_sq_ = max_distance_ * max_distance_;

    /* ── TF listener ─────────────────────────────────────────────────── */
    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    /* ── subscriptions & publishers ──────────────────────────────────── */
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, rclcpp::SensorDataQoS(),
        std::bind(&HeadDetectorNode::cloudCallback, this, _1));

    auto qos = rclcpp::QoS(10).reliable().transient_local();
    pub_ = create_publisher<visualization_msgs::msg::Marker>("head_box", qos);

    /* ── startup log ─────────────────────────────────────────────────── */
    RCLCPP_INFO(get_logger(),
        "HeadDetectorNode ready — input: \"%s\"  target_frame: \"%s\""
        "  detect_every: %d  tf_timeout: %.3f s"
        "  max_distance(XY): %.2f m  min_z: %.2f m  (GPU=%d)",
        cloud_topic_.c_str(), target_frame_.c_str(),
        detect_every_, tf_timeout_, max_distance_, min_z_, HAVE_GPU);
}

/* ======================================================================= */
void HeadDetectorNode::cloudCallback(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    /***** frame skipping *************************************************/
    if (frame_count_ % detect_every_ != 0) { ++frame_count_; return; }
    ++frame_count_;

    /***** TF transform ***************************************************/
    sensor_msgs::msg::PointCloud2 cloud_tf;
    try {
        const auto tf = tf_buffer_->lookupTransform(
            target_frame_,                       // to
            msg->header.frame_id,                // from
            msg->header.stamp,                   // data time
            tf2::durationFromSec(tf_timeout_));  // timeout

        tf2::doTransform(*msg, cloud_tf, tf);
    }
    catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "TF lookup failed: %s", ex.what());
        return;
    }

    /***** ROS → PCL *****************************************************/
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(cloud_tf, *cloud_raw);

    /***** XY-radius + Z filter ******************************************/
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->reserve(cloud_raw->size());

    for (const auto& p : cloud_raw->points)
    {
        const double xy2 = static_cast<double>(p.x) * p.x +
                           static_cast<double>(p.y) * p.y;

        if (xy2 <= max_distance_sq_ &&
            p.z  >= min_z_ &&
            std::isfinite(xy2) && std::isfinite(p.z))
        {
            cloud->push_back(p);
        }
    }

    if (cloud->empty()) return;   // nothing in range after filtering

    /***** GPU cascade ***************************************************/
    const auto heads = detector_.detectHeads<pcl::PointXYZ>(cloud);
    if (heads.empty()) return;

    const detection::HeadPose& h = heads.front();   // highest confidence

    /***** RViz marker ***************************************************/
    visualization_msgs::msg::Marker m;
    m.header.stamp    = cloud_tf.header.stamp;
    m.header.frame_id = target_frame_;
    m.ns     = "head_detector";
    m.id     = 0;
    m.type   = visualization_msgs::msg::Marker::CUBE;
    m.action = visualization_msgs::msg::Marker::ADD;

    m.pose.position.x = h.x;
    m.pose.position.y = h.y;
    m.pose.position.z = h.z;

    tf2::Quaternion q;
    q.setRPY(h.roll, h.pitch, h.yaw);
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    m.pose.orientation.w = q.w();

    m.scale.x = m.scale.y = m.scale.z = 0.25f;   // 25 cm cube
    m.color.r = 1.0f;  m.color.a = 0.6f;         // translucent red
    m.lifetime = rclcpp::Duration::from_seconds(0.3);

    pub_->publish(m);
}

/* ======================================================================= */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HeadDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
