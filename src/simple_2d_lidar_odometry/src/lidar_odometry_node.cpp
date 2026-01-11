#include <cstdio>
#include <memory>
#include <pcl_conversions/pcl_conversions.h>
#include "rclcpp/rclcpp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp" // [추가]
#include "tf2_ros/transform_broadcaster.h"         // [추가]
#include "lidar_odometry/lidar_odometry.hpp"

class LidarOdometryNode : public rclcpp::Node
{
  public:
    LidarOdometryNode() : Node("lidar_odometry_node")
    {
      RCLCPP_INFO(this->get_logger(), "Lidar Odometry Node Started!");

      parameter_initilization();

      double max_correspondence_distance;
      double transformation_epsilon;
      double maximum_iterations;
      std::string scan_topic_name;
      std::string odom_topic_name;

      this->get_parameter("max_correspondence_distance", max_correspondence_distance);
      this->get_parameter("transformation_epsilon", transformation_epsilon);
      this->get_parameter("maximum_iterations", maximum_iterations);
      this->get_parameter("scan_topic_name", scan_topic_name);
      this->get_parameter("odom_topic_name", odom_topic_name);

      RCLCPP_INFO(this->get_logger(), "===== Configuration =====");
      RCLCPP_INFO(this->get_logger(), "scan_topic: %s", scan_topic_name.c_str());
      RCLCPP_INFO(this->get_logger(), "odom_topic: %s", odom_topic_name.c_str());

      lidar_odometry_ptr = std::make_shared<LidarOdometry>(max_correspondence_distance, transformation_epsilon, maximum_iterations);

      odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name, 100);
      
      // [추가] TF 브로드캐스터 초기화
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_name, 1000, std::bind(&LidarOdometryNode::scan_callback, this, std::placeholders::_1)
      );
    }

    private:
      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
      rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber;
      std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; // [추가] TF 브로드캐스터 선언
      std::shared_ptr<LidarOdometry> lidar_odometry_ptr;

      void parameter_initilization() {
        this->declare_parameter<double>("max_correspondence_distance", 1.0);
        this->declare_parameter<double>("transformation_epsilon", 0.005);
        this->declare_parameter<double>("maximum_iterations", 30);
        this->declare_parameter<std::string>("scan_topic_name", "scan");
        this->declare_parameter<std::string>("odom_topic_name", "odom");
      }

      void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        auto point_cloud_msg = laser2cloudmsg(scan_msg);
        auto pcl_point_cloud = cloudmsg2cloud(point_cloud_msg);

        auto scan_data = std::make_shared<ScanData>();
        scan_data->timestamp = scan_msg->header.stamp.sec + scan_msg->header.stamp.nanosec / 1e9;
        scan_data->point_cloud = pcl_point_cloud;

        lidar_odometry_ptr->process_scan_data(scan_data);
        publish_odometry();
      }

      void publish_odometry() {
        auto state = lidar_odometry_ptr->get_state();
        std::string fixed_id = "odom";
        std::string child_id = "base_link"; // [수정] 로봇 베이스 프레임으로 변경

        rclcpp::Time current_time = this->get_clock()->now();

        // 1. Odometry 메시지 발행
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.frame_id = fixed_id;
        odom_msg.child_frame_id = child_id;
        odom_msg.header.stamp = current_time;
        odom_msg.pose.pose = Eigen::toMsg(state->pose);
        odom_msg.twist.twist = Eigen::toMsg(state->velocity);
        odom_publisher->publish(odom_msg);

        // 2. [추가] TF (odom -> base_link) 방송
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = current_time;
        t.header.frame_id = fixed_id;
        t.child_frame_id = child_id;

        // Eigen pose를 TransformStamped로 변환
        t.transform.translation.x = state->pose.translation().x();
        t.transform.translation.y = state->pose.translation().y();
        t.transform.translation.z = 0.0;
        
        Eigen::Quaterniond q(state->pose.rotation());
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(t);
      }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarOdometryNode>());
  rclcpp::shutdown();
  return 0;
}