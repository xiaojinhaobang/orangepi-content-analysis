#pragma once

#include <mutex>
#include <string>
#include <vector>
#include <deque>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace activity_control_pkg
{

struct Target
{
  double x_cm;
  double y_cm;
  double z_cm;
  double yaw_deg;
};

class RouteTargetPublisherNode : public rclcpp::Node
{
public:
  explicit RouteTargetPublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void addTarget(const Target & target);
  bool replaceTarget(std::size_t index, const Target & target);
  bool queryCurrentPose(double & x_cm, double & y_cm, double & z_cm, double & yaw_deg);
  bool publishHoldTarget(const Target & target);
  bool skipCurrentTarget();

  std::size_t currentIndex() const;

  std::size_t size() const;

private:
  void publishCurrent();
  void publishTarget(const Target & target, bool init_flag);

  bool getCurrentPose(double & x_cm, double & y_cm, double & z_cm, double & yaw_deg);
  bool isReached(const Target & target, double x_cm, double y_cm, double z_cm, double yaw_deg) const;

  void monitorTimerCallback();
  void heightCallback(const std_msgs::msg::Int16::SharedPtr msg);
  void bluetooth_data_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
  
  static double meterToCm(double value_m);
  static double radToDeg(double value_rad);
  double normalizeAngleDeg(double angle_deg) const;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr active_controller_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr reached_waypoint_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr current_waypoint_pub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr height_sub_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr bluetooth_sub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


  mutable std::mutex mutex_;
  std::vector<Target> targets_;
  std::size_t current_idx_;

  bool has_height_;
  double current_height_cm_;

  double pos_tol_cm_;
  double yaw_tol_deg_;
  double height_tol_cm_;


  std::string map_frame_;
  std::string laser_link_frame_;
  std::string output_topic_;

  uint8_t control_mode_ = 0;
};

class RouteTestNode : public rclcpp::Node
{
public:
  explicit RouteTestNode(const std::shared_ptr<RouteTargetPublisherNode> & route_node,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void bluetoothCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
  void addTimerCallback();
  bool computeAverageXY(double & x_cm, double & y_cm) const;

  std::shared_ptr<RouteTargetPublisherNode> route_node_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr bluetooth_sub_;
  rclcpp::TimerBase::SharedPtr add_timer_;

  bool started_;
  int next_target_index_;

  bool use_two_second_capture_mode_;
  bool two_second_capture_done_;
  bool capture_stop_issued_;
  int elapsed_ticks_;
  int first_drive_ticks_;
  int add_interval_ticks_;
  int capture_settle_ticks_;
  int capture_settle_elapsed_ticks_;

  double first_forward_x_cm_;
  double first_forward_y_cm_;
  double first_forward_z_cm_;
  double takeoff_height_cm_;

  std::deque<double> recent_x_cm_;
  std::deque<double> recent_y_cm_;
  double capture_last_z_cm_;
  double capture_last_yaw_deg_;
  std::size_t capture_window_size_;
};

}  // namespace activity_control_pkg
