#ifndef UART_TO_STM32__UART_TO_STM32_HPP_
#define UART_TO_STM32__UART_TO_STM32_HPP_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <serial_comm/serial_comm.h>

namespace uart_to_stm32
{

class UartToStm32
{
public:
  explicit UartToStm32(rclcpp::Node::SharedPtr node);
  ~UartToStm32();

  bool initialize(double update_rate, const std::string & source_frame, const std::string & target_frame,
                  const std::string & serial_port = "/dev/ttyS3", int baud_rate = 921600);

private:
  void lookupTransform();
  void processTfTransform(const geometry_msgs::msg::TransformStamped & transform);
  void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void targetVelocityCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  Eigen::Vector3d transformVelocity(const Eigen::Vector3d & linear, double yaw);
  void sendVelocityToSerial(const Eigen::Vector3d & transformed_velocity);
  void sendTargetVelocityToSerial(float vx_cm_per_s, float vy_cm_per_s, float vz_cm_per_s, float vyaw_deg_per_s);
  void sendA2ReadyResponse();
  void sendThirdWaypointNotifyOnce();
  void sendThirdWaypointNotifyThreeTimes();
  void startContinuousNotifyBurst();
  void simulateFourthWaypointNotify();
  void activeControllerCallback(const std_msgs::msg::UInt8::SharedPtr msg);
  void routeReachedIndexCallback(const std_msgs::msg::UInt8::SharedPtr msg);
  void fcTestTriggerCallback(const std_msgs::msg::UInt8::SharedPtr msg);
  void protocolDataHandler(uint8_t id, const std::vector<uint8_t> & data);
  void publishFcAttitudeQuaternion(const std::vector<uint8_t> & data);
  void publishFcAttitudeEuler(const std::vector<uint8_t> & data);
  void missionCmdCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  double update_rate_;
  std::string source_frame_;
  std::string target_frame_;
  std::string serial_port_ = "/dev/ttyS3";
  int baud_rate_ = 921600;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_velocity_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr bluetooth_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr active_controller_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr route_reached_index_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr fc_test_trigger_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr mission_cmd_sub_;
  rclcpp::TimerBase::SharedPtr continuous_fc_notify_timer_;
  rclcpp::TimerBase::SharedPtr simulate_fourth_waypoint_timer_;

  std::unique_ptr<serial_comm::SerialComm> serial_comm_;

  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr height_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr is_st_ready_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mission_step_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr fc_attitude_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr fc_attitude_euler_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr link_len_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr vel_echo_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr fc_status_pub_;

  void bluetoothCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
  double current_yaw_;
  bool yaw_valid_;
  geometry_msgs::msg::Twist current_velocity_;
  bool velocity_valid_;
  bool has_st_ready_pub_;
  bool third_waypoint_notified_;
  bool continuous_fc_notify_enabled_;
  int continuous_fc_notify_remaining_count_;
  double continuous_fc_notify_period_sec_;
  double continuous_fc_notify_duration_sec_;
  bool simulate_fourth_waypoint_notify_enabled_;

  static constexpr uint8_t VELOCITY_FRAME_ID = 0x32;
  static constexpr uint8_t TARGET_VELOCITY_FRAME_ID = 0x31;
  static constexpr uint8_t THIRD_WAYPOINT_NOTIFY_FRAME_ID = 0x11;
  static constexpr uint8_t ST_READY_QUERY_ID = 0xF1;
  static constexpr uint8_t A2_READY_RESP_ID = 0xA2;
  // 飞控 -> 香橙派 的姿态帧
  static constexpr uint8_t FC_IMU_FRAME_ID = 0x01;    // acc/gyro + 四元数(x10000)，源 5ms
  static constexpr uint8_t FC_EULER_FRAME_ID = 0x03;  // 欧拉角(x100)，源 500ms，仅作交叉校验
  static constexpr uint8_t MISSION_CMD_FRAME_ID = 0x66;
};

}  // namespace uart_to_stm32

#endif  // UART_TO_STM32__UART_TO_STM32_HPP_
