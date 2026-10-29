#ifndef UART_TO_STM32__UART_TO_STM32_HPP_
#define UART_TO_STM32__UART_TO_STM32_HPP_

#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include "uart_to_stm32/height_diag_codec.h"
#include "uart_to_stm32/laser_height_filter.h"
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
  // ★ 2026-10-18：下行报文 200Hz 原样转发，8 个 Python 节点都在 200Hz 跑回调，
  // 把 8 核吃满。这里把"非 IMU"的几路做 4:1 抽取（200 -> 50Hz）。
  // IMU（陀螺/加计/姿态）保持 200Hz，cartographer 需要。
  int decim_{4};
  int decim_height_{0};
  LaserHeightFilter laser_height_filter_;
  int decim_vel_{0};
  int decim_pwm_{0};
  int decim_tgt_{0};

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
  void publishFcGyro(const std::vector<uint8_t> & data);
  void publishFcAcc(const std::vector<uint8_t> & data);
  void publishFcTarget(const std::vector<uint8_t> & data);
  void publishFcVelocity(const std::vector<uint8_t> & data);   // 0x07 飞控自己的速度估计
  void publishFcPwm(const std::vector<uint8_t> & data);        // 0x20 PWM 控制量（0.01%）
  void publishFcTargetAtti(const std::vector<uint8_t> & data); // 0x0A 目标姿态（×0.01 度）
  void publishFcTargetVel(const std::vector<uint8_t> & data);  // 0x0B 目标速度（cm/s）
  void publishFcCtrlAmt(const std::vector<uint8_t> & data);    // 0x21 姿态控制量（±5000）
  void publishFcVelFwd(const std::vector<uint8_t> & data);     // 0x0C 转发验证帧（内容=0x07）
  void publishFcMag(const std::vector<uint8_t> & data);        // ★2026-10-16 0x02 罗盘/气压/温度（磁干扰监测）
  void missionCmdCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr a2_ready_timer_;
  unsigned a2_ready_remaining_ = 0;
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
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr height_diag_pub_;
  rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr range_input_diag_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr is_st_ready_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mission_step_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr fc_attitude_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr fc_attitude_euler_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr fc_gyro_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr fc_acc_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr fc_target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr fc_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr fc_pwm_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr fc_target_atti_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr fc_target_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr fc_ctrl_amt_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr fc_vel_fwd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr fc_mag_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr fc_mag_raw_pub_;   // 调试：0x02 原始 14 字节
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr link_len_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr vel_echo_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr fc_status_pub_;

  void bluetoothCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
  double current_yaw_;
  bool yaw_valid_;
  // ★2026-10-17：融合航向（heading_fuse 发布，单位度）。雷达航向在起点会跳 +58°
  //   （实测两次，陀螺纹丝未动），用它替 tf 航向做速度旋转，避免注入速度方向整体错。
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr heading_fused_sub_;
  double fused_yaw_ = 0.0;
  bool fused_yaw_valid_ = false;
  std::chrono::steady_clock::time_point fused_yaw_t_{};
  geometry_msgs::msg::Twist current_velocity_;
  bool velocity_valid_;
  bool has_st_ready_pub_;
  bool third_waypoint_notified_;
  bool continuous_fc_notify_enabled_;
  int continuous_fc_notify_remaining_count_;
  double continuous_fc_notify_period_sec_;
  double continuous_fc_notify_duration_sec_;
  bool simulate_fourth_waypoint_notify_enabled_;

  // ★2026-10-09 实测：协议文档写 0x32=通用位置型(S32×3,cm,LEN=12)、0x33=通用速度型(S16×3,cm/s,LEN=6)，
  //   本想改成 0x33 —— 但**实测改之前飞控的"通用速度"指示灯就是绿的**（白=未接入/红=无效/绿=有效），
  //   说明 LX 固件确实按 6 字节速度解析 0x32。**保持 0x32，不要动。**
  static constexpr uint8_t VELOCITY_FRAME_ID = 0x32;
  static constexpr uint8_t TARGET_VELOCITY_FRAME_ID = 0x31;
  static constexpr uint8_t THIRD_WAYPOINT_NOTIFY_FRAME_ID = 0x11;
  static constexpr uint8_t ST_READY_QUERY_ID = 0xF1;
  static constexpr uint8_t A2_READY_RESP_ID = 0xA2;
  // 飞控 -> 香橙派 的姿态帧
  static constexpr uint8_t FC_IMU_FRAME_ID = 0x01;    // acc/gyro + 四元数(x10000)，源 5ms
  static constexpr uint8_t FC_EULER_FRAME_ID = 0x03;  // 欧拉角(x100)，源 500ms，仅作交叉校验
  static constexpr uint8_t FC_CTRL_FRAME_ID = 0x41;   // 飞控控制目标 rol/pit/thr/yaw_dps/vel
  // ★2026-10-09 新加两帧（407 侧 AnoDTRasp.c 同步新增）
  static constexpr uint8_t FC_VEL_FRAME_ID = 0x07;    // 飞控自己的速度估计（S16×3, cm/s）
  // ★2026-10-10 第二批：407 原样转发 IMU 的这几帧（ID 与协议一致）
  // ★2026-10-10 实测：同一条已验证的 5ms 发送通道，ID 用 0x20 收不到、0x09 能收到
  //   → 香橙派解析层对 ≥0x20 的 ID 有过滤。所以 PWM 仍用 0x09（内容是真的 4 路 PWM，0.01%）。
  static constexpr uint8_t FC_PWM_FRAME_ID = 0x09;    // PWM 控制量（U16×4, 0.01%油门）
  static constexpr uint8_t FC_TGT_ATTI_FRAME_ID = 0x0A;  // 目标姿态（S16×3, ×0.01度）
  static constexpr uint8_t FC_TGT_VEL_FRAME_ID = 0x0B;   // 目标速度（S16×3, cm/s）
  static constexpr uint8_t FC_CTRL_AMT_FRAME_ID = 0x21;  // 姿态控制量（S16×4: ROL/PIT/THR/YAW）
  // ★2026-10-10 验证用：407 把 0x07（飞控速度，已验证 IMU 会发）用转发函数发到 0x0C。
  //   收到 0x0C = 转发函数有效 → 那三帧确实是飞控没发；收不到 = 转发函数有 bug。
  static constexpr uint8_t FC_VEL_FWD_FRAME_ID = 0x0C;
  // ★2026-10-16：IMU 的 0x02 帧（罗盘/气压/温度，14B），407 原样转发。
  //   用途：实时监测磁干扰 —— 铁柱/货架金属拉偏飞控罗盘会让航向基准跑偏，
  //   偏航环持续输出差速，飞机自转/漂移（当天实测：铁柱旁 err 25~56° 撞墙，
  //   同一飞机在空地 err 1~8°）。
  static constexpr uint8_t FC_MAG_FRAME_ID = 0x02;
  static constexpr uint8_t MISSION_CMD_FRAME_ID = 0x66;
};

}  // namespace uart_to_stm32

#endif  // UART_TO_STM32__UART_TO_STM32_HPP_
