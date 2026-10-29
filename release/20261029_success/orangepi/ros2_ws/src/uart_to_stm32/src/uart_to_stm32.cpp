#include "uart_to_stm32/uart_to_stm32.hpp"
#include <iostream>
  
#include <chrono>
#include <cmath>
#include <utility>

#include <tf2/exceptions.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace uart_to_stm32
{

using namespace std::chrono_literals;

UartToStm32::UartToStm32(rclcpp::Node::SharedPtr node)
: node_(std::move(node)),
  update_rate_(0.0),
  current_yaw_(0.0),
  yaw_valid_(false),
  velocity_valid_(false),
  has_st_ready_pub_(false),
  third_waypoint_notified_(false)
{
  RCLCPP_INFO(node_->get_logger(), "UartToStm32 created");
}

UartToStm32::~UartToStm32()
{
  if (timer_) {
    timer_->cancel();
  }
  if (target_velocity_heartbeat_timer_) {
    target_velocity_heartbeat_timer_->cancel();
  }
  if (serial_comm_) {
    serial_comm_->stop_protocol_receive();
    serial_comm_->close();
  }
}

bool UartToStm32::initialize(double update_rate, const std::string & source_frame,
                             const std::string & target_frame, const std::string & serial_port,
                             int baud_rate)
{
  try {
    update_rate_ = update_rate;
    source_frame_ = source_frame;
    target_frame_ = target_frame;
    serial_port_ = serial_port;
    baud_rate_ = baud_rate;

    RCLCPP_INFO(node_->get_logger(), "UartToStm32 initialized with update rate: %.1f Hz", update_rate_);
    RCLCPP_INFO(node_->get_logger(), "Looking for transform from '%s' to '%s'", source_frame_.c_str(), target_frame_.c_str());
    RCLCPP_INFO(node_->get_logger(), "Using serial port '%s' at %d baudrate",
                serial_port_.c_str(), baud_rate_);

    serial_comm_ = std::make_unique<serial_comm::SerialComm>();
    if (!serial_comm_->initialize(serial_port_, static_cast<unsigned int>(baud_rate_))) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to initialize serial port %s at %d baudrate",
                   serial_port_.c_str(), baud_rate_);
      RCLCPP_ERROR(node_->get_logger(), "Serial error: %s", serial_comm_->get_last_error().c_str());
      return false;
    }
    RCLCPP_INFO(node_->get_logger(), "Serial port %s initialized at %d baudrate",
                serial_port_.c_str(), baud_rate_);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto period = std::chrono::duration<double>(1.0 / update_rate_);
    timer_ = node_->create_wall_timer(
      period,
      std::bind(&UartToStm32::lookupTransform, this));

    // ★ 2026-10-09：喂给飞控的"速度观测"用哪个话题，做成参数，方便 A/B 对比：
    //   /velocity_map    = 雷达原始（实测滞后 IMU 约 200ms，定点会拖后腿）
    //   /velocity_fused  = IMU 延迟补偿后的（见 imu_fusion.py）
    //   切法: 在 flight_link_run.sh 的 launch 里加 vel_topic:=/velocity_fused
    const std::string vel_topic =
      node_->declare_parameter<std::string>("vel_topic", "/velocity_map");
    velocity_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      vel_topic, 10,
      std::bind(&UartToStm32::velocityCallback, this, std::placeholders::_1));
    RCLCPP_INFO(node_->get_logger(), "velocity observation topic: %s", vel_topic.c_str());

    target_velocity_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/target_velocity", 10,
      std::bind(&UartToStm32::targetVelocityCallback, this, std::placeholders::_1));
    // 2026-10-24：飞控要求 0x31 持续到达。mission 在 pos_hold -> waypoint_nav
    // 交棒时会短暂没有 ROS 发布者；这里由串口节点维护最终保活，超过 150ms
    // 没有新命令就以 20Hz 直接补发零速帧，不让底层协议静默。
    target_velocity_heartbeat_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&UartToStm32::targetVelocityHeartbeat, this));

    active_controller_sub_ = node_->create_subscription<std_msgs::msg::UInt8>(
      "/active_controller", 10,
      std::bind(&UartToStm32::activeControllerCallback, this, std::placeholders::_1));

    route_reached_index_sub_ = node_->create_subscription<std_msgs::msg::UInt8>(
      "/route_reached_index", 10,
      std::bind(&UartToStm32::routeReachedIndexCallback, this, std::placeholders::_1));

    fc_test_trigger_sub_ = node_->create_subscription<std_msgs::msg::UInt8>(
      "/fc_test_trigger", 10,
      std::bind(&UartToStm32::fcTestTriggerCallback, this, std::placeholders::_1));

    // ★2026-10-17：融合航向（陀螺为主 + 雷达慢速校准，heading_fuse 发布，单位度）。
    //   起点实测起飞后 1.5s 雷达航向 1 秒内跳 +58°（陀螺纹丝未动），而速度旋转用的是
    //   tf(=雷达) 航向 → 注入飞控的观测速度方向整体错 58° → 飞控定点环被拖歪。
    heading_fused_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
      "/heading_fused", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        fused_yaw_ = msg->data * M_PI / 180.0;      // 度 → 弧度
        fused_yaw_valid_ = true;
        fused_yaw_t_ = std::chrono::steady_clock::now();
      });

    mission_cmd_sub_ = node_->create_subscription<std_msgs::msg::Int16MultiArray>(
      "/mission_cmd", 10,
      std::bind(&UartToStm32::missionCmdCallback, this, std::placeholders::_1));

    // bluetooth_sub_ = node_->create_subscription<std_msgs::msg::UInt8MultiArray>(
    //   "/bluetooth_data", 10,
    //   std::bind(&UartToStm32::bluetoothCallback, this, std::placeholders::_1));

    height_pub_ = node_->create_publisher<std_msgs::msg::Int16>("/height", 10);
    height_diag_pub_ = node_->create_publisher<std_msgs::msg::Int32MultiArray>("/fc_height_diag", 10);
    range_input_diag_pub_ = node_->create_publisher<std_msgs::msg::Int64MultiArray>("/fc_range_input_diag", 10);
    is_st_ready_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/is_st_ready", rclcpp::QoS(10).transient_local());
    mission_step_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/mission_step", 10);
    fc_attitude_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("/fc_attitude", 10);
    fc_attitude_euler_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("/fc_attitude_euler", 10);
    fc_gyro_pub_ = node_->create_publisher<geometry_msgs::msg::Vector3>("/fc_gyro", 10);
  fc_acc_pub_ = node_->create_publisher<geometry_msgs::msg::Vector3>("/fc_acc", 10);
  fc_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Vector3>("/fc_vel", 10);
  fc_pwm_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("/fc_pwm", 10);
  fc_target_atti_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("/fc_target_atti", 10);
  fc_target_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Vector3>("/fc_target_vel", 10);
  fc_ctrl_amt_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("/fc_ctrl_amt", 10);
  fc_vel_fwd_pub_ = node_->create_publisher<geometry_msgs::msg::Vector3>("/fc_vel_fwd", 10);
  // ★2026-10-16：罗盘/气压/温度 —— 用于磁干扰监测（[mx,my,mz,mag_sta,bar_sta,tmp,alt_bar]）
  fc_mag_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("/fc_mag", 10);
  fc_mag_raw_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("/fc_mag_raw", 10);
  fc_diag_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("/fc_diag", 10);
    fc_target_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("/fc_target", 10);
    link_len_pub_ = node_->create_publisher<std_msgs::msg::Int16>("/fc_link_len", 10);
    vel_echo_pub_ = node_->create_publisher<std_msgs::msg::Int16MultiArray>("/fc_vel_echo", 10);
    fc_status_pub_ = node_->create_publisher<std_msgs::msg::UInt8MultiArray>("/fc_status", 10);

    has_st_ready_pub_ = false;

    serial_comm_->start_protocol_receive(
      [this](uint8_t id, const std::vector<uint8_t> & data) { protocolDataHandler(id, data); },
      [this](const std::string & err) {
        RCLCPP_WARN(node_->get_logger(), "Serial protocol error: %s", err.c_str());
      });

    RCLCPP_INFO(node_->get_logger(), "UartToStm32 initialized successfully");
    RCLCPP_INFO(node_->get_logger(),
      "Subscribed to /velocity_map, /target_velocity, /route_reached_index, /fc_test_trigger topics");
    return true;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to initialize topic subscriber: %s", e.what());
    return false;
  }
}

void UartToStm32::lookupTransform()
{
  try {
    const auto transform = tf_buffer_->lookupTransform(
      source_frame_, target_frame_, tf2::TimePointZero);
    processTfTransform(transform);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG(node_->get_logger(), "Transform lookup failed: %s", ex.what());
  }
}

void UartToStm32::processTfTransform(const geometry_msgs::msg::TransformStamped & transform)
{
  const double x = transform.transform.translation.x;
  const double y = transform.transform.translation.y;
  const double z = transform.transform.translation.z;

  const double qx = transform.transform.rotation.x;
  const double qy = transform.transform.rotation.y;
  const double qz = transform.transform.rotation.z;
  const double qw = transform.transform.rotation.w;

  tf2::Quaternion q(qx, qy, qz, qw);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  current_yaw_ = yaw;
  yaw_valid_ = true;

  RCLCPP_INFO_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 2000,
    "Transform %s -> %s: pos(%.3f, %.3f, %.3f) rot(%.3f, %.3f, %.3f)",
    source_frame_.c_str(), target_frame_.c_str(), x, y, z, roll, pitch, yaw);

  if (velocity_valid_ && yaw_valid_) {
    Eigen::Vector3d linear_vel(
      current_velocity_.linear.x,
      current_velocity_.linear.y,
      current_velocity_.linear.z);
    const Eigen::Vector3d transformed_vel = transformVelocity(linear_vel, current_yaw_);
    sendVelocityToSerial(transformed_vel);
  }
}

void UartToStm32::velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  current_velocity_ = *msg;
  velocity_valid_ = true;

  const double linear_x = msg->linear.x;
  const double linear_y = msg->linear.y;
  const double linear_z = msg->linear.z;
  const double angular_x = msg->angular.x;
  const double angular_y = msg->angular.y;
  const double angular_z = msg->angular.z;

  RCLCPP_INFO_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 2000,
    "Velocity: linear(%.3f, %.3f, %.3f) angular(%.3f, %.3f, %.3f)",
    linear_x, linear_y, linear_z, angular_x, angular_y, angular_z);

  if (yaw_valid_ && velocity_valid_) {
    Eigen::Vector3d linear_vel(linear_x, linear_y, linear_z);
    const Eigen::Vector3d transformed_vel = transformVelocity(linear_vel, current_yaw_);
    sendVelocityToSerial(transformed_vel);
  }
}

// void UartToStm32::bluetoothCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
// {
//   if(msg->data.empty()) {
//     RCLCPP_WARN(node_->get_logger(), "Received empty bluetooth_data message.");
//     return;
//   }
//   constexpr uint8_t BLUETOOTH_FRAME_ID = 0x34;
//   if(serial_comm_ && serial_comm_->is_open()) {
//     if(serial_comm_->send_protocol_data(BLUETOOTH_FRAME_ID, static_cast<uint8_t>(msg->data.size()), msg->data)) {
//       RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
//         "Sent bluetooth data %d",msg->data[0]);
//     } else {
//       RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
//         "Failed to send bluetooth data: %s", serial_comm_->get_last_error().c_str());
//     }  } 
//     else {
//       RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
//         "Serial port is not open, cannot send bluetooth data");
//     }
// }

Eigen::Vector3d UartToStm32::transformVelocity(const Eigen::Vector3d & linear, double yaw)
{
  // ★2026-10-17：优先用融合航向；过期（>0.5s）或未就绪则回退 tf 航向
  const auto age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - fused_yaw_t_).count();
  if (fused_yaw_valid_ && age < 0.5) {
    yaw = fused_yaw_;
  }

  Eigen::Matrix3d Rz;
  Rz << std::cos(yaw), std::sin(yaw), 0.0,
    -std::sin(yaw), std::cos(yaw), 0.0,
    0.0, 0.0, 1.0;

  const Eigen::Vector3d transformed = Rz * linear;

  RCLCPP_INFO_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 2000,
    "Velocity transform: yaw=%.3f deg, original(%.3f,%.3f,%.3f) -> transformed(%.3f,%.3f,%.3f)",
    yaw * 180.0 / M_PI,
    linear.x(), linear.y(), linear.z(),
    transformed.x(), transformed.y(), transformed.z());

  return transformed;
}

void UartToStm32::sendVelocityToSerial(const Eigen::Vector3d & transformed_velocity)
{
  if (!serial_comm_ || !serial_comm_->is_open()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "Serial port is not open, cannot send velocity data");
    return;
  }

  try {
    constexpr double scale_factor = 100.0;

    const int16_t vel_x = static_cast<int16_t>(transformed_velocity.x() * scale_factor);
    const int16_t vel_y = static_cast<int16_t>(transformed_velocity.y() * scale_factor);
    const int16_t vel_z = static_cast<int16_t>(transformed_velocity.z() * scale_factor);

    std::vector<uint8_t> data(6);
    data[0] = static_cast<uint8_t>(vel_x & 0xFF);
    data[1] = static_cast<uint8_t>((vel_x >> 8) & 0xFF);
    data[2] = static_cast<uint8_t>(vel_y & 0xFF);
    data[3] = static_cast<uint8_t>((vel_y >> 8) & 0xFF);
    data[4] = static_cast<uint8_t>(vel_z & 0xFF);
    data[5] = static_cast<uint8_t>((vel_z >> 8) & 0xFF);

    if (serial_comm_->send_protocol_data(VELOCITY_FRAME_ID, static_cast<uint8_t>(data.size()), data)) {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "Sent velocity data: x=%d, y=%d, z=%d (cm/s)", vel_x, vel_y, vel_z);
    } else {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
        "Failed to send velocity data: %s", serial_comm_->get_last_error().c_str());
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Exception in sendVelocityToSerial: %s", e.what());
  }
}

void UartToStm32::targetVelocityCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 4) {
    RCLCPP_WARN(node_->get_logger(),
      "Target velocity message should contain 4 float values [vx_cm/s, vy_cm/s, vz_cm/s, vyaw_deg/s]");
    return;
  }

  const float vx_cm_per_s = msg->data[0];
  const float vy_cm_per_s = msg->data[1];
  const float vz_cm_per_s = msg->data[2];
  const float vyaw_deg_per_s = msg->data[3];
  last_target_velocity_rx_ = std::chrono::steady_clock::now();
  if (target_velocity_heartbeat_active_) {
    RCLCPP_INFO(node_->get_logger(),
      "target_velocity command stream recovered; zero-velocity heartbeat stopped");
    target_velocity_heartbeat_active_ = false;
  }

  RCLCPP_INFO_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 1000,
    "Target Velocity: linear(%.1f, %.1f, %.1f)cm/s angular(%.1f)deg/s",
    vx_cm_per_s, vy_cm_per_s, vz_cm_per_s, vyaw_deg_per_s);

  sendTargetVelocityToSerial(vx_cm_per_s, vy_cm_per_s, vz_cm_per_s, vyaw_deg_per_s);
}

void UartToStm32::targetVelocityHeartbeat()
{
  if (!serial_comm_ || !serial_comm_->is_open()) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  const bool received_once =
    last_target_velocity_rx_.time_since_epoch().count() != 0;
  const bool stale = !received_once ||
    now - last_target_velocity_rx_ >= std::chrono::milliseconds(150);
  if (!stale) {
    return;
  }

  if (!target_velocity_heartbeat_active_) {
    RCLCPP_WARN(node_->get_logger(),
      "target_velocity stale for >=150ms; sending 0x31 zero-velocity heartbeat at 20Hz");
    target_velocity_heartbeat_active_ = true;
  }
  sendTargetVelocityToSerial(0.0f, 0.0f, 0.0f, 0.0f);
}

void UartToStm32::sendTargetVelocityToSerial(float vx_cm_per_s, float vy_cm_per_s, float vz_cm_per_s, float vyaw_deg_per_s)
{
  if (!serial_comm_ || !serial_comm_->is_open()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "Serial port is not open, cannot send target velocity data");
    return;
  }

  try {
    const int16_t vel_x = static_cast<int16_t>(std::lround(vx_cm_per_s));
    const int16_t vel_y = static_cast<int16_t>(std::lround(vy_cm_per_s));
    const int16_t vel_z = static_cast<int16_t>(std::lround(vz_cm_per_s));
    const int16_t vel_yaw = static_cast<int16_t>(std::lround(vyaw_deg_per_s));

    std::vector<uint8_t> data(8);
    data[0] = static_cast<uint8_t>(vel_x & 0xFF);
    data[1] = static_cast<uint8_t>((vel_x >> 8) & 0xFF);
    data[2] = static_cast<uint8_t>(vel_y & 0xFF);
    data[3] = static_cast<uint8_t>((vel_y >> 8) & 0xFF);
    data[4] = static_cast<uint8_t>(vel_z & 0xFF);
    data[5] = static_cast<uint8_t>((vel_z >> 8) & 0xFF);
    data[6] = static_cast<uint8_t>(vel_yaw & 0xFF);
    data[7] = static_cast<uint8_t>((vel_yaw >> 8) & 0xFF);

    if (serial_comm_->send_protocol_data(TARGET_VELOCITY_FRAME_ID, static_cast<uint8_t>(data.size()), data)) {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
        "Sent target velocity data (frame=0x%02X): x=%d, y=%d, z=%d, yaw=%d",
        TARGET_VELOCITY_FRAME_ID, vel_x, vel_y, vel_z, vel_yaw);
    } else {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
        "Failed to send target velocity data: %s", serial_comm_->get_last_error().c_str());
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Exception in sendTargetVelocityToSerial: %s", e.what());
  }
}

void UartToStm32::protocolDataHandler(uint8_t id, const std::vector<uint8_t> & data)
{
  switch (id) {
    case ST_READY_QUERY_ID: {
      if (data.size() < 9) {
        RCLCPP_WARN(node_->get_logger(), "protocolDataHandler: ID 0xF1 data too short, len=%zu", data.size());
        break;
      }
      const uint8_t first = data[0];
      if (mission_step_pub_) {
        std_msgs::msg::UInt8 msg;
        msg.data = first;
        mission_step_pub_->publish(msg);
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
          "Published /mission_step: %u (from 0xF1 frame)", static_cast<unsigned>(first));
      }
      //----- link echo: byte[3..4] = LEN of the last frame the FC received -----
      // The only receipt channel on the UART3 link: after we send a 0x31 (LEN=8)
      // this field must read back 8.
      const int16_t last_frame_len = static_cast<int16_t>(
        static_cast<uint16_t>(data[3]) | (static_cast<uint16_t>(data[4]) << 8));
      if (link_len_pub_) {
        std_msgs::msg::Int16 link_msg;
        link_msg.data = last_frame_len;
        link_len_pub_->publish(link_msg);
      }
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "FC link echo: last frame LEN=%d (8 = FC received our 0x31)",
        static_cast<int>(last_frame_len));

      //----- velocity echo: byte[5..8] = tacc[0], tacc[1] held by the FC -----
      const int16_t echo_vx = static_cast<int16_t>(
        static_cast<uint16_t>(data[5]) | (static_cast<uint16_t>(data[6]) << 8));
      const int16_t echo_vy = static_cast<int16_t>(
        static_cast<uint16_t>(data[7]) | (static_cast<uint16_t>(data[8]) << 8));
      if (vel_echo_pub_) {
        std_msgs::msg::Int16MultiArray echo_msg;
        echo_msg.data = {echo_vx, echo_vy};
        vel_echo_pub_->publish(echo_msg);
      }

      //----- core-board status echo: byte[9..15] -----
      if (data.size() >= 16) {
        if (fc_status_pub_) {
          std_msgs::msg::UInt8MultiArray status_msg;
          status_msg.data.assign(data.begin() + 9, data.begin() + 16);
          fc_status_pub_->publish(status_msg);
        }
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
          "FC status: mode_sta=%u unlock_sta=%u mode_cmd=%u unlock_cmd=%u cmdCID=%u CMD0=%u CMD1=%u",
          static_cast<unsigned>(data[9]), static_cast<unsigned>(data[10]),
          static_cast<unsigned>(data[11]), static_cast<unsigned>(data[12]),
          static_cast<unsigned>(data[13]), static_cast<unsigned>(data[14]),
          static_cast<unsigned>(data[15]));
      }

      if (has_st_ready_pub_) {
        break;
      }
      const uint8_t second = data[1];
      if (second == 1) {
        if (is_st_ready_pub_) {
          std_msgs::msg::UInt8 msg;
          msg.data = 1;
          is_st_ready_pub_->publish(msg);
          RCLCPP_INFO(node_->get_logger(), "Published /is_st_ready: 1 (from 0xF1 frame)");
        }
        // sendA2ReadyResponse();
        has_st_ready_pub_ = true;
      } else {
        RCLCPP_DEBUG(node_->get_logger(), "0xF1 frame second byte != 1 (%u), ignoring", static_cast<unsigned>(second));
      }
      break;
    }
    case 0xF3: {
      if (data.size() != 15) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
          "Invalid range input diagnostic length: %zu", data.size());
        break;
      }
      const auto u32le = [&data](size_t offset) -> uint32_t {
        return static_cast<uint32_t>(data[offset]) |
          (static_cast<uint32_t>(data[offset + 1]) << 8) |
          (static_cast<uint32_t>(data[offset + 2]) << 16) |
          (static_cast<uint32_t>(data[offset + 3]) << 24);
      };
      std_msgs::msg::Int64MultiArray msg;
      msg.data = {data[0], static_cast<int64_t>(data[1] | (data[2] << 8)),
        static_cast<int64_t>(u32le(3)), static_cast<int64_t>(u32le(7)),
        static_cast<int64_t>(u32le(11))};
      if (range_input_diag_pub_) range_input_diag_pub_->publish(msg);
      break;
    }
    case 0xF2: {
      int32_t values[3];
      if (!decode_height_diag(data.data(), data.size(), values)) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
          "Invalid height diagnostic payload length: %zu", data.size());
        break;
      }
      if (height_diag_pub_) {
        std_msgs::msg::Int32MultiArray msg;
        msg.data.assign(values, values + 3);
        height_diag_pub_->publish(msg);
      }
      break;
    }
    case 0x05: {
      if (data.size() < 2) {
        RCLCPP_WARN(node_->get_logger(), "protocolDataHandler: ID 0x05 data too short");
        break;
      }
      const int16_t value = static_cast<int16_t>(static_cast<uint16_t>(data[0]) |
        (static_cast<uint16_t>(data[1]) << 8));
      std_msgs::msg::Int16 msg;
      msg.data = value;
      // ★ 2026-10-19：原来把"抽帧跳过"也当成"publisher 没初始化"，
      //   200Hz 里有 3/4 的帧都打 WARN，把 journald 刷到限流，
      //   真故障（active/Pose stale）反而被丢掉。分开判断。
      if (!height_pub_) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
          "Height publisher not initialized");
        break;
      }
      if (++decim_height_ % decim_ == 0) {
        const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
        if (!laser_height_filter_.update(value, now_ns, msg.data)) break;
        height_pub_->publish(msg);
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
          "Published /height: %d", msg.data);
      }
      break;
    }
    case 0xB1: {  // 飞控发送的目标速度数据
      if (data.size() < 8) {
        RCLCPP_WARN(node_->get_logger(),
                    "protocolDataHandler: ID 0xB1 data too short (expected 8, got %zu)", data.size());
        break;
      }

      int16_t vel_x = static_cast<int16_t>(data[0] | (data[1] << 8));
      int16_t vel_y = static_cast<int16_t>(data[2] | (data[3] << 8));
      int16_t vel_z = static_cast<int16_t>(data[4] | (data[5] << 8));
      int16_t yaw   = static_cast<int16_t>(data[6] | (data[7] << 8));

      // 每秒打印两次日志（500ms一次）
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
        "[0xB1] Target Speed -> X:%d, Y:%d, Z:%d, Yaw:%d",
        vel_x, vel_y, vel_z, yaw);

      break;
    }
    case FC_IMU_FRAME_ID: {
      publishFcGyro(data);
      publishFcAcc(data);
      publishFcAttitudeQuaternion(data);
      break;
    }
    case FC_EULER_FRAME_ID: {
      publishFcAttitudeEuler(data);
      break;
    }
    case FC_CTRL_FRAME_ID: {
      publishFcTarget(data);
      break;
    }
    case FC_VEL_FRAME_ID: {   // 0x07：飞控自己的速度估计
      publishFcVelocity(data);
      break;
    }
    case FC_PWM_FRAME_ID: {   // 0x20：PWM 控制量（4 轴只发前 4 路）
      publishFcPwm(data);
      break;
    }
    case FC_TGT_ATTI_FRAME_ID: {   // 0x0A：目标姿态
      publishFcTargetAtti(data);
      break;
    }
    case FC_TGT_VEL_FRAME_ID: {    // 0x0B：目标速度（飞控内部位置环输出）
      publishFcTargetVel(data);
      break;
    }
    case FC_CTRL_AMT_FRAME_ID: {   // 0x21：姿态控制量
      publishFcCtrlAmt(data);
      break;
    }
    case FC_VEL_FWD_FRAME_ID: {    // 0x0C：转发验证帧（内容 = 0x07 飞控速度）
      publishFcVelFwd(data);
      break;
    }
    case FC_MAG_FRAME_ID: {        // ★2026-10-16 0x02：罗盘/气压/温度（407 原样转发）→ 磁干扰监测
      publishFcMag(data);
      break;
    }
    case FC_DIAG_FRAME_ID: {       // 0x0E：电压 / RC失联 / 传感器状态（407 只读诊断）
      publishFcDiag(data);
      break;
    }
    default: {
      RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
        "Unhandled protocol ID: 0x%02X, len=%zu", id, data.size());
      break;
    }
  }
}

// 飞控 0x01 帧（200Hz）：acc x/y/z (3*s16) + gyro x/y/z (3*s16) + 四元数 w,x,y,z (4*s16, x10000)
// 四元数才是高速姿态源（凌霄 IMU 的欧拉角只有 2Hz），所以主姿态从这一帧反算。
void UartToStm32::publishFcAttitudeQuaternion(const std::vector<uint8_t> & data)
{
  if (data.size() < 20) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "0x01 attitude frame too short: %zu (expected 20)", data.size());
    return;
  }

  auto read_s16 = [](const std::vector<uint8_t> & d, size_t off) -> int16_t {
      return static_cast<int16_t>(static_cast<uint16_t>(d[off]) |
                                  (static_cast<uint16_t>(d[off + 1]) << 8));
    };

  const double qw = read_s16(data, 12) / 10000.0;
  const double qx = read_s16(data, 14) / 10000.0;
  const double qy = read_s16(data, 16) / 10000.0;
  const double qz = read_s16(data, 18) / 10000.0;

  const double norm2 = qw * qw + qx * qx + qy * qy + qz * qz;
  if (!(norm2 > 1e-6)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "0x01 frame carries an all-zero quaternion - is the LX IMU configured to output the "
      "quaternion (0x04) frame?");
    return;
  }

  tf2::Quaternion q(qx, qy, qz, qw);
  q.normalize();
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  const double roll_deg = roll * 180.0 / M_PI;
  const double pitch_deg = pitch * 180.0 / M_PI;
  const double yaw_deg = yaw * 180.0 / M_PI;

  // ★2026-10-16 调试出口：把 14 个字节原样发出来（/fc_mag_raw），
  //   用来核对协议表里写的字段偏移（MAG_X/Y/Z s16 + ALT_BAR s32 + TMP s16 + BAR_STA u8 + MAG_STA u8）
  //   到底和飞控实际发的是不是一致 —— tmp 读出 529℃ 说明至少有一处对不上。
  std_msgs::msg::Float32MultiArray raw;
  raw.data.assign(data.begin(), data.begin() + 14);
  if (fc_mag_raw_pub_) {
    fc_mag_raw_pub_->publish(raw);
  }
  std_msgs::msg::Float32MultiArray msg;
  msg.data = {static_cast<float>(roll_deg),
              static_cast<float>(pitch_deg),
              static_cast<float>(yaw_deg)};
  if (fc_attitude_pub_) {
    fc_attitude_pub_->publish(msg);
  }

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
    "FC attitude (0x01 quat): roll=%.1f pitch=%.1f yaw=%.1f deg",
    roll_deg, pitch_deg, yaw_deg);
}

// 飞控 0x01 帧里的陀螺仪原始值（字节 6-11，3*s16）。
// 协议没定义量纲，看波形用相对值 —— 但"陀螺 Z 轴在摆"就是机身在转的最硬证据，
// 它和 SLAM 完全无关，所以能直接回答"是机体在转还是位姿估计在摆"。
void UartToStm32::publishFcGyro(const std::vector<uint8_t> & data)
{
  if (data.size() < 12) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "0x01 gyro frame too short: %zu (expected >= 12)", data.size());
    return;
  }

  auto read_s16 = [](const std::vector<uint8_t> & d, size_t off) -> int16_t {
      return static_cast<int16_t>(static_cast<uint16_t>(d[off]) |
                                  (static_cast<uint16_t>(d[off + 1]) << 8));
    };

  const int16_t gx = read_s16(data, 6);
  const int16_t gy = read_s16(data, 8);
  const int16_t gz = read_s16(data, 10);

  geometry_msgs::msg::Vector3 msg;
  msg.x = static_cast<double>(gx);
  msg.y = static_cast<double>(gy);
  msg.z = static_cast<double>(gz);
  if (fc_gyro_pub_) {
    fc_gyro_pub_->publish(msg);
  }

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
    "FC gyro raw (0x01): x=%d y=%d z=%d", static_cast<int>(gx),
    static_cast<int>(gy), static_cast<int>(gz));
}

// 飞控 0x01 帧里的加速度原始值（字节 0-5，3*s16）。
// ⚠️ 协议只写"依次为加速度"，没给量纲 —— 融合节点靠"静止时 |a| = 1g"自己标定。
void UartToStm32::publishFcAcc(const std::vector<uint8_t> & data)
{
  if (data.size() < 6) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "0x01 acc frame too short: %zu (expected >= 6)", data.size());
    return;
  }

  auto read_s16 = [](const std::vector<uint8_t> & d, size_t off) -> int16_t {
      return static_cast<int16_t>(static_cast<uint16_t>(d[off]) |
                                  (static_cast<uint16_t>(d[off + 1]) << 8));
    };

  geometry_msgs::msg::Vector3 msg;
  msg.x = static_cast<double>(read_s16(data, 0));
  msg.y = static_cast<double>(read_s16(data, 2));
  msg.z = static_cast<double>(read_s16(data, 4));
  if (fc_acc_pub_) {
    fc_acc_pub_->publish(msg);
  }
}

// ★2026-10-09 新增：飞控自己的速度估计（407 侧 AnoPTv7SendVEL 发的帧，ID 0x07）
// S16 × 3：vx, vy, vz，单位 cm/s。协议注："速度在定点模式下才有数据"。
// 这是判定"飞控知不知道自己在漂"的直接证据：
//   它报的也在漂 → 它知道，只是纠不动（权限/增益）；
//   它报 0 而实际在漂 → 它的观测/融合里就没这个漂移。
void UartToStm32::publishFcVelocity(const std::vector<uint8_t> & data)
{
  if (data.size() < 6) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "0x07 velocity frame too short: %zu (expected >= 6)", data.size());
    return;
  }

  auto read_s16 = [](const std::vector<uint8_t> & d, size_t off) -> int16_t {
      return static_cast<int16_t>(static_cast<uint16_t>(d[off]) |
                                  (static_cast<uint16_t>(d[off + 1]) << 8));
    };

  geometry_msgs::msg::Vector3 msg;
  msg.x = static_cast<double>(read_s16(data, 0));
  msg.y = static_cast<double>(read_s16(data, 2));
  msg.z = static_cast<double>(read_s16(data, 4));
  if (fc_vel_pub_ && (++decim_vel_ % decim_ == 0)) {
    fc_vel_pub_->publish(msg);
  }

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
    "FC own velocity (0x07): x=%.1f y=%.1f z=%.1f cm/s", msg.x, msg.y, msg.z);
}

// PWM 控制量（407 原样转发的 IMU 帧，ID 0x20）。
// 协议：范围 0-10000，单位 0.01% 油门；默认 4 轴，只输出前 4 通道。
// 这是控制链最末端（IMU 板的混控输出）：
//   输出在 8~12 秒周期上摆 = 飞控自己在驱动摆动；输出平 = 外部扰动、飞控被动纠。
void UartToStm32::publishFcPwm(const std::vector<uint8_t> & data)
{
  if (data.size() < 2) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "0x20 pwm frame too short: %zu", data.size());
    return;
  }

  std_msgs::msg::Float32MultiArray msg;
  const size_t n = data.size() / 2;          // 4 轴 = 4，6 轴 = 6，8 轴 = 8
  msg.data.resize(n);
  for (size_t i = 0; i < n; ++i) {
    const uint16_t raw = static_cast<uint16_t>(data[i * 2]) |
                         (static_cast<uint16_t>(data[i * 2 + 1]) << 8);
    msg.data[i] = static_cast<float>(raw) * 0.01f;  // 0.01% -> %
  }
  if (fc_pwm_pub_ && (++decim_pwm_ % decim_ == 0)) {
    fc_pwm_pub_->publish(msg);
  }

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
    "FC pwm (0x20, %zu 路): %.2f %.2f %.2f %.2f %%",
    n, msg.data[0], n > 1 ? msg.data[1] : 0.0f,
    n > 2 ? msg.data[2] : 0.0f, n > 3 ? msg.data[3] : 0.0f);
}

// 目标姿态（407 原样转发的 IMU 帧，ID 0x0A）：S16×3 ROL/PIT/YAW，精确到 0.01 度。
void UartToStm32::publishFcTargetAtti(const std::vector<uint8_t> & data)
{
  if (data.size() < 6) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "0x0A target-attitude frame too short: %zu (expected >= 6)", data.size());
    return;
  }
  auto read_s16 = [](const std::vector<uint8_t> & d, size_t off) -> int16_t {
      return static_cast<int16_t>(static_cast<uint16_t>(d[off]) |
                                  (static_cast<uint16_t>(d[off + 1]) << 8));
    };
  std_msgs::msg::Float32MultiArray msg;
  msg.data = {static_cast<float>(read_s16(data, 0) * 0.01),
              static_cast<float>(read_s16(data, 2) * 0.01),
              static_cast<float>(read_s16(data, 4) * 0.01)};
  if (fc_target_atti_pub_) {
    fc_target_atti_pub_->publish(msg);
  }
}

// 目标速度（407 原样转发的 IMU 帧，ID 0x0B）：S16×3，cm/s。这是飞控内部位置环的输出。
void UartToStm32::publishFcTargetVel(const std::vector<uint8_t> & data)
{
  if (data.size() < 6) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "0x0B target-velocity frame too short: %zu (expected >= 6)", data.size());
    return;
  }
  auto read_s16 = [](const std::vector<uint8_t> & d, size_t off) -> int16_t {
      return static_cast<int16_t>(static_cast<uint16_t>(d[off]) |
                                  (static_cast<uint16_t>(d[off + 1]) << 8));
    };
  geometry_msgs::msg::Vector3 msg;
  msg.x = static_cast<double>(read_s16(data, 0));
  msg.y = static_cast<double>(read_s16(data, 2));
  msg.z = static_cast<double>(read_s16(data, 4));
  if (fc_target_vel_pub_) {
    fc_target_vel_pub_->publish(msg);
  }
}

// ★2026-10-16：罗盘/气压/温度（407 原样转发的 IMU 0x02 帧，14B）
//   载荷：MAG_X/Y/Z(s16×3) + ALT_BAR(s32) + TMP(s16) + BAR_STA(u8) + MAG_STA(u8)
//   用途：实时监测磁干扰。铁柱/货架等金属会把飞控罗盘拉偏 → 航向基准跑偏 →
//   偏航环持续输出差速 → 飞机自转/漂移（当天实测：铁柱旁悬停 err 25~56°、撞墙；
//   同一架飞机在空地 err 1~8°）。发布 [mag_x, mag_y, mag_z, mag_sta, bar_sta, tmp, alt_bar]。
void UartToStm32::publishFcMag(const std::vector<uint8_t> & data)
{
  if (data.size() < 14) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "0x02 mag frame too short: %zu (expected >= 14)", data.size());
    return;
  }
  auto read_s16 = [](const std::vector<uint8_t> & d, size_t off) -> int16_t {
      return static_cast<int16_t>(static_cast<uint16_t>(d[off]) |
                                  (static_cast<uint16_t>(d[off + 1]) << 8));
    };
  const int16_t mx = read_s16(data, 0);
  const int16_t my = read_s16(data, 2);
  const int16_t mz = read_s16(data, 4);
  const int32_t alt_bar = static_cast<int32_t>(
      static_cast<uint32_t>(data[6]) | (static_cast<uint32_t>(data[7]) << 8) |
      (static_cast<uint32_t>(data[8]) << 16) | (static_cast<uint32_t>(data[9]) << 24));
  const int16_t tmp = read_s16(data, 10);
  std_msgs::msg::Float32MultiArray msg;
  msg.data = {static_cast<float>(mx), static_cast<float>(my), static_cast<float>(mz),
              static_cast<float>(data[13]),   // MAG_STA：罗盘状态（飞控自己判的干扰标志）
              static_cast<float>(data[12]),   // BAR_STA：气压状态
              static_cast<float>(tmp),
              static_cast<float>(alt_bar)};
  if (fc_mag_pub_) {
    fc_mag_pub_->publish(msg);
  }
}

// 407 核心健康诊断（0x0E，20Hz）。
// 首5项兼容旧6字节帧；20字节扩展帧追加RC failsafe、降落来源、
// 0x31/0x32超时状态和CH3/CH5/CH6原始值。全部字段只读。
// /fc_diag = [
//   voltage_V, rc_lost, imu_state, bar_sta, mag_sta,
//   rc_failsafe, mission_end_req, land_calls, cmd_lost, fbk_lost,
//   takeoff_if, flown_flag, tk_req, tk_fail, ch3, ch5, ch6
// ]
void UartToStm32::publishFcDiag(const std::vector<uint8_t> & data)
{
  if (data.size() < 6) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "0x0E fc-diag frame too short: %zu (expected at least 6)", data.size());
    return;
  }

  const uint16_t voltage_100 = static_cast<uint16_t>(data[0]) |
    (static_cast<uint16_t>(data[1]) << 8);
  const auto s16le = [](const std::vector<uint8_t> & d, size_t off) -> int16_t {
    return static_cast<int16_t>(
      static_cast<uint16_t>(d[off]) |
      (static_cast<uint16_t>(d[off + 1]) << 8));
  };

  std_msgs::msg::Float32MultiArray msg;
  const bool extended = data.size() >= 20;
  if (extended) {
    const uint16_t land_calls = static_cast<uint16_t>(data[8]) |
      (static_cast<uint16_t>(data[9]) << 8);
    const uint8_t state_bits = data[12];
    msg.data = {
      static_cast<float>(voltage_100) / 100.0f,
      static_cast<float>(data[2]),                         // rc_lost
      static_cast<float>(data[4]),                         // imu_state
      static_cast<float>(data[5]),                         // bar_sta
      static_cast<float>(data[6]),                         // mag_sta
      static_cast<float>(data[3]),                         // rc_failsafe
      static_cast<float>(data[7]),                         // mission_end_req
      static_cast<float>(land_calls),                      // land_calls
      static_cast<float>(data[10]),                        // cmd_lost
      static_cast<float>(data[11]),                        // fbk_lost
      static_cast<float>((state_bits & 0x01) != 0),        // takeoff_if
      static_cast<float>((state_bits & 0x02) != 0),        // flown_flag
      static_cast<float>((state_bits & 0x04) != 0),        // tk_req
      static_cast<float>((state_bits & 0x08) != 0),        // tk_fail
      static_cast<float>(s16le(data, 14)),                 // CH3 throttle
      static_cast<float>(s16le(data, 16)),                 // CH5
      static_cast<float>(s16le(data, 18))};                // CH6
  } else {
    msg.data = {
      static_cast<float>(voltage_100) / 100.0f,
      static_cast<float>(data[2]),
      static_cast<float>(data[3]),
      static_cast<float>(data[4]),
      static_cast<float>(data[5])};
  }
  if (fc_diag_pub_) {
    fc_diag_pub_->publish(msg);
  }

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
    "FC diag (0x0E): volt=%.2fV rc_lost=%u imu_state=%u bar=%u mag=%u ext=%s",
    msg.data[0], static_cast<unsigned>(data[2]),
    static_cast<unsigned>(extended ? data[4] : data[3]),
    static_cast<unsigned>(extended ? data[5] : data[4]),
    static_cast<unsigned>(extended ? data[6] : data[5]),
    extended ? "yes" : "no");
}

// 姿态控制量（407 原样转发的 IMU 帧，ID 0x21）：S16×4 ROL/PIT/THR/YAW，前三个 ±5000，THR 0-10000。
// ★2026-10-10 验证帧：内容与 0x07 完全相同，用来判断"转发函数"是否有效。
void UartToStm32::publishFcVelFwd(const std::vector<uint8_t> & data)
{
  if (data.size() < 6) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "0x0C fwd frame too short: %zu (expected >= 6)", data.size());
    return;
  }
  auto read_s16 = [](const std::vector<uint8_t> & d, size_t off) -> int16_t {
      return static_cast<int16_t>(static_cast<uint16_t>(d[off]) |
                                  (static_cast<uint16_t>(d[off + 1]) << 8));
    };
  geometry_msgs::msg::Vector3 msg;
  msg.x = static_cast<double>(read_s16(data, 0));
  msg.y = static_cast<double>(read_s16(data, 2));
  msg.z = static_cast<double>(read_s16(data, 4));
  if (fc_vel_fwd_pub_) {
    fc_vel_fwd_pub_->publish(msg);
  }
}

// 这是姿态环 PID 的输出 —— 判定慢摆"是谁在驱动"最直接的证据。
void UartToStm32::publishFcCtrlAmt(const std::vector<uint8_t> & data)
{
  if (data.size() < 8) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "0x21 ctrl-amount frame too short: %zu (expected >= 8)", data.size());
    return;
  }
  auto read_s16 = [](const std::vector<uint8_t> & d, size_t off) -> int16_t {
      return static_cast<int16_t>(static_cast<uint16_t>(d[off]) |
                                  (static_cast<uint16_t>(d[off + 1]) << 8));
    };
  std_msgs::msg::Float32MultiArray msg;
  msg.data = {static_cast<float>(read_s16(data, 0)),
              static_cast<float>(read_s16(data, 2)),
              static_cast<float>(read_s16(data, 4)),
              static_cast<float>(read_s16(data, 6))};
  if (fc_ctrl_amt_pub_) {
    fc_ctrl_amt_pub_->publish(msg);
  }
}

// 飞控 0x41 帧：控制目标(匿名协议"实时控制帧")，7 个 s16
//   rol/pit x100deg, thr 0-1000(0.1%), yaw_dps deg/s, vel_x/y/z cm/s
// 用途：和 fcGyroZ(实际) 对比 —— 命令也在 0.1Hz 摆 = 控制/估计侧在驱动；
//       命令平而实际摆 = 外部扰动/机械。
void UartToStm32::publishFcTarget(const std::vector<uint8_t> & data)
{
  if (data.size() < 14) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "0x41 control frame too short: %zu (expected 14)", data.size());
    return;
  }

  auto read_s16 = [](const std::vector<uint8_t> & d, size_t off) -> int16_t {
      return static_cast<int16_t>(static_cast<uint16_t>(d[off]) |
                                  (static_cast<uint16_t>(d[off + 1]) << 8));
    };

  const double rol = read_s16(data, 0) / 100.0;
  const double pit = read_s16(data, 2) / 100.0;
  const double thr = read_s16(data, 4);
  const double yaw_dps = read_s16(data, 6);
  const double vx = read_s16(data, 8);
  const double vy = read_s16(data, 10);
  const double vz = read_s16(data, 12);

  std_msgs::msg::Float32MultiArray msg;
  msg.data = {static_cast<float>(rol), static_cast<float>(pit), static_cast<float>(thr),
              static_cast<float>(yaw_dps), static_cast<float>(vx),
              static_cast<float>(vy), static_cast<float>(vz)};
  if (fc_target_pub_) {
    if (++decim_tgt_ % decim_ == 0) fc_target_pub_->publish(msg);
  }

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
    "FC target (0x41): rol=%.1f pit=%.1f thr=%.0f yaw_dps=%.0f v=(%.0f,%.0f,%.0f)",
    rol, pit, thr, yaw_dps, vx, vy, vz);
}

// 飞控 0x03 帧（源只有 2Hz）：roll,pitch,yaw (3*s16, x100)，单位度
// 只用来交叉校验 —— 和 /fc_attitude 的 yaw 对不上就说明某一路有问题。
void UartToStm32::publishFcAttitudeEuler(const std::vector<uint8_t> & data)
{
  if (data.size() < 6) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "0x03 attitude frame too short: %zu (expected 6)", data.size());
    return;
  }

  auto read_s16 = [](const std::vector<uint8_t> & d, size_t off) -> int16_t {
      return static_cast<int16_t>(static_cast<uint16_t>(d[off]) |
                                  (static_cast<uint16_t>(d[off + 1]) << 8));
    };

  const double roll_deg = read_s16(data, 0) / 100.0;
  const double pitch_deg = read_s16(data, 2) / 100.0;
  const double yaw_deg = read_s16(data, 4) / 100.0;

  std_msgs::msg::Float32MultiArray msg;
  msg.data = {static_cast<float>(roll_deg),
              static_cast<float>(pitch_deg),
              static_cast<float>(yaw_deg)};
  if (fc_attitude_euler_pub_) {
    fc_attitude_euler_pub_->publish(msg);
  }

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
    "FC attitude (0x03 euler): roll=%.1f pitch=%.1f yaw=%.1f deg",
    roll_deg, pitch_deg, yaw_deg);
}

void UartToStm32::sendA2ReadyResponse()
{
  if (!serial_comm_ || !serial_comm_->is_open()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "Serial port is not open, cannot send A2 ready response");
    return;
  }

  std::vector<uint8_t> data(9, 0x00);
  data[0] = 0x01;
  if (serial_comm_->send_protocol_data(A2_READY_RESP_ID, static_cast<uint8_t>(data.size()), data)) {
    RCLCPP_INFO(node_->get_logger(), "Sent A2 ready response (len=9, first=0x01)");
  } else {
    RCLCPP_WARN(node_->get_logger(), "Failed to send A2 ready response: %s", serial_comm_->get_last_error().c_str());
  }
}

void UartToStm32::sendThirdWaypointNotifyThreeTimes()
{
  if (!serial_comm_ || !serial_comm_->is_open()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "Serial port is not open, cannot send third-waypoint notify frame");
    return;
  }

  std::vector<uint8_t> data;
  for (int i = 0; i < 3; ++i) {
    if (serial_comm_->send_protocol_data(THIRD_WAYPOINT_NOTIFY_FRAME_ID, static_cast<uint8_t>(data.size()), data)) {
      RCLCPP_INFO(node_->get_logger(),
        "Sent third-waypoint notify frame 0x11 (%d/3)", i + 1);
    } else {
      RCLCPP_WARN(node_->get_logger(),
        "Failed to send third-waypoint notify frame 0x11 (%d/3): %s",
        i + 1, serial_comm_->get_last_error().c_str());
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void UartToStm32::activeControllerCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  if (msg->data != 2 || a2_ready_remaining_ > 0) {
    return;
  }
  sendA2ReadyResponse();
  a2_ready_remaining_ = 2;
  if (!a2_ready_timer_) {
    a2_ready_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100), [this]() {
      sendA2ReadyResponse();
      if (--a2_ready_remaining_ == 0) {
        a2_ready_timer_->cancel();
      }
    });
  } else {
    a2_ready_timer_->reset();
  }
}

void UartToStm32::routeReachedIndexCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  if (!third_waypoint_notified_ && msg->data >= 2) {
    third_waypoint_notified_ = true;
    RCLCPP_INFO(node_->get_logger(),
      "Reached waypoint index %u (third waypoint reached), send notify frame 0x11 three times",
      static_cast<unsigned>(msg->data));
    sendThirdWaypointNotifyThreeTimes();
  }
}

// /mission_cmd = [子命令, 高度cm]  -> 0x66 任务控制帧
//   子命令 0x01 起飞(高度) / 0x02 降落 / 0x03 返航 / 0x04 锁定
void UartToStm32::missionCmdCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 1) {
    RCLCPP_WARN(node_->get_logger(), "/mission_cmd needs at least [sub_command]");
    return;
  }
  if (!serial_comm_ || !serial_comm_->is_open()) {
    RCLCPP_WARN(node_->get_logger(), "Serial not open, cannot send 0x66");
    return;
  }
  const uint8_t sub = static_cast<uint8_t>(msg->data[0] & 0xFF);
  const uint16_t height = (msg->data.size() >= 2 && msg->data[1] > 0)
                            ? static_cast<uint16_t>(msg->data[1]) : 0;

  std::vector<uint8_t> data(3);
  data[0] = sub;
  data[1] = static_cast<uint8_t>(height & 0xFF);
  data[2] = static_cast<uint8_t>((height >> 8) & 0xFF);

  if (serial_comm_->send_protocol_data(MISSION_CMD_FRAME_ID,
                                       static_cast<uint8_t>(data.size()), data)) {
    RCLCPP_INFO(node_->get_logger(),
      "Sent 0x66 mission cmd: sub=0x%02X height=%u cm", sub, static_cast<unsigned>(height));
  } else {
    RCLCPP_WARN(node_->get_logger(), "Failed to send 0x66: %s",
      serial_comm_->get_last_error().c_str());
  }
}

void UartToStm32::fcTestTriggerCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  if (msg->data == 1) {
    RCLCPP_INFO(node_->get_logger(),
      "Received /fc_test_trigger=1, sending test notify frame 0x11 three times");
    sendThirdWaypointNotifyThreeTimes();
    return;
  }

  RCLCPP_INFO(node_->get_logger(),
    "Received /fc_test_trigger=%u (ignored). Send 1 to trigger 0x11 test.",
    static_cast<unsigned>(msg->data));
}

}  // namespace uart_to_stm32
