#include "pid_control_pkg/pid_controller.hpp"

#include <angles/angles.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace pid_control_pkg
{

PIDController::PIDController(
  double kp,
  double ki,
  double kd,
  double max_output,
  double min_output,
  double integral_limit,
  double deadzone)
: kp_(kp),
  ki_(ki),
  kd_(kd),
  max_output_(max_output),
  min_output_(min_output),
  integral_limit_(integral_limit),
  deadzone_(deadzone),
  prev_error_(0.0),
  current_error_(0.0),
  integral_(0.0),
  prev_derivative_(0.0),
  first_call_(true),
  derivative_filter_alpha_(0.8)
{
}

double PIDController::calculate(double setpoint, double measured_value, double dt)
{
  current_error_ = setpoint - measured_value;

  if (std::fabs(current_error_) < deadzone_) {
    current_error_ = 0.0;
  }

  if (first_call_) {
    prev_error_ = current_error_;
    first_call_ = false;
  }

  const double proportional = kp_ * current_error_;

  integral_ += current_error_ * dt;
  if (integral_ > integral_limit_) {
    integral_ = integral_limit_;
  } else if (integral_ < -integral_limit_) {
    integral_ = -integral_limit_;
  }
  const double integral_term = ki_ * integral_;

  const double derivative_raw = (dt > 0.0) ? (current_error_ - prev_error_) / dt : 0.0;
  const double derivative_filtered = derivative_filter_alpha_ * prev_derivative_ +
    (1.0 - derivative_filter_alpha_) * derivative_raw;
  const double derivative_term = kd_ * derivative_filtered;

  double output = proportional + integral_term + derivative_term;

  if (output > max_output_) {
    output = max_output_;
  } else if (output < min_output_) {
    output = min_output_;
  }

  if ((output >= max_output_ && current_error_ > 0.0) ||
    (output <= min_output_ && current_error_ < 0.0))
  {
    integral_ -= current_error_ * dt;
  }

  prev_error_ = current_error_;
  prev_derivative_ = derivative_filtered;

  return output;
}

void PIDController::reset()
{
  prev_error_ = 0.0;
  current_error_ = 0.0;
  integral_ = 0.0;
  prev_derivative_ = 0.0;
  first_call_ = true;
}

void PIDController::setPID(double kp, double ki, double kd)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void PIDController::setOutputLimits(double max_output, double min_output)
{
  max_output_ = max_output;
  min_output_ = min_output;
}

void PIDController::setIntegralLimit(double integral_limit)
{
  integral_limit_ = integral_limit;
}

void PIDController::setDeadzone(double deadzone)
{
  deadzone_ = deadzone;
}
/*
  订阅/发布主题：
    订阅 /target_position (std_msgs::msg::Float32MultiArray)：接收目标位置和朝向，包含四个浮点数 [x_cm, y_cm, z_cm, yaw_deg]。
    订阅 /height 
    发布 /target_velocity (std_msgs::msg::Float32MultiArray)
*/
PositionPIDController::PositionPIDController()
: rclcpp::Node("position_pid_controller"),
  // integral_limit 5.0 → 20.0（2026-09-30）：原来积分被夹在 ±5，配 ki=0.1 最多只贡献
  // 0.5cm/s，压根补不上实测那 5cm 静差。放到 20 后 ki=0.1 最多贡献 2cm/s，够用。
  pid_x_(0.5, 0.0, 0.1, 36.0, -33.0, 20.0, 0.6),
  pid_y_(0.5, 0.0, 0.1, 36.0, -33.0, 20.0, 0.6),
  pid_yaw_(1.0, 0.0, 0.2, 30.0, -30.0, 2.0, 0.5),
  pid_z_(0.9, 0.0, 0.15, 35.0, -60.0, 3.0, 0.6),
  pid_xy_speed_(0.5, 0.0, 0.1, 36.0, -36.0, 5.0, 0.6),
  target_x_cm_(0.0),
  target_y_cm_(0.0),
  target_z_cm_(0.0),
  target_yaw_deg_(0.0),
  has_target_position_(false),
  has_target_height_(false),
  current_x_cm_(0.0),
  current_y_cm_(0.0),
  current_yaw_deg_(0.0),
  current_z_cm_(0.0),
  has_current_pose_(false),
  control_frequency_(50.0),
  map_frame_("map"),
  laser_link_frame_("laser_link"),
  control_mode_(ControlMode::NORMAL),
  position_tolerance_(6.0),
  yaw_tolerance_(5.0),
  yaw_deadband_deg_(0.0),
  height_tolerance_(6.0),
  pose_data_timeout_sec_(0.5),
  height_min_cm_(0.0),
  height_max_cm_(250.0),
  height_filter_jump_threshold_cm_(35.0),
  height_filter_required_frames_(5),
  has_height_candidate_(false),
  height_candidate_cm_(0.0),
  height_candidate_frames_(0),
  max_linear_vel_(36.0),
  max_angular_vel_(30.0),
  max_vertical_vel_(40.0),
  max_slow_vel_(20.0),
  distance_xy_cm_(0.0),
  error_x_cm_(0.0),
  error_y_cm_(0.0),
  error_yaw_deg_(0.0),
  error_z_cm_(0.0),
  is_active_controller_(false),
  is_emergency_landing_(false),
  should_stop_(false),
  has_route_current_index_(false),
  route_current_index_(0),
  vertical_takeoff_index_(1),
  vertical_takeoff_min_z_cm_(20.0),
  takeoff_max_xy_velocity_(8.0),
  yaw_turning_(false),
  last_target_yaw_deg_(0.0),
  has_last_target_yaw_(false),
  yaw_spin_left_s_(0.0),
  yaw_spin_sign_(1.0),
  last_update_time_(now())
{
  loadParameters();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  target_position_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
    "/target_position", rclcpp::QoS(10),
    std::bind(&PositionPIDController::targetPositionCallback, this, std::placeholders::_1));

  active_controller_sub_ = create_subscription<std_msgs::msg::UInt8>(
    "/active_controller", rclcpp::QoS(10),
    std::bind(&PositionPIDController::activeControllerCallback, this, std::placeholders::_1));

  route_current_index_sub_ = create_subscription<std_msgs::msg::UInt8>(
    "/route_current_index", rclcpp::QoS(10).transient_local().reliable(),
    std::bind(&PositionPIDController::routeCurrentIndexCallback, this, std::placeholders::_1));

  bluetooth_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
    "/bluetooth_data", 10,
    std::bind(&PositionPIDController::bluetoothCallback, this, std::placeholders::_1));

    // target_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    //         "target_position", 10, std::bind(&ControlNode::targetCallback, this, std::placeholders::_1));

  height_sub_ = create_subscription<std_msgs::msg::Int16>(
    "/height", rclcpp::QoS(10),
    std::bind(&PositionPIDController::heightCallback, this, std::placeholders::_1));

  target_velocity_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
    "/target_velocity", rclcpp::QoS(10));

  const double period_sec = 1.0 / std::max(control_frequency_, 1.0);
  control_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_sec)),
    std::bind(&PositionPIDController::controlTimerCallback, this));

  RCLCPP_INFO(get_logger(), "Position PID Controller initialized (%.1f Hz)", control_frequency_);
  RCLCPP_INFO(get_logger(), "Frames: map=%s, laser_link=%s", map_frame_.c_str(), laser_link_frame_.c_str());
}

void PositionPIDController::activeControllerCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  // 2 代表无人机 (Drone)
  if (msg->data == 2) {
    is_active_controller_ = true;
    should_stop_ = false;
  } else if (msg->data == 3) {
    // 3 代表无人机停止 (Drone Stop)
    is_active_controller_ = true;
    should_stop_ = true;
  } else {
    is_active_controller_ = false;
    should_stop_ = false;
  }
}

void PositionPIDController::routeCurrentIndexCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  route_current_index_ = msg->data;
  has_route_current_index_ = true;
  RCLCPP_INFO(get_logger(), "Route current waypoint index: %u", static_cast<unsigned>(route_current_index_));
}

void PositionPIDController::bluetoothCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
  if (!msg->data.empty() && msg->data[0] == 3) {
    is_emergency_landing_ = true;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "EMERGENCY LANDING TRIGGERED (Bluetooth=3)");
  } else {
    is_emergency_landing_ = false;
  }
}

void PositionPIDController::targetPositionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 4) {
    RCLCPP_WARN(get_logger(), "Target position message requires 4 floats [x_cm, y_cm, z_cm, yaw_deg]");
    return;
  }

  target_x_cm_ = static_cast<double>(msg->data[0]);
  target_y_cm_ = static_cast<double>(msg->data[1]);
  target_z_cm_ = static_cast<double>(msg->data[2]);
  target_yaw_deg_ = static_cast<double>(msg->data[3]);
  has_target_position_ = true;
  if (!has_last_target_yaw_) {          // 第一条目标消息：当作基准，不算跳变
    last_target_yaw_deg_ = target_yaw_deg_;
    has_last_target_yaw_ = true;
  }

  RCLCPP_INFO(get_logger(),
    "Received target: x=%.1fcm y=%.1fcm z=%.1fcm yaw=%.1fdeg",
    target_x_cm_, target_y_cm_, target_z_cm_, target_yaw_deg_);
}

void PositionPIDController::heightCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  const double h = static_cast<double>(msg->data);
  // ★ 2026-10-07：高度遥测偶发被串口干扰成 ~3840/3998cm（实测 3810 个采样里
  //   218 个是假的，约 6%）。位置环拿到这种值会以为"飞太高"而猛往下压 ——
  //   实测起飞目标 142cm，飞机只到 47~72cm 就被压回地面。这里做两道剔除：
  //     1) 绝对值超过 500cm 的直接丢（本场地最高也就 2m 出头）
  //     2) 单次跳变超过 100cm 的也丢（相邻两帧高度不会这么跳）
  //   丢掉的帧保持上一次的值不动，Z 环不受影响。
  if (h < height_min_cm_ || h > height_max_cm_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
      "高度遥测异常 %.0fcm → 丢弃", h);
    return;
  }
  const double jump = height_filter_jump_threshold_cm_;
  if (!has_target_height_ || jump <= 0.0 || std::fabs(h - current_z_cm_) <= jump) {
    current_z_cm_ = h;
    has_target_height_ = true;
    has_height_candidate_ = false;
    height_candidate_frames_ = 0;
    return;
  }
  if (!has_height_candidate_ ||
      std::fabs(h - height_candidate_cm_) > jump)
  {
    height_candidate_cm_ = h;
    height_candidate_frames_ = 1;
    has_height_candidate_ = true;
  } else {
    height_candidate_cm_ = h;
    ++height_candidate_frames_;
  }
  if (has_height_candidate_ &&
      height_candidate_frames_ >= height_filter_required_frames_)
  {
    current_z_cm_ = h;
    has_target_height_ = true;
    has_height_candidate_ = false;
    height_candidate_frames_ = 0;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
      "高度跳变已连续 %d 帧确认，接受 %.0fcm", height_filter_required_frames_, h);
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
      "高度跳变 %.0f -> %.0fcm，等待连续确认 %d/%d",
      current_z_cm_, h, height_candidate_frames_, height_filter_required_frames_);
  }
}
/*
    获取自动获取tf数据
*/
bool PositionPIDController::getCurrentPose()
{
  try {
    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
      map_frame_, laser_link_frame_, tf2::TimePointZero);
    const rclcpp::Time stamp(transform.header.stamp, get_clock()->get_clock_type());
    if (stamp.nanoseconds() > 0) {
      const double age = (now() - stamp).seconds();
      if (age < 0.0 || age > pose_data_timeout_sec_) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
          "Pose stale: age=%.3fs limit=%.3fs -> zero velocity",
          age, pose_data_timeout_sec_);
        return false;
      }
    }

    current_x_cm_ = meterToCm(transform.transform.translation.x);
    current_y_cm_ = meterToCm(transform.transform.translation.y);

    tf2::Quaternion q;
    tf2::fromMsg(transform.transform.rotation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    current_yaw_deg_ = radToDeg(yaw);

    has_current_pose_ = true;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Failed to lookup transform %s -> %s: %s",
      map_frame_.c_str(), laser_link_frame_.c_str(), ex.what());
    return false;
  }
}

double PositionPIDController::normalizeAngleDeg(double angle_deg) const
{
  double result = angles::normalize_angle(angles::from_degrees(angle_deg));
  return angles::to_degrees(result);
}

void PositionPIDController::calculateErrors()
{
  error_x_cm_ = target_x_cm_ - current_x_cm_;
  error_y_cm_ = target_y_cm_ - current_y_cm_;
  distance_xy_cm_ = std::hypot(error_x_cm_, error_y_cm_);
  error_yaw_deg_ = normalizeAngleDeg(target_yaw_deg_ - current_yaw_deg_);
  if (has_target_height_) {
    error_z_cm_ = target_z_cm_ - current_z_cm_;
  } else {
    error_z_cm_ = 0.0;
  }
}

bool PositionPIDController::isTargetReached() const
{
  return std::fabs(error_x_cm_) <= position_tolerance_ &&
         std::fabs(error_y_cm_) <= position_tolerance_ &&
         std::fabs(error_yaw_deg_) <= yaw_tolerance_ &&
         (!has_target_height_ || std::fabs(error_z_cm_) <= height_tolerance_);
}

void PositionPIDController::setControlMode(ControlMode mode)
{
  control_mode_ = mode;
  double limit = max_linear_vel_;

  switch (mode) {
    case ControlMode::NORMAL:
      limit = max_linear_vel_;
      break;
    case ControlMode::SLOW:
    case ControlMode::LOCK_Y:
    case ControlMode::LOCK_X:
      limit = max_slow_vel_;
      break;
    case ControlMode::HOVER:
      limit = 10.0;
      break;
  }

  pid_x_.setOutputLimits(limit, -limit);
  pid_y_.setOutputLimits(limit, -limit);
  pid_xy_speed_.setOutputLimits(limit, -limit);
}

std_msgs::msg::Float32MultiArray PositionPIDController::processPID(double dt)
{
  std_msgs::msg::Float32MultiArray cmd;
  cmd.data.resize(4);

  calculateErrors();

  double vel_x_cm = 0.0;
  double vel_y_cm = 0.0;

  switch (control_mode_) {
    case ControlMode::NORMAL:
    case ControlMode::SLOW:
    {
      if (distance_xy_cm_ > 0.1) {
        double speed_cmd = -pid_xy_speed_.calculate(0.0, distance_xy_cm_, dt);
        if (speed_cmd < 0.0) {
          speed_cmd = 0.0;
        }
        const double cos_theta = error_x_cm_ / distance_xy_cm_;
        const double sin_theta = error_y_cm_ / distance_xy_cm_;
        vel_x_cm = speed_cmd * cos_theta;
        vel_y_cm = speed_cmd * sin_theta;
      } else {
        vel_x_cm = 0.0;
        vel_y_cm = 0.0;
      }
      break;
    }
    case ControlMode::LOCK_Y:
      vel_y_cm = pid_y_.calculate(target_y_cm_, current_y_cm_, dt);
      vel_x_cm = 0.4 * pid_x_.calculate(target_x_cm_, current_x_cm_, dt);
      break;
    case ControlMode::LOCK_X:
      vel_x_cm = pid_x_.calculate(target_x_cm_, current_x_cm_, dt);
      vel_y_cm = 0.4 * pid_y_.calculate(target_y_cm_, current_y_cm_, dt);
      break;
    case ControlMode::HOVER:
      vel_x_cm = pid_x_.calculate(target_x_cm_, current_x_cm_, dt);
      vel_y_cm = pid_y_.calculate(target_y_cm_, current_y_cm_, dt);
      break;
  }

  // ★ 2026-10-07：换面必须掉头 180°，但原来 kp_yaw=0、积分上限 integral_limit=10
  //   → 积分最多给 ki*10 = 0.04*10 = 0.4 deg/s，**物理上转不动**（实测飞机
  //   卡在第二个过渡点上方，waypoint_nav 等 dyaw≤20° 永远等不到）。
  //
  //   ★★ 最终版（按现场要求"到第二个过渡点再开转向"）：
  //   **不看误差大小，只看航向目标有没有整跳**。航线在第二个过渡点把目标
  //   航向从 psi0 跳到 psi0+180，位置环一发现"目标跳了 >90°"就进入主动
  //   转向，转到误差 <10° 才退出。其它任何时刻（起飞、逐格扫描、降落）
  //   目标航向都没变，于是航向**完全交回飞控**，行为与改动前一致。
  //   这样不依赖误差门限，也不会被起飞时那个 ~7° 静态误差误触发
  //   （实测第一版就是这么把飞机起飞就转起来的）。
  const double kp_yaw_turn = 1.0;              // 掉头比例增益（deg/s per deg）
  // ★★ 2026-10-07 第二次实测修正：原来掉头直接吃 max_angular_vel_(30deg/s)，
  //   实测 8 秒转完 180°，但**转的过程中雷达/建图算不准位姿**（位置环 XY 指令
  //   全程 ±20cm/s 抖），转完拿着坏位姿继续纠 → 越纠越飘。改成限速 15deg/s
  //   （180° 约 12 秒），给 slam 足够时间跟上，转完再配合航线的 5 秒稳定段。
  // ★★ 第四次实测修正（"第四步转过头"）：反馈航向来自 dead_reckon 的低通，
  //   τ≈1.7s，转速 15deg/s 时"真实航向"比反馈超前 25° → 退出阈值一满足其实已过头。
  //   配合把 yaw_lpf_alpha 从 0.03 提到 0.12（τ→0.42s），这里再把转速降到 10deg/s，
  //   滞后误差 10×0.42≈4°，过头量回到可接受范围。
  const double yaw_turn_max_rate = 10.0;       // 掉头时角速度上限 deg/s
  // ★★ 第三次实测修正：只有"目标跳变"才进掉头模式的话，掉头转完退出后
  //   如果航向误差又涨回来（飞控保持不完美 / SLAM 航向跳变），就**再也不会介入**，
  //   只剩慢积分(≤0.4deg/s) → 实测表现："机体斜着、一直悬停不动"。
  //   所以再加一条**误差兜底触发**：误差 >18°（低于 waypoint_nav 的 20° 判定）
  //   就重新进入掉头模式，转到 <10° 退出。起飞时那个 ~8° 静态误差不会触发。
  // ★★ 第五次修正（现场："飞正方形那次开环转很准，为什么不用那个办法"）：
  //   那次成功靠的是 rotate.py —— **定速定时开环**（发固定角速度×固定时长，
  //   不进反馈环），所以没有"反馈滞后导致过头"的问题。
  //   这里照搬：**目标航向一变，就按 |Δ|/速率 算出该转多久，然后定时发固定角速度**。
  const double yaw_spin_rate_deg = 10.0;       // 开环掉头角速度 deg/s（飞控限幅 ±30）
  const double yaw_spin_trigger_deg = 8.0;     // 目标变化超过这个度数才触发
  const double yaw_turn_trigger_deg = 90.0;    // 目标航向跳变判据
  const double yaw_error_trigger_deg = 18.0;   // 误差兜底判据
  const double yaw_turn_exit_deg = 10.0;       // 转到这个误差内就退出
  const double d_target_signed =
    normalizeAngleDeg(target_yaw_deg_ - last_target_yaw_deg_);
  const double d_target_yaw = std::fabs(d_target_signed);
  if (std::fabs(d_target_signed) > yaw_spin_trigger_deg && has_last_target_yaw_) {
    // 定速定时开环掉头（照搬 rotate.py 那种"很准"的做法）
    yaw_spin_left_s_ = std::fabs(d_target_signed) / yaw_spin_rate_deg;
    yaw_spin_sign_ = (d_target_signed > 0.0) ? 1.0 : -1.0;
    RCLCPP_INFO(get_logger(),
      "航向目标变化 %.1f° -> 定速开环转 %.1f°@%.0fdeg/s，用 %.2fs",
      d_target_signed, d_target_signed, yaw_spin_rate_deg, yaw_spin_left_s_);
  }
  if (d_target_yaw > yaw_turn_trigger_deg) {
    yaw_turning_ = true;
    RCLCPP_INFO(get_logger(),
      "航向目标跳变 %.1f° -> 启动主动掉头（目标=%.1f° 当前=%.1f°）",
      d_target_yaw, target_yaw_deg_, current_yaw_deg_);
  } else if (!yaw_turning_ && std::fabs(error_yaw_deg_) > yaw_error_trigger_deg) {
    yaw_turning_ = true;
    RCLCPP_WARN(get_logger(),
      "航向误差 %.1f° 超阈值 -> 重新介入转向（误差兜底）", error_yaw_deg_);
  } else if (yaw_turning_ && std::fabs(error_yaw_deg_) < yaw_turn_exit_deg) {
    yaw_turning_ = false;
    RCLCPP_INFO(get_logger(), "掉头完成（误差 %.1f°）→ 航向交回飞控",
                error_yaw_deg_);
  }
  last_target_yaw_deg_ = target_yaw_deg_;
  double vel_yaw_deg;
  if (yaw_spin_left_s_ > 0.0) {
    // 开环段：固定角速度，不看反馈
    vel_yaw_deg = yaw_spin_sign_ * yaw_spin_rate_deg;
    yaw_spin_left_s_ -= dt;
    if (yaw_spin_left_s_ <= 0.0) {
      yaw_spin_left_s_ = 0.0;
      RCLCPP_INFO(get_logger(), "定速开环掉头结束 → 航向交回飞控（当前 %.1f°）",
                  current_yaw_deg_);
    }
  } else if (yaw_turning_) {
    vel_yaw_deg = kp_yaw_turn * error_yaw_deg_;
    if (vel_yaw_deg > yaw_turn_max_rate) {
      vel_yaw_deg = yaw_turn_max_rate;
    } else if (vel_yaw_deg < -yaw_turn_max_rate) {
      vel_yaw_deg = -yaw_turn_max_rate;
    }
  } else {
    vel_yaw_deg = pid_yaw_.calculate(0.0, -error_yaw_deg_, dt);
  }
  // 航向死区：误差落在带内就完全不给偏航角速度，避免慢速外环来回追
  // 造成的小幅摆动（实测 0.38Hz / ±4°）。ki_yaw=0 所以清零不会积风。
  if (yaw_deadband_deg_ > 0.0 &&
      std::fabs(error_yaw_deg_) < yaw_deadband_deg_) {
    vel_yaw_deg = 0.0;
  }
  
  double vel_z_cm = 0.0;
  if (has_target_height_) {
    vel_z_cm = pid_z_.calculate(target_z_cm_, current_z_cm_, dt);
  } else {
    // 如果有目标高度但没有高度反馈，打印警告
    if (std::fabs(target_z_cm_) > 1.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Waiting for height data... Z velocity suppressed (Target Z=%.1f)", target_z_cm_);
    }
    vel_z_cm = 0.0;
  }

  const bool vertical_takeoff_mode =
    has_route_current_index_ &&
    route_current_index_ == vertical_takeoff_index_ &&
    target_z_cm_ > vertical_takeoff_min_z_cm_;

  if (vertical_takeoff_mode) {
    const double xy_speed = std::hypot(vel_x_cm, vel_y_cm);
    if (xy_speed > takeoff_max_xy_velocity_ && xy_speed > 0.0) {
      const double scale = takeoff_max_xy_velocity_ / xy_speed;
      vel_x_cm *= scale;
      vel_y_cm *= scale;
    }
  }

  // double vel_z_cm = 0.0;
  // if(has_target_height_)
  // {
  //   double pid_z_output = pid_z_.calculate(target_z_cm_, current_z_cm_, dt);
  //   double err_z = target_z_cm_ - current_z_cm_;
  //   double abs_err_z = std::fabs(err_z);

  //   const double min_err = 2.0; // cm
  //   const double medium_err = 10.0; // cm
  //   const double min_boost = 10.0; // cm/s

  //   if(abs_err_z < min_err)
  //   {
  //     // 误差很小，直接悬停
  //     vel_z_cm = pid_z_output;
  //   }
  //   else if(abs_err_z < medium_err)
  //   {
  //     // 误差中等，减速
  //     vel_z_cm = pid_z_output + min_boost*(err_z > 0 ? 1 : -1);
  //   }
  //   else
  //   {
  //     // 误差较大，正常速度
  //     vel_z_cm = pid_z_output;
  //   }
  //}

  cmd.data[0] = static_cast<float>(vel_x_cm);
  cmd.data[1] = static_cast<float>(vel_y_cm);
  cmd.data[2] = static_cast<float>(vel_z_cm);
  cmd.data[3] = static_cast<float>(vertical_takeoff_mode ? 0.0 : vel_yaw_deg);
  return cmd;
}

void PositionPIDController::controlTimerCallback()
{
  if (!has_target_position_) {
    return;
  }

  if (!getCurrentPose()) {
    return;
  }

  const rclcpp::Time now_time = now();
  double dt = (now_time - last_update_time_).seconds();
  if (dt <= 0.0) {
    dt = 1.0 / std::max(control_frequency_, 1.0);
  }
  last_update_time_ = now_time;

  auto cmd_vel = processPID(dt);
  
  if (is_active_controller_) {
    if (should_stop_) {
      std_msgs::msg::Float32MultiArray stop_cmd;
      stop_cmd.data = {0.0f, 0.0f, 0.0f, 0.0f};
      target_velocity_pub_->publish(stop_cmd);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Drone Stopped (Velocity 0)");
    } else if (is_emergency_landing_) {
      std_msgs::msg::Float32MultiArray emergency_cmd;
      emergency_cmd.data = {0.0f, 0.0f, 100.0f, 0.0f};
      target_velocity_pub_->publish(emergency_cmd);
    } else {
      target_velocity_pub_->publish(cmd_vel);
    }
  }

  if (isTargetReached()) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "Target reached: distance=%.1fcm yaw_error=%.1fdeg",
      distance_xy_cm_, error_yaw_deg_);
  }

  RCLCPP_DEBUG(get_logger(),
    "Current[%.1f, %.1f, %.1fdeg] Target[%.1f, %.1f, %.1fdeg] Error[%.1f, %.1f, %.1fdeg]",
    current_x_cm_, current_y_cm_, current_yaw_deg_,
    target_x_cm_, target_y_cm_, target_yaw_deg_,
    error_x_cm_, error_y_cm_, error_yaw_deg_);
}

void PositionPIDController::loadParameters()
{
  control_frequency_ = declare_parameter<double>("control_frequency", 50.0);
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  laser_link_frame_ = declare_parameter<std::string>("laser_link_frame", "laser_link");
  position_tolerance_ = declare_parameter<double>("position_tolerance", 6.0);
  yaw_tolerance_ = declare_parameter<double>("yaw_tolerance", 5.0);
  yaw_deadband_deg_ = declare_parameter<double>("yaw_deadband_deg", 0.0);
  height_tolerance_ = declare_parameter<double>("height_tolerance", 6.0);
  pose_data_timeout_sec_ = std::max(0.1,
    declare_parameter<double>("pose_data_timeout_sec", 0.5));
  height_min_cm_ = declare_parameter<double>("height_min_cm", 0.0);
  height_max_cm_ = declare_parameter<double>("height_max_cm", 250.0);
  height_filter_jump_threshold_cm_ =
    declare_parameter<double>("height_filter_jump_threshold_cm", 35.0);
  height_filter_required_frames_ =
    declare_parameter<int>("height_filter_required_frames", 5);

  const double kp_xy = declare_parameter<double>("kp_xy", 0.5);
  const double ki_xy = declare_parameter<double>("ki_xy", 0.0);
  const double kd_xy = declare_parameter<double>("kd_xy", 0.1);

  const double kp_yaw = declare_parameter<double>("kp_yaw", 1.0);
  const double ki_yaw = declare_parameter<double>("ki_yaw", 0.0);
  const double kd_yaw = declare_parameter<double>("kd_yaw", 0.2);

  const double kp_z = declare_parameter<double>("kp_z", 0.9);
  const double ki_z = declare_parameter<double>("ki_z", 0.0);
  const double kd_z = declare_parameter<double>("kd_z", 0.15);

  max_linear_vel_ = declare_parameter<double>("max_linear_velocity", 36.0);
  max_angular_vel_ = declare_parameter<double>("max_angular_velocity", 30.0);
  max_vertical_vel_ = declare_parameter<double>("max_vertical_velocity", 40.0);
  max_slow_vel_ = declare_parameter<double>("max_slow_velocity", 20.0);
  vertical_takeoff_index_ = static_cast<uint8_t>(
    declare_parameter<int>("vertical_takeoff_index", 1));
  vertical_takeoff_min_z_cm_ = declare_parameter<double>("vertical_takeoff_min_z_cm", 20.0);
  takeoff_max_xy_velocity_ = declare_parameter<double>("takeoff_max_xy_velocity", 8.0);

  pid_x_.setPID(kp_xy, ki_xy, kd_xy);
  pid_y_.setPID(kp_xy, ki_xy, kd_xy);
  pid_yaw_.setPID(kp_yaw, ki_yaw, kd_yaw);
  pid_z_.setPID(kp_z, ki_z, kd_z);
  pid_xy_speed_.setPID(kp_xy, ki_xy, kd_xy);

  pid_x_.setOutputLimits(max_linear_vel_, -max_linear_vel_);
  pid_y_.setOutputLimits(max_linear_vel_, -max_linear_vel_);
  pid_yaw_.setOutputLimits(max_angular_vel_, -max_angular_vel_);
  pid_z_.setOutputLimits(max_vertical_vel_, -60.0);
  pid_xy_speed_.setOutputLimits(max_linear_vel_, -max_linear_vel_);

  RCLCPP_INFO(get_logger(),
    "PID params: XY(kp=%.2f, ki=%.2f, kd=%.2f) Yaw(kp=%.2f, ki=%.2f, kd=%.2f, deadband=%.1fdeg) "
    "Z(kp=%.2f, ki=%.2f, kd=%.2f)",
    kp_xy, ki_xy, kd_xy, kp_yaw, ki_yaw, kd_yaw, yaw_deadband_deg_, kp_z, ki_z, kd_z);
  RCLCPP_INFO(get_logger(),
    "Tolerances: pos=%.1fcm yaw=%.1fdeg height=%.1fcm",
    position_tolerance_, yaw_tolerance_, height_tolerance_);
  RCLCPP_INFO(get_logger(),
    "Velocity limits: linear=%.1fcm/s angular=%.1fdeg/s vertical=%.1fcm/s slow=%.1fcm/s",
    max_linear_vel_, max_angular_vel_, max_vertical_vel_, max_slow_vel_);
  RCLCPP_INFO(get_logger(),
    "Vertical takeoff: route_index=%u min_target_z=%.1fcm max_xy=%.1fcm/s",
    static_cast<unsigned>(vertical_takeoff_index_), vertical_takeoff_min_z_cm_,
    takeoff_max_xy_velocity_);
}

}  // namespace pid_control_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pid_control_pkg::PositionPIDController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
