#include "activity_control_pkg/route_target_publisher.hpp"
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <angles/angles.h>
#include <clocale>
#include <cmath>
#include <limits>
#include <algorithm>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#define fly_start 1


namespace activity_control_pkg
{

namespace
{
constexpr double kDefaultTimerPeriodSec = 0.05;
}  // namespace

RouteTargetPublisherNode::RouteTargetPublisherNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("route_target_publisher", options),
  current_idx_(std::numeric_limits<std::size_t>::max()),
  has_height_(false),
  current_height_cm_(0.0)
{
  pos_tol_cm_ = declare_parameter("position_tolerance_cm", 20.0);
  yaw_tol_deg_ = declare_parameter("yaw_tolerance_deg", 5.0);
  height_tol_cm_ = declare_parameter("height_tolerance_cm", 12.0);
  map_frame_ = declare_parameter("map_frame", "map");
  laser_link_frame_ = declare_parameter("laser_link_frame", "laser_link");
  output_topic_ = declare_parameter("output_topic", "/target_position");

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  target_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(output_topic_, qos);
  active_controller_pub_ = create_publisher<std_msgs::msg::UInt8>("/active_controller", qos);
  reached_waypoint_pub_ = create_publisher<std_msgs::msg::UInt8>("/route_reached_index", 10);
  current_waypoint_pub_ = create_publisher<std_msgs::msg::UInt8>("/route_current_index", qos);

  height_sub_ = create_subscription<std_msgs::msg::Int16>(
    "/height", rclcpp::QoS(10),
    std::bind(&RouteTargetPublisherNode::heightCallback, this, std::placeholders::_1));

  bluetooth_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
      "/bluetooth_data", 10,
      std::bind(&RouteTargetPublisherNode::bluetooth_data_callback, this, std::placeholders::_1));

  monitor_timer_ = create_wall_timer(
    std::chrono::duration<double>(kDefaultTimerPeriodSec),
    std::bind(&RouteTargetPublisherNode::monitorTimerCallback, this));

  RCLCPP_INFO(get_logger(),
    "RouteTargetPublisher initialized: map=%s laser_link=%s topic=%s", map_frame_.c_str(),
    laser_link_frame_.c_str(), output_topic_.c_str());
  RCLCPP_INFO(get_logger(),
    "Tolerances: position=%.1fcm yaw=%.1fdeg height=%.1fcm",
    pos_tol_cm_, yaw_tol_deg_, height_tol_cm_);
}

void RouteTargetPublisherNode::addTarget(const Target & target)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const bool was_empty = targets_.empty();
  targets_.push_back(target);
  if (was_empty) {
    current_idx_ = 0;
    publishCurrent();
  }
}

bool RouteTargetPublisherNode::replaceTarget(std::size_t index, const Target & target)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (index >= targets_.size()) {
    return false;
  }
  targets_[index] = target;
  if (current_idx_ == index) {
    publishCurrent();
  }
  return true;
}

bool RouteTargetPublisherNode::queryCurrentPose(double & x_cm, double & y_cm, double & z_cm, double & yaw_deg)
{
  return getCurrentPose(x_cm, y_cm, z_cm, yaw_deg);
}

bool RouteTargetPublisherNode::publishHoldTarget(const Target & target)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (current_idx_ == std::numeric_limits<std::size_t>::max() || current_idx_ >= targets_.size()) {
    return false;
  }

  publishTarget(target, false);
  return true;
}

bool RouteTargetPublisherNode::skipCurrentTarget()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (current_idx_ == std::numeric_limits<std::size_t>::max() || current_idx_ >= targets_.size()) {
    return false;
  }

  current_idx_++;
  if (current_idx_ < targets_.size()) {
    publishCurrent();
  } else {
    current_idx_ = targets_.size();
    std_msgs::msg::UInt8 active_msg;
    active_msg.data = 3;
    active_controller_pub_->publish(active_msg);
  }
  return true;
}

std::size_t RouteTargetPublisherNode::currentIndex() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return current_idx_;
}

std::size_t RouteTargetPublisherNode::size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return targets_.size();
}

void RouteTargetPublisherNode::publishCurrent()
{
  if (current_idx_ != std::numeric_limits<std::size_t>::max() && current_idx_ < targets_.size()) {
    publishTarget(targets_[current_idx_], current_idx_ == 0);
  }
}

void RouteTargetPublisherNode::publishTarget(const Target & target, bool init_flag)
{
  std_msgs::msg::Float32MultiArray message;
  message.data.resize(4);
  message.data[0] = static_cast<float>(target.x_cm);
  message.data[1] = static_cast<float>(target.y_cm);
  message.data[2] = static_cast<float>(target.z_cm);
  message.data[3] = static_cast<float>(target.yaw_deg);

  std_msgs::msg::UInt8 current_msg;
  current_msg.data = static_cast<uint8_t>(current_idx_);
  current_waypoint_pub_->publish(current_msg);
  target_pub_->publish(message);

  std_msgs::msg::UInt8 active_msg;
  if (current_idx_ == 0) {
    active_msg.data = 1; // Car
    RCLCPP_INFO(get_logger(), "激活控制器: 车 (Car)");
  } else if(current_idx_ >= fly_start) {
    active_msg.data = 2; // Drone
    RCLCPP_INFO(get_logger(), "激活控制器: 飞机 (Drone)");
  }
  active_controller_pub_->publish(active_msg);

  RCLCPP_INFO(get_logger(),
    "发布目标: x=%.1fcm y=%.1fcm z=%.1fcm yaw=%.1fdeg%s",
    target.x_cm, target.y_cm, target.z_cm, target.yaw_deg,
    init_flag ? " (首个)" : "");
}

void RouteTargetPublisherNode::heightCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  current_height_cm_ = static_cast<double>(msg->data);
  has_height_ = true;
}

void RouteTargetPublisherNode::bluetooth_data_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
    control_mode_ = msg->data[0];
    if (control_mode_ == 1) {
      RCLCPP_INFO(this->get_logger(), "Received control mode 1.");
    } else {
      RCLCPP_INFO(this->get_logger(), "Received control mode %d.", control_mode_);
    }
  }


bool RouteTargetPublisherNode::getCurrentPose(double & x_cm, double & y_cm, double & z_cm, double & yaw_deg)
{
  try {
    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
      map_frame_, laser_link_frame_, tf2::TimePointZero);
    x_cm = meterToCm(transform.transform.translation.x);
    y_cm = meterToCm(transform.transform.translation.y);
    z_cm = has_height_ ? current_height_cm_ : 0.0;
    tf2::Quaternion q;
    tf2::fromMsg(transform.transform.rotation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    yaw_deg = radToDeg(yaw);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "TF 查询失败 (%s->%s): %s", map_frame_.c_str(), laser_link_frame_.c_str(), ex.what());
    return false;
  }
}

// bool RouteTargetPublisherNode::isReached(
//   const Target & target,
//   double x_cm,
//   double y_cm,
//   double z_cm,
//   double yaw_deg) const
// {
//   const double dx = target.x_cm - x_cm;
//   const double dy = target.y_cm - y_cm;
//   const double dxy = std::hypot(dx, dy);
//   const double dz = target.z_cm - z_cm;
//   const double dyaw = normalizeAngleDeg(target.yaw_deg - yaw_deg);
//   // 容忍度动态选择
//   const double z_tol = ever_received_st_ready_ ? height_tol_cm_ : 20.0;            // 飞机： 6 cm   车：20 cm 
//   const bool z_ok = (std::fabs(dz) <= z_tol);
//   // const bool z_ok = has_height_ ? (std::fabs(dz) <= height_tol_cm_) : true;
//   return z_ok && (dxy <= pos_tol_cm_) && (std::fabs(dyaw) <= yaw_tol_deg_);
// }

bool RouteTargetPublisherNode::isReached(
  const Target & target,
  double x_cm,
  double y_cm,
  double z_cm,
  double yaw_deg) const
{
  const double dx = target.x_cm - x_cm;
  const double dy = target.y_cm - y_cm;
  const double dxy = std::hypot(dx, dy);
  const double dz = target.z_cm - z_cm;
  const double dyaw = normalizeAngleDeg(target.yaw_deg - yaw_deg);
  
  // 车(idx < fly_start)的时候容忍度大(20.0)，飞机的时候容忍度小(height_tol_cm_)
  const double z_tol = (current_idx_ < fly_start) ? 20.0 : height_tol_cm_;
  const double xy_tol = (current_idx_ < fly_start) ? 15.0 : pos_tol_cm_;
  
  const bool z_ok = (std::fabs(dz) <= z_tol);
  const bool xy_ok = (dxy <= xy_tol);
  const bool yaw_ok = (std::fabs(dyaw) <= yaw_tol_deg_);

  // 如果目标Z值大于一个阈值（比如20cm），说明这是一个起飞或空中航点
  // 这种情况下，我们放宽对XY和Yaw的要求，只要高度差不多就认为到达
  if (target.z_cm > 20.0) {
    // 起飞点是跳过地面车后的第一个飞行目标，只看高度，避免起飞时 TF/SLAM 水平漂移卡住流程。
    if (current_idx_ == fly_start) {
        return z_ok;
    }
    // 对于空中的航点，只要高度和水平位置都差不多就行，暂时忽略yaw
    return z_ok && xy_ok;
  }

  // 对于Z值很低（比如降落）或为0的航点，要求所有条件都满足
  return z_ok && xy_ok && yaw_ok;
}

void RouteTargetPublisherNode::monitorTimerCallback()
{
  std::lock_guard<std::mutex> lock(mutex_);
  
  // 修改：如果已经完成所有目标，持续发送停止信号，确保PID控制器收到
  if (current_idx_ != std::numeric_limits<std::size_t>::max() && current_idx_ >= targets_.size()) {
    std_msgs::msg::UInt8 active_msg;
    active_msg.data = 3; // Drone Stop
    active_controller_pub_->publish(active_msg);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "所有目标已完成，持续发送停止信号(3)");
    return;
  }

  if (current_idx_ == std::numeric_limits<std::size_t>::max()) {
    return;
  }

  double x_cm = 0.0;
  double y_cm = 0.0;
  double z_cm = 0.0;
  double yaw_deg = 0.0;
  if (!getCurrentPose(x_cm, y_cm, z_cm, yaw_deg)) {
    return;
  }

  const Target & target = targets_[current_idx_];
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 5000,
    "当前目标 %zu: x=%.1f,y=%.1f,z=%.1f,yaw=%.1f",
    current_idx_, target.x_cm, target.y_cm, target.z_cm, target.yaw_deg
  );
  if (isReached(target, x_cm, y_cm, z_cm, yaw_deg)) {
    std_msgs::msg::UInt8 reached_msg;
    reached_msg.data = static_cast<uint8_t>(current_idx_);
    reached_waypoint_pub_->publish(reached_msg);

    RCLCPP_INFO(get_logger(),
      "目标 %zu 已完成，准备下一个", current_idx_);
    current_idx_++;
    if (current_idx_ < targets_.size()) {
      publishCurrent();
    } else {
      current_idx_ = targets_.size();
      RCLCPP_INFO(get_logger(), "所有目标已完成");
      
      // 发布停止信号
      std_msgs::msg::UInt8 active_msg;
      active_msg.data = 3; // Drone Stop
      active_controller_pub_->publish(active_msg);
    }
  }
}

double RouteTargetPublisherNode::meterToCm(double value_m)
{
  return value_m * 100.0;
}

double RouteTargetPublisherNode::radToDeg(double value_rad)
{
  return value_rad * 180.0 / M_PI;
}

double RouteTargetPublisherNode::normalizeAngleDeg(double angle_deg) const
{
  const double normalized = angles::normalize_angle(angles::from_degrees(angle_deg));
  return angles::to_degrees(normalized);
}

// RouteTestNode::RouteTestNode(
//   const std::shared_ptr<RouteTargetPublisherNode> & route_node,
//   const rclcpp::NodeOptions & options)
// : rclcpp::Node("route_test_node", options),
//   route_node_(route_node),
//   started_(false),
//   next_target_index_(1)
// {
//   std::setlocale(LC_ALL, "");

//   ready_sub_ = create_subscription<std_msgs::msg::UInt8>(
//     "/is_st_ready", rclcpp::QoS(10),
//     std::bind(&RouteTestNode::readyCallback, this, std::placeholders::_1));

//   add_timer_ = create_wall_timer(
//     std::chrono::seconds(1),
//     std::bind(&RouteTestNode::addTimerCallback, this));
//   add_timer_->cancel();

//   RCLCPP_INFO(get_logger(), "Route test node ready. 等待 /is_st_ready == 1");
// }

// void RouteTestNode::readyCallback(const std_msgs::msg::UInt8::SharedPtr msg)
// {
//   if (msg->data == 1 && !started_) {
//     Target first{0.0, 0.0, 140, 0.0};
//     route_node_->addTarget(first);
//     const auto current = route_node_->currentIndex();
//     RCLCPP_INFO(get_logger(),
//       "收到 /is_st_ready=1，添加首个目标: x=%.1f y=%.1f z=%.1f yaw=%.1f | 当前第 %zu 个目标",
//       first.x_cm, first.y_cm, first.z_cm, first.yaw_deg,
//       (current == std::numeric_limits<std::size_t>::max() ? 0 : current + 1));
//     add_timer_->reset();
//     started_ = true;
//   } else if (!started_) {
//     RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
//       "/is_st_ready=%u，等待为1", static_cast<unsigned>(msg->data));
//   }
// }

RouteTestNode::RouteTestNode(
  const std::shared_ptr<RouteTargetPublisherNode> & route_node,
  const rclcpp::NodeOptions & options)
: rclcpp::Node("route_test_node", options),
  route_node_(route_node),
  started_(false),
  next_target_index_(1),
  use_two_second_capture_mode_(true),
  two_second_capture_done_(false),
  capture_stop_issued_(false),
  elapsed_ticks_(0),
  first_drive_ticks_(50),
  add_interval_ticks_(10),
  capture_settle_ticks_(3),
  capture_settle_elapsed_ticks_(0),
  first_forward_x_cm_(300.0),
  first_forward_y_cm_(0.0),
  first_forward_z_cm_(4.0),
  takeoff_height_cm_(100.0),
  capture_last_z_cm_(0.0),
  capture_last_yaw_deg_(0.0),
  capture_window_size_(7)
{
  std::setlocale(LC_ALL, "");

  use_two_second_capture_mode_ = declare_parameter("use_two_second_capture_mode", true);
  const int first_drive_ticks_param = static_cast<int>(declare_parameter("first_drive_ticks", 50));
  first_drive_ticks_ = std::max(1, first_drive_ticks_param);
  first_forward_x_cm_ = declare_parameter("first_forward_x_cm", 300.0);
  first_forward_y_cm_ = declare_parameter("first_forward_y_cm", 0.0);
  first_forward_z_cm_ = declare_parameter("first_forward_z_cm", 4.0);
  takeoff_height_cm_ = declare_parameter("takeoff_height_cm", 100.0);
  const int capture_settle_ticks_param =
    static_cast<int>(declare_parameter("capture_settle_ticks", 3));
  capture_settle_ticks_ = std::max(0, capture_settle_ticks_param);
  const int capture_window_size_param = declare_parameter("capture_window_size", 7);
  capture_window_size_ = static_cast<std::size_t>(std::max(3, capture_window_size_param));

  bluetooth_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
      "/bluetooth_data", 10,
      std::bind(&RouteTestNode::bluetoothCallback, this, std::placeholders::_1));

  // 创建定时器，但先不启动
  add_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&RouteTestNode::addTimerCallback, this));
  add_timer_->cancel();

  RCLCPP_INFO(get_logger(),
    "Route test node 就绪，等待蓝牙指令 (data[0] == 1) 开始任务... mode=%s",
    use_two_second_capture_mode_ ? "2s_capture" : "legacy");
}

void RouteTestNode::bluetoothCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
  if (started_) {
    return;
  }

  if (!msg->data.empty() && msg->data[0] == 1) {
    RCLCPP_INFO(get_logger(), "收到蓝牙指令 1，开始执行航点任务！");

    Target first{first_forward_x_cm_, first_forward_y_cm_, first_forward_z_cm_, 0.0};
    route_node_->addTarget(first);

    recent_x_cm_.clear();
    recent_y_cm_.clear();
    elapsed_ticks_ = 0;
    two_second_capture_done_ = false;
    capture_stop_issued_ = false;
    capture_settle_elapsed_ticks_ = 0;
    capture_last_z_cm_ = 0.0;
    capture_last_yaw_deg_ = 0.0;
    next_target_index_ = use_two_second_capture_mode_ ? 2 : 1;
    
    const auto current = route_node_->currentIndex();
    RCLCPP_INFO(get_logger(),
      "添加首个目标: x=%.1f y=%.1f z=%.1f yaw=%.1f | 当前第 %zu 个目标",
      first.x_cm, first.y_cm, first.z_cm, first.yaw_deg,
      (current == std::numeric_limits<std::size_t>::max() ? 0 : current + 1));

    add_timer_->reset();
    started_ = true;
  }
}




void RouteTestNode::addTimerCallback()
{
  if (!started_) {
    return;
  }

  // 每100ms回调，10次约等于1秒
  elapsed_ticks_++;

  if (use_two_second_capture_mode_ && !two_second_capture_done_) {
    if (elapsed_ticks_ < first_drive_ticks_) {
      return;
    }

    if (!capture_stop_issued_) {
      double stop_x_cm = 0.0, stop_y_cm = 0.0, stop_z_cm = 0.0, stop_yaw_deg = 0.0;
      if (!route_node_->queryCurrentPose(stop_x_cm, stop_y_cm, stop_z_cm, stop_yaw_deg)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "2秒后等待 TF 成功，再发布停车目标并开始平均采样");
        return;
      }

      const Target stop_target{stop_x_cm, stop_y_cm, stop_z_cm, stop_yaw_deg};
      if (!route_node_->publishHoldTarget(stop_target)) {
        RCLCPP_WARN(get_logger(), "停车目标发布失败，当前索引状态异常");
        return;
      }

      recent_x_cm_.clear();
      recent_y_cm_.clear();
      capture_last_z_cm_ = stop_z_cm;
      capture_last_yaw_deg_ = stop_yaw_deg;
      capture_stop_issued_ = true;
      capture_settle_elapsed_ticks_ = 0;
      RCLCPP_INFO(get_logger(),
        "2秒到达，先停车并等待稳定: stop x=%.1f y=%.1f z=%.1f yaw=%.1f settle_ticks=%d samples=%zu",
        stop_target.x_cm, stop_target.y_cm, stop_target.z_cm, stop_target.yaw_deg,
        capture_settle_ticks_, capture_window_size_);
      return;
    }

    if (capture_settle_elapsed_ticks_ < capture_settle_ticks_) {
      capture_settle_elapsed_ticks_++;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
        "停车稳定等待中: %d/%d",
        capture_settle_elapsed_ticks_, capture_settle_ticks_);
      return;
    }

    double x_cm = 0.0, y_cm = 0.0, z_cm = 0.0, yaw_deg = 0.0;
    if (!route_node_->queryCurrentPose(x_cm, y_cm, z_cm, yaw_deg)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "停车后等待 TF 平均采样成功，不使用默认位姿");
      return;
    }

    recent_x_cm_.push_back(x_cm);
    recent_y_cm_.push_back(y_cm);
    capture_last_z_cm_ = z_cm;
    capture_last_yaw_deg_ = yaw_deg;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      "停车后 TF 采样中: %zu/%zu latest x=%.1f y=%.1f",
      recent_x_cm_.size(), capture_window_size_, x_cm, y_cm);

    if (recent_x_cm_.size() < capture_window_size_) {
      return;
    }

    double avg_x_cm = 0.0, avg_y_cm = 0.0;
    if (!computeAverageXY(avg_x_cm, avg_y_cm)) {
      RCLCPP_WARN(get_logger(), "TF 平均采样窗口不足，继续等待");
      return;
    }

    const double second_z_cm = std::max(0.0, capture_last_z_cm_ + takeoff_height_cm_);
    const Target second_takeoff{avg_x_cm, avg_y_cm, second_z_cm, capture_last_yaw_deg_};
    route_node_->addTarget(second_takeoff);
    RCLCPP_INFO(get_logger(),
      "添加第2目标(停车后平均位姿原地起飞+100cm): x=%.1f y=%.1f z=%.1f yaw=%.1f samples=%zu",
      second_takeoff.x_cm, second_takeoff.y_cm, second_takeoff.z_cm, second_takeoff.yaw_deg,
      recent_x_cm_.size());

    const Target third_fly{300.0, 250.0, 100.0, 0.0};
    route_node_->addTarget(third_fly);
    RCLCPP_INFO(get_logger(),
      "添加第3目标(固定巡航点): x=%.1f y=%.1f z=%.1f yaw=%.1f",
      third_fly.x_cm, third_fly.y_cm, third_fly.z_cm, third_fly.yaw_deg);

    if (!route_node_->skipCurrentTarget()) {
      RCLCPP_WARN(get_logger(), "首航点强制切换失败，当前索引状态异常");
      return;
    }

    two_second_capture_done_ = true;
    elapsed_ticks_ = 0;
    next_target_index_ = 3;
    return;
  }

  // 兼容原逻辑：按约1秒节奏继续追加后续航点
  if (elapsed_ticks_ % add_interval_ticks_ != 0) {
    return;
  }

  Target target{};
  switch (next_target_index_) {
    case 1:
      target = Target{first_forward_x_cm_, first_forward_y_cm_, takeoff_height_cm_, 0.0};
      break;
    case 2:
      target = Target{300.0, 250.0, 100.0, 0.0};
      break;
    case 3:
      target = Target{300.0, 150.0, 100.0, 0.0};
      break;
    case 4:
      target  = Target{0.0, 0.0, 100.0, 0.0};
      break;
    case 5:
      target  = Target{0.0, 0.0, 6.0, 0.0};
      break;
    default:
      add_timer_->cancel();
      RCLCPP_INFO(get_logger(), "预设目标全部添加完毕");
      return;
  }

  route_node_->addTarget(target);
  const auto current = route_node_->currentIndex();
  RCLCPP_INFO(get_logger(),
    "追加目标 idx=%d: x=%.1f y=%.1f z=%.1f yaw=%.1f | 当前第 %zu 个目标",
    next_target_index_, target.x_cm, target.y_cm, target.z_cm, target.yaw_deg,
    (current == std::numeric_limits<std::size_t>::max() ? 0 : current + 1));

  ++next_target_index_;
}

bool RouteTestNode::computeAverageXY(double & x_cm, double & y_cm) const
{
  if (recent_x_cm_.size() < 3 || recent_y_cm_.size() < 3) {
    return false;
  }

  double sum_x_cm = 0.0;
  double sum_y_cm = 0.0;
  for (std::size_t i = 0; i < recent_x_cm_.size(); ++i) {
    sum_x_cm += recent_x_cm_[i];
    sum_y_cm += recent_y_cm_[i];
  }

  x_cm = sum_x_cm / static_cast<double>(recent_x_cm_.size());
  y_cm = sum_y_cm / static_cast<double>(recent_y_cm_.size());
  return true;
}

}  // namespace activity_control_pkg
