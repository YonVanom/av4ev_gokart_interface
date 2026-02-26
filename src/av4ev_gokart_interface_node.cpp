#include <cmath>
#include <memory>

#include "av4ev_gokart_interface/av4ev_gokart_interface_node.hpp"

AV4EVGokartInterfaceNode::AV4EVGokartInterfaceNode()
: Node("av4ev_gokart_interface_node")
{
  using std::placeholders::_1;

  wheelbase_m_ = this->declare_parameter<double>("wheelbase_m", wheelbase_m_);
  control_cmd_topic_ = this->declare_parameter<std::string>("control_cmd_topic", control_cmd_topic_);
  drive_info_topic_ = this->declare_parameter<std::string>("drive_info_topic", drive_info_topic_);
  gokart_mode_topic_ = this->declare_parameter<std::string>("gokart_mode_topic", gokart_mode_topic_);
  autonomous_command_topic_ =
    this->declare_parameter<std::string>("autonomous_command_topic", autonomous_command_topic_);
  control_mode_topic_ = this->declare_parameter<std::string>("control_mode_topic", control_mode_topic_);
  steering_status_topic_ =
    this->declare_parameter<std::string>("steering_status_topic", steering_status_topic_);
  velocity_status_topic_ =
    this->declare_parameter<std::string>("velocity_status_topic", velocity_status_topic_);

  control_cmd_sub_ = this->create_subscription<autoware_control_msgs::msg::Control>(
    control_cmd_topic_, rclcpp::QoS{1},
    std::bind(&AV4EVGokartInterfaceNode::onControlCmd, this, _1));

  drive_info_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
    drive_info_topic_, rclcpp::QoS{1},
    std::bind(&AV4EVGokartInterfaceNode::onDriveInfoFromNucleo, this, _1));

  gokart_mode_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    gokart_mode_topic_, rclcpp::QoS{1},
    std::bind(&AV4EVGokartInterfaceNode::onGokartMode, this, _1));

  autonomous_command_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
    autonomous_command_topic_, rclcpp::QoS{1});

  control_mode_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>(
    control_mode_topic_, rclcpp::QoS{1});

  steering_status_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::SteeringReport>(
    steering_status_topic_, rclcpp::QoS{1});

  velocity_status_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::VelocityReport>(
    velocity_status_topic_, rclcpp::QoS{1});
}

void AV4EVGokartInterfaceNode::onControlCmd(
  const autoware_control_msgs::msg::Control::SharedPtr msg)
{
  auto out = convertControlCmdToNucleoCommand(*msg);
  autonomous_command_pub_->publish(out);
}

void AV4EVGokartInterfaceNode::onGokartMode(const std_msgs::msg::Int32::SharedPtr msg)
{
  auto out = convertGokartModeToControlMode(*msg);
  control_mode_pub_->publish(out);
}

void AV4EVGokartInterfaceNode::onDriveInfoFromNucleo(
  const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
  auto velocity = convertDriveInfoToVelocityStatus(*msg);
  auto steering = convertDriveInfoToSteeringStatus(*msg);

  velocity_status_pub_->publish(velocity);
  steering_status_pub_->publish(steering);
}

ackermann_msgs::msg::AckermannDriveStamped AV4EVGokartInterfaceNode::convertControlCmdToNucleoCommand(
  const autoware_control_msgs::msg::Control & control_cmd) const
{
  ackermann_msgs::msg::AckermannDriveStamped out;

  out.drive.speed = control_cmd.longitudinal.velocity;
  out.drive.steering_angle = control_cmd.lateral.steering_tire_angle;
  out.header.stamp = control_cmd.stamp;
  out.header.frame_id = "base_link";

  return out;
}

autoware_vehicle_msgs::msg::ControlModeReport AV4EVGokartInterfaceNode::convertGokartModeToControlMode(
  const std_msgs::msg::Int32 & gokart_mode) const
{
  autoware_vehicle_msgs::msg::ControlModeReport out;

  if (gokart_mode.data == 1) {
    out.mode = autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
  } else {
    // Map 0 and 2 to MANUAL. Any unexpected value also falls back to MANUAL.
    out.mode = autoware_vehicle_msgs::msg::ControlModeReport::MANUAL;
  }
  out.stamp = this->now();

  return out;
}

autoware_vehicle_msgs::msg::VelocityReport AV4EVGokartInterfaceNode::convertDriveInfoToVelocityStatus(
  const ackermann_msgs::msg::AckermannDriveStamped & drive_info) const
{
  autoware_vehicle_msgs::msg::VelocityReport out;

  // Kinematic bicycle model at rear-axle-centered base_link:
  // yaw_rate = v / L * tan(delta)
  const auto v_measured = static_cast<double>(drive_info.drive.speed);
  const auto steering_angle = static_cast<double>(drive_info.drive.steering_angle);

  double yaw_rate = 0.0;
  if (std::abs(wheelbase_m_) > 1e-6) {
    yaw_rate = (v_measured / wheelbase_m_) * std::tan(steering_angle);
  }

  out.longitudinal_velocity = static_cast<float>(v_measured);
  out.lateral_velocity = 0.0F;
  out.heading_rate = static_cast<float>(yaw_rate);
  out.header = drive_info.header;
  out.header.frame_id = "base_link";

  return out;
}

autoware_vehicle_msgs::msg::SteeringReport AV4EVGokartInterfaceNode::convertDriveInfoToSteeringStatus(
  const ackermann_msgs::msg::AckermannDriveStamped & drive_info) const
{
  autoware_vehicle_msgs::msg::SteeringReport out;

  out.steering_tire_angle = drive_info.drive.steering_angle;
  out.stamp = drive_info.header.stamp;

  return out;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AV4EVGokartInterfaceNode>());
  rclcpp::shutdown();
  return 0;
}
