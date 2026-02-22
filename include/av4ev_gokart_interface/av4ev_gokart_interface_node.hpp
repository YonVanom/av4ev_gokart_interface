#ifndef AV4EV_GOKART_INTERFACE__AV4EV_GOKART_INTERFACE_NODE_HPP_
#define AV4EV_GOKART_INTERFACE__AV4EV_GOKART_INTERFACE_NODE_HPP_

#include <cstdint>
#include <string>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "autoware_control_msgs/msg/control.hpp"
#include "autoware_vehicle_msgs/msg/control_mode_report.hpp"
#include "autoware_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_vehicle_msgs/msg/velocity_report.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class AV4EVGokartInterfaceNode : public rclcpp::Node
{
public:
  AV4EVGokartInterfaceNode();

private:
  void onControlCmd(const autoware_control_msgs::msg::Control::SharedPtr msg);
  void onGokartMode(const std_msgs::msg::Int32::SharedPtr msg);
  void onDriveInfoFromNucleo(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);

  ackermann_msgs::msg::AckermannDriveStamped convertControlCmdToNucleoCommand(
    const autoware_control_msgs::msg::Control & control_cmd) const;

  autoware_vehicle_msgs::msg::ControlModeReport convertGokartModeToControlMode(
    const std_msgs::msg::Int32 & gokart_mode) const;

  autoware_vehicle_msgs::msg::VelocityReport convertDriveInfoToVelocityStatus(
    const ackermann_msgs::msg::AckermannDriveStamped & drive_info) const;

  autoware_vehicle_msgs::msg::SteeringReport convertDriveInfoToSteeringStatus(
    const ackermann_msgs::msg::AckermannDriveStamped & drive_info) const;

  rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr control_cmd_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_info_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gokart_mode_sub_;

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr autonomous_command_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr
    control_mode_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr
    steering_status_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr
    velocity_status_pub_;

  double wheelbase_m_{1.6};

  std::string control_cmd_topic_{"/control/command/control_cmd"};
  std::string drive_info_topic_{"/drive_info_from_nucleo"};
  std::string gokart_mode_topic_{"/gokart_mode"};
  std::string autonomous_command_topic_{"/automous_command_to_nucleo"};
  std::string control_mode_topic_{"/vehicle/status/control_mode"};
  std::string steering_status_topic_{"/vehicle/status/steering_status"};
  std::string velocity_status_topic_{"/vehicle/status/velocity_status"};
};

#endif  // AV4EV_GOKART_INTERFACE__AV4EV_GOKART_INTERFACE_NODE_HPP_
