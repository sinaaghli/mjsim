#ifndef MJSIM_H__
#define MJSIM_H__

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "rclcpp/parameter_client.hpp"
#include "mjsim_data.h"
#include <sensor_msgs/msg/joint_state.hpp>
#include <stdexcept>

using namespace std::chrono_literals;

class mjsim : public rclcpp::Node
{
public:
  mjsim(std::shared_ptr<mjsim_data> shared_data)
  : Node("mjsim"), data_handler_(shared_data)
  {
    robot_description_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/robot_description", 10, std::bind(&mjsim::RobotDescriptionCallback, this, std::placeholders::_1));

    this->declare_parameter<std::string>("urdf_file_path", "");
    this->get_parameter("urdf_file_path", urdf_file_path);
    this->declare_parameter<int>("joint_states_pub_freq_hz", 0);
    this->get_parameter("joint_states_pub_freq_hz", jstate_freq_);

    if(!urdf_file_path.empty()){
      data_handler_->UpdateURDFPath(urdf_file_path);
    }

    jstates_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    if(jstate_freq_>0) {
      auto period = std::chrono::milliseconds(static_cast<int>(1000 / jstate_freq_));
      jstate_timer_ = this->create_wall_timer(
        period, std::bind(&mjsim::PublishJointStates, this));
    }
  }

  void PublishJointStates() {
    auto message = sensor_msgs::msg::JointState();
    for (const auto& joint : data_handler_->GetJointStates()) {
        message.name.push_back(joint.jname);
        message.position.push_back(joint.jposition);
        message.velocity.push_back(joint.jvelocity);
        message.effort.push_back(joint.jtorque);
    }
    jstates_publisher_->publish(message);
  }

private:
  void RobotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received URDF description");
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  std::shared_ptr<mjsim_data> data_handler_;
  std::string urdf_file_path;
  int jstate_freq_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jstates_publisher_;
  rclcpp::TimerBase::SharedPtr jstate_timer_;
};

#endif //MJSIM_H__