#ifndef MJSIM_H__
#define MJSIM_H__

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "rclcpp/parameter_client.hpp"
#include "mjsim_data.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
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

    if(!urdf_file_path.empty()){
      data_handler_->UpdateURDFPath(urdf_file_path);
    }
  }

private:
  void RobotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received URDF description");
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  std::shared_ptr<mjsim_data> data_handler_;
  std::string urdf_file_path;
};

#endif //MJSIM_H__