/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class GripperSubPublisher : public rclcpp::Node
{
public:
  GripperSubPublisher()
  : Node("gripper_sub_publisher")
  {
    gripper_joint_sub_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("gripper_sub_position/commands", 1);
    gripper_joint_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "gripper_position/commands", 1, std::bind(&GripperSubPublisher::gripperJointCallback, this, std::placeholders::_1));
  }

private:
  void gripperJointCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    gripper_joint_sub_pub_->publish(*msg);
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_joint_sub_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_joint_sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GripperSubPublisher>());
  rclcpp::shutdown();
  return 0;
}