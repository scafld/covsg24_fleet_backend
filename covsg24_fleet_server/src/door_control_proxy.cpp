#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rmf_door_msgs/msg/door_request.hpp"

class DoorProxy : public rclcpp::Node
{
  public:
    DoorProxy()
    : Node("rmf_door_proxy")
    {
      door_request_sub_ = create_subscription<rmf_door_msgs::msg::DoorRequest>("rmf_door_request"
      , 10
      , std::bind(&DoorProxy::rmfDoorRequestCallback, this, std::placeholders::_1));

      door_request_pub_ = this->create_publisher<std_msgs::msg::String>("rmf_door_request_umrf", 10);
    }

  private:
    void rmfDoorRequestCallback(const rmf_door_msgs::msg::DoorRequest::SharedPtr msg)
    {
      
    }

    bool sendDoorOpen(std::string door_name)
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world!";
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      door_request_pub_->publish(message);
      return true;
    }
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr door_request_pub_;
    rclcpp::Subscription<rmf_door_msgs::msg::DoorRequest>::SharedPtr door_request_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DoorProxy>());
  rclcpp::shutdown();
  return 0;
}