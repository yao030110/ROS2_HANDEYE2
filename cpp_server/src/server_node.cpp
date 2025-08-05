#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

// 必须添加这行：正确引入placeholders
using std::placeholders::_1;
using std::placeholders::_2;

class MinimalService : public rclcpp::Node
{
public:
  MinimalService() : Node("minimal_service")
  {
    service_ = this->create_service<std_srvs::srv::Trigger>(
      "simple_service", 
      std::bind(&MinimalService::handle_service, this, _1, _2)
    );
    RCLCPP_INFO(this->get_logger(), "Service ready.");
  }

private:
  void handle_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;
    RCLCPP_INFO(this->get_logger(), "Received request.");
    response->success = true;
    response->message = "Service triggered successfully!";
    RCLCPP_INFO(this->get_logger(), "Sent response: %s", response->message.c_str());
  }
  
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto service = std::make_shared<MinimalService>();
  rclcpp::spin(service);
  rclcpp::shutdown();
  return 0;
}