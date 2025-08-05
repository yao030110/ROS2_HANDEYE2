#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <chrono>

using namespace std::chrono_literals;

class MinimalClient : public rclcpp::Node
{
public:
  MinimalClient() : Node("minimal_client")
  {
    client_ = this->create_client<std_srvs::srv::Trigger>("simple_service");
    RCLCPP_INFO(this->get_logger(), "Client initialized.");
  }

  void send_request()
  {
    // 等待服务可用
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for service...");
    }

    // 创建请求
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    
    // 发送异步请求
    auto future = client_->async_send_request(
      request,
      std::bind(&MinimalClient::handle_response, this, std::placeholders::_1)
    );
    
    RCLCPP_INFO(this->get_logger(), "Request sent, waiting for response...");
  }

  void handle_response(const std::shared_future<std_srvs::srv::Trigger::Response::SharedPtr> future)
  {
    try {
      auto response = future.get();
      if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Success: %s", response->message.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "Service returned failure");
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
    }
    
    // 完成后退出
    rclcpp::shutdown();
  }

private:
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  
  // 创建客户端节点
  auto client_node = std::make_shared<MinimalClient>();
  
  // 关键修正：先启动执行器再发送请求
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(client_node);
  
  // 在单独线程中启动执行器
  std::thread spin_thread([&executor]() {
    executor->spin();
  });
  
  // 发送请求（现在节点已添加到执行器）
  client_node->send_request();
  
  // 等待响应处理完成
  spin_thread.join();
  
  return 0;
}