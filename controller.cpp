#include "controller_manager_msgs/srv/load_controller.hpp"

rclcpp::Client<controller_manager_msgs::srv::LoadController>::SharedPtr load_controller_client_;

load_controller_client_ = this->create_client<controller_manager_msgs::srv::LoadController>(
    "/lbr/controller_manager/load_controller");

bool load_controller_if_needed(const std::string &controller_name) {
  while (!load_controller_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for load_controller service.");
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "Waiting for load_controller service...");
  }

  auto request = std::make_shared<controller_manager_msgs::srv::LoadController::Request>();
  request->name = controller_name;

  auto future = load_controller_client_->async_send_request(request);
  rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);

  if (future.get()->ok) {
    RCLCPP_INFO(this->get_logger(), "Controller '%s' loaded successfully.", controller_name.c_str());
    return true;
  } else {
    RCLCPP_WARN(this->get_logger(), "Controller '%s' may already be loaded or failed to load.", controller_name.c_str());
    return false;
  }
}

void switch_to_twist_controller() {
    load_controller_if_needed("twist_controller");
  
    // 以降は元のSwitchController処理
    ...
  }
  