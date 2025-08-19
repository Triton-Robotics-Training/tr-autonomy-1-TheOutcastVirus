#include "spin_slow_update.h"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlowSolution>());
  rclcpp::shutdown();
  return 0;
}

SlowSolution::SlowSolution() : Node("slowsolution") {
  // RCLCPP_INFO(this->get_logger(), "Remove this statement from spin_slow_update.cpp");
  // your code here
  measuredPosSubscriber = this->create_subscription<ArrayMsg>("measuredpos", 10, std::bind(&SlowSolution::measuredPosCallback, this, std::placeholders::_1));
  measuredVelSubscriber = this->create_subscription<ArrayMsg>("measuredvel", 10, std::bind(&SlowSolution::measuredVelCallback, this, std::placeholders::_1));

  predPosPublisher = this->create_publisher<ArrayMsg>("predictedpos", 10);
  timer = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&SlowSolution::timerCallback, this));
  
  pos_vec = {0.0, 0.0};
  vel_vec = {0.0, 0.0};
  pred_pos_vec = {0.0, 0.0};
  
  // So this doesnt crash if we dont have data yet
  pos_received = false;
  vel_received = false;
  
  last_vel_time = std::chrono::steady_clock::now();
}

void SlowSolution::measuredPosCallback(const ArrayMsg::SharedPtr msg) {
  pos_vec = msg->data;
  pos_received = true;
}

void SlowSolution::measuredVelCallback(const ArrayMsg::SharedPtr msg) {
  vel_vec = msg->data;
  last_vel_time = std::chrono::steady_clock::now();
  vel_received = true;
}

void SlowSolution::timerCallback() {
  if (!pos_received || !vel_received) {
    return;
  }
  
  std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
  double dt = std::chrono::duration<double>(current_time - last_vel_time).count();
  
  pred_pos_vec = {pos_vec[0] + vel_vec[0] * dt, pos_vec[1] + vel_vec[1] * dt};
  
  std_msgs::msg::Float64MultiArray msg;
  msg.data = pred_pos_vec;
  predPosPublisher->publish(msg);
}