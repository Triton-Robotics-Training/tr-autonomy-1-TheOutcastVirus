#ifndef YOUR_SOLUTION_SRC_SPIN_SLOW_UPDATE_H_
#define YOUR_SOLUTION_SRC_SPIN_SLOW_UPDATE_H_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <vector>
#include <chrono>
#include <functional>

using ArrayMsg = std_msgs::msg::Float64MultiArray;

class SlowSolution : public rclcpp::Node {
 public:
  SlowSolution();
 private:
  rclcpp::Subscription<ArrayMsg>::SharedPtr measuredPosSubscriber;
  rclcpp::Subscription<ArrayMsg>::SharedPtr measuredVelSubscriber;

  void measuredPosCallback(const ArrayMsg::SharedPtr msg);
  void measuredVelCallback(const ArrayMsg::SharedPtr msg);

  std::vector<double> predictPos();

  std::vector<double> pos_vec;
  rclcpp::Time pos_time;
  std::vector<double> vel_vec;
  rclcpp::Time vel_time;

  std::vector<double> pred_pos_vec;
  
  bool pos_received;
  bool vel_received;
  
  std::chrono::steady_clock::time_point last_vel_time;

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<ArrayMsg>::SharedPtr predPosPublisher;
  void timerCallback();
};

#endif //YOUR_SOLUTION_SRC_SPIN_SLOW_UPDATE_H_
