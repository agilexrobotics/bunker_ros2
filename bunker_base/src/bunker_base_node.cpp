/*
 * bunker_base_node.cpp
 *
 * Created on: 3 2, 2022 16:41
 * Description:
 *
 * Copyright (c) 2021 Agilex Robot Pte. Ltd.
 */

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "bunker_base/bunker_base_ros.hpp"

using namespace westonrobot;

std::shared_ptr<BunkerBaseRos> robot;

void DetachRobot(int signal) {
  (void)signal;
  robot->Stop();
}

int main(int argc, char **argv) {
  // setup ROS node
  rclcpp::init(argc, argv);
  //   std::signal(SIGINT, DetachRobot);

  robot = std::make_shared<BunkerBaseRos>("bunker");
  while(robot->Initialize()) {

    std::cout << "Robot initialized, start running ..." << std::endl;
    robot->Run();
  }

  return 0;
}