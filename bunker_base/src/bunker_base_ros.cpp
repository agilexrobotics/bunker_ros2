/*
 * bunker_base_ros.cpp
 *
 * Created on: 3 2, 2022 16:41
 * Description:
 *
 * Copyright (c) 2022 Agilex Robot Pte. Ltd.
 */

#include "bunker_base/bunker_base_ros.hpp"

#include "bunker_base/bunker_messenger.hpp"
#include "ugv_sdk/utilities/protocol_detector.hpp"

namespace westonrobot {
BunkerBaseRos::BunkerBaseRos(std::string node_name)
    : rclcpp::Node(node_name), keep_running_(false) {
  this->declare_parameter("port_name");   

  this->declare_parameter("odom_frame");
  this->declare_parameter("base_frame");
  this->declare_parameter("odom_topic_name");

  this->declare_parameter("is_bunker_mini");
  this->declare_parameter("simulated_robot");
  this->declare_parameter("control_rate");

  LoadParameters();
}

void BunkerBaseRos::LoadParameters() {
  this->get_parameter_or<std::string>("port_name", port_name_, "can0");

  this->get_parameter_or<std::string>("odom_frame", odom_frame_, "odom");
  this->get_parameter_or<std::string>("base_frame", base_frame_, "base_link");
  this->get_parameter_or<std::string>("odom_topic_name", odom_topic_name_,
                                      "odom");
  this->get_parameter_or<bool>("is_bunker_mini", is_bunker_mini_, false);
  this->get_parameter_or<bool>("simulated_robot", simulated_robot_, false);
  this->get_parameter_or<int>("control_rate", sim_control_rate_, 50);

  std::cout << "Loading parameters: " << std::endl;
  std::cout << "- port name: " << port_name_ << std::endl;
  std::cout << "- odom frame name: " << odom_frame_ << std::endl;
  std::cout << "- base frame name: " << base_frame_ << std::endl;
  std::cout << "- odom topic name: " << odom_topic_name_ << std::endl;

  std::cout << "- is bunker mini: " << std::boolalpha << is_bunker_mini_
            << std::endl;


  std::cout << "- simulated robot: " << std::boolalpha << simulated_robot_
            << std::endl;
  std::cout << "- sim control rate: " << sim_control_rate_ << std::endl;
  std::cout << "----------------------------" << std::endl;
}

bool BunkerBaseRos::Initialize() {
  if (is_bunker_mini_) {
    std::cout << "Robot base: Bunker Mini" << std::endl;
  } else {
    std::cout << "Robot base: Bunker" << std::endl;
  }

  ProtocolDectctor detector;
  if (detector.Connect(port_name_)) {
    auto proto = detector.DetectProtocolVersion(5);
      if (proto == ProtocolVersion::AGX_V1) {
        std::cout << "Detected protocol: AGX_V1" << std::endl;
        robot_ = std::unique_ptr<BunkerRobot>(
        new BunkerRobot(ProtocolVersion::AGX_V1));
      }
      else if (proto == ProtocolVersion::AGX_V2)
      {
        std::cout << "Detected protocol: AGX_V2" << std::endl;
        robot_ = std::unique_ptr<BunkerRobot>(
        new BunkerRobot(ProtocolVersion::AGX_V2));
      }else {
      std::cout << "Detected protocol: UNKONWN" << std::endl;
      return false;
    }
  } else {
    return false;
  }

  return true;
}

void BunkerBaseRos::Stop() { keep_running_ = false; }

void BunkerBaseRos::Run() {

  std::unique_ptr<BunkerMessenger<BunkerRobot>> messenger =
      std::unique_ptr<BunkerMessenger<BunkerRobot>>(new BunkerMessenger<BunkerRobot>(robot_,this));

  messenger->SetOdometryFrame(odom_frame_);
  messenger->SetBaseFrame(base_frame_);
  messenger->SetOdometryTopicName(odom_topic_name_);
  if (simulated_robot_) messenger->SetSimulationMode(sim_control_rate_);

  // connect to robot and setup ROS subscription
  if (port_name_.find("can") != std::string::npos) {
    if (robot_->Connect(port_name_)) {
      robot_->EnableCommandedMode();
      std::cout << "Using CAN bus to talk with the robot" << std::endl;
    } else {
      std::cout << "Failed to connect to the robot CAN bus" << std::endl;
      return;
    }
  } else {
    std::cout << "Please check the specified port name is a CAN port"
              << std::endl;
    return;
  }

  // publish robot state at 50Hz while listening to twist commands
  messenger->SetupSubscription();
  keep_running_ = true;
  rclcpp::Rate rate(50);
  while (keep_running_) {
    messenger->PublishStateToROS();
    rclcpp::spin_some(shared_from_this());
    rate.sleep();
  }
  
}
}  // namespace westonrobot