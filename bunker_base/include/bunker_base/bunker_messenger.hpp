/*
 * bunker_messenger.hpp
 *
 * Created on: 3 2, 2022 16:41
 * Description:
 *
 * Copyright (c) 2022 tx 
 */

#ifndef BUNKER_MESSENGER_HPP
#define BUNKER_MESSENGER_HPP

#include <string>
#include <mutex>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "bunker_msgs/msg/bunker_status.hpp"

#include "ugv_sdk/mobile_robot/bunker_robot.hpp"
#include "ugv_sdk/utilities/protocol_detector.hpp"

namespace westonrobot {
template <typename BunkerType>
class BunkerMessenger {
 public:
  BunkerMessenger(std::shared_ptr<BunkerType> bunker, rclcpp::Node *node)
      : bunker_(bunker), node_(node) {}

  void SetOdometryFrame(std::string frame) { odom_frame_ = frame; }
  void SetBaseFrame(std::string frame) { base_frame_ = frame; }
  void SetOdometryTopicName(std::string name) { odom_topic_name_ = name; }

  void SetSimulationMode(int loop_rate) {
    simulated_robot_ = true;
    sim_control_rate_ = loop_rate;
  }

  void SetupSubscription() {
    // odometry publisher
    odom_pub_ =
        node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name_, 50);
    status_pub_ = node_->create_publisher<bunker_msgs::msg::BunkerStatus>(
        "/bunker_status", 10);

    // cmd subscriber
    motion_cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 5,
        std::bind(&BunkerMessenger::TwistCmdCallback, this,
                  std::placeholders::_1));
    // light_cmd_sub_ = node_->create_subscription<bunker_msgs::msg::BunkerLightCmd>(
    //     "/light_control", 5,
    //     std::bind(&BunkerMessenger::LightCmdCallback, this,
    //               std::placeholders::_1));

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
  }

  void PublishStateToROS() {
    current_time_ = node_->get_clock()->now();

    static bool init_run = true;
    if (init_run) {
      last_time_ = current_time_;
      init_run = false;
      return;
    }
    double dt = (current_time_ - last_time_).seconds();

    auto state = bunker_->GetRobotState();

    // publish bunker state message
    bunker_msgs::msg::BunkerStatus status_msg;

    status_msg.header.stamp = current_time_;

    status_msg.linear_velocity = state.motion_state.linear_velocity;
    status_msg.angular_velocity = state.motion_state.angular_velocity;

    status_msg.vehicle_state = state.system_state.vehicle_state;
    status_msg.control_mode = state.system_state.control_mode;
    status_msg.error_code = state.system_state.error_code;
    status_msg.battery_voltage = state.system_state.battery_voltage;

    auto actuator = bunker_->GetActuatorState();

    for (int i = 0; i < 2; ++i) {
      // actuator_hs_state
      uint8_t motor_id = actuator.actuator_hs_state[i].motor_id;

      status_msg.actuator_states[motor_id].rpm =
          actuator.actuator_hs_state[i].rpm;
      status_msg.actuator_states[motor_id].current =
          actuator.actuator_hs_state[i].current;
      status_msg.actuator_states[motor_id].pulse_count =
          actuator.actuator_hs_state[i].pulse_count;

      // actuator_ls_state
      motor_id = actuator.actuator_ls_state[i].motor_id;

      status_msg.actuator_states[motor_id].driver_voltage =
          actuator.actuator_ls_state[i].driver_voltage;
      status_msg.actuator_states[motor_id].driver_temperature =
          actuator.actuator_ls_state[i].driver_temp;
      status_msg.actuator_states[motor_id].motor_temperature =
          actuator.actuator_ls_state[i].motor_temp;
      status_msg.actuator_states[motor_id].driver_state =
          actuator.actuator_ls_state[i].driver_state;
    }


    status_pub_->publish(status_msg);

    // publish odometry and tf
    PublishOdometryToROS(state.motion_state, dt);

    // record time for next integration
    last_time_ = current_time_;
  }

 private:
  std::shared_ptr<BunkerType> bunker_;
  rclcpp::Node *node_;

  std::string odom_frame_;
  std::string base_frame_;
  std::string odom_topic_name_;

  bool simulated_robot_ = false;
  int sim_control_rate_ = 50;

  std::mutex twist_mutex_;
  geometry_msgs::msg::Twist current_twist_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<bunker_msgs::msg::BunkerStatus>::SharedPtr status_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motion_cmd_sub_;
  // rclcpp::Subscription<bunker_msgs::msg::BunkerLightCmd>::SharedPtr
      // light_cmd_sub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // speed variables
  double position_x_ = 0.0;
  double position_y_ = 0.0;
  double theta_ = 0.0;

  rclcpp::Time last_time_;
  rclcpp::Time current_time_;

  void TwistCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!simulated_robot_) {
      SetBunkerMotionCommand(msg);
    } else {
      std::lock_guard<std::mutex> guard(twist_mutex_);
      current_twist_ = *msg.get();
    }
    // ROS_INFO("Cmd received:%f, %f", msg->linear.x, msg->angular.z);
  }
  void SetBunkerMotionCommand(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    bunker_->SetMotionCommand(msg->linear.x, msg->angular.z);
  }

  

  geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw) {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
  }

  void PublishOdometryToROS(const MotionStateMessage &msg, double dt) {
    // perform numerical integration to get an estimation of pose
    double linear_speed = msg.linear_velocity;
    double angular_speed = msg.angular_velocity;

    // if (std::is_base_of<BunkerMiniOmniRobot, BunkerType>::value) {
    //   lateral_speed = msg.lateral_velocity;
    // } else {
    //   lateral_speed = 0;
    // }

    double d_x = linear_speed * std::cos(theta_) * dt;
    double d_y = linear_speed * std::sin(theta_) * dt;
    double d_theta = angular_speed * dt;

    position_x_ += d_x;
    position_y_ += d_y;
    theta_ += d_theta;

    geometry_msgs::msg::Quaternion odom_quat =
        createQuaternionMsgFromYaw(theta_);

    // publish tf transformation
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time_;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;

    tf_msg.transform.translation.x = position_x_;
    tf_msg.transform.translation.y = position_y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom_quat;

    tf_broadcaster_->sendTransform(tf_msg);

    // publish odometry and tf messages
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time_;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    odom_msg.pose.pose.position.x = position_x_;
    odom_msg.pose.pose.position.y = position_y_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;

    odom_msg.twist.twist.linear.x = linear_speed;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = angular_speed;

    odom_pub_->publish(odom_msg);
  }
};
}  // namespace westonrobot

#endif /* SCOUT_MESSENGER_HPP */
