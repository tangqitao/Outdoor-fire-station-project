/*
 * scout_messenger.cpp
 *
 * Created on: Apr 26, 2019 22:14
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "scout_base/scout_messenger.hpp"

#include <tf/transform_broadcaster.h>

#include "scout_msgs/ScoutStatus.h"
#include "scout_msgs/ScoutBmsStatus.h"

namespace westonrobot
{
  ScoutROSMessenger::ScoutROSMessenger(ros::NodeHandle *nh)
      : scout_(nullptr), nh_(nh) {}

  ScoutROSMessenger::ScoutROSMessenger(ScoutBase *scout, ros::NodeHandle *nh)
      : scout_(scout), nh_(nh) {}

  void ScoutROSMessenger::SetupSubscription()
  {
    // odometry publisher
    odom_publisher_ = nh_->advertise<nav_msgs::Odometry>(odom_topic_name_, 250);
    status_publisher_ =
        nh_->advertise<scout_msgs::ScoutStatus>("/scout_status", 10);
    BMS_status_publisher_ = nh_->advertise<scout_msgs::ScoutBmsStatus>("/BMS_status", 10);

    // cmd subscriber
    motion_cmd_subscriber_ = nh_->subscribe<geometry_msgs::Twist>(
        "/cmd_vel", 5, &ScoutROSMessenger::TwistCmdCallback, this);
    light_cmd_subscriber_ = nh_->subscribe<scout_msgs::ScoutLightCmd>(
        "/scout_light_control", 5, &ScoutROSMessenger::LightCmdCallback, this);
  }

  void ScoutROSMessenger::TwistCmdCallback(
      const geometry_msgs::Twist::ConstPtr &msg)
  {
    if (!simulated_robot_)
    {
      scout_->SetMotionCommand(msg->linear.x, msg->linear.y, msg->angular.z);
    }
    else
    {
      std::lock_guard<std::mutex> guard(twist_mutex_);
      current_twist_ = *msg.get();
    }
    // ROS_INFO("cmd received:%f, %f", msg->linear.x, msg->angular.z);
  }

  void ScoutROSMessenger::GetCurrentMotionCmdForSim(double &linear,
                                                    double &angular)
  {
    std::lock_guard<std::mutex> guard(twist_mutex_);
    linear = current_twist_.linear.x;
    angular = current_twist_.angular.z;
  }

  void ScoutROSMessenger::LightCmdCallback(
      const scout_msgs::ScoutLightCmd::ConstPtr &msg)
  {
    if (!simulated_robot_)
    {
      if (msg->enable_cmd_light_control)
      {
        ScoutLightCmd cmd;
        cmd.enable_ctrl = msg->enable_cmd_light_control;
        switch (msg->front_mode)
        {
        case scout_msgs::ScoutLightCmd::LIGHT_CONST_OFF:
        {
          cmd.front_mode = ScoutLightCmd::LightMode::CONST_OFF;
          break;
        }
        case scout_msgs::ScoutLightCmd::LIGHT_CONST_ON:
        {
          cmd.front_mode = ScoutLightCmd::LightMode::CONST_ON;
          break;
        }
        case scout_msgs::ScoutLightCmd::LIGHT_BREATH:
        {
          cmd.front_mode = ScoutLightCmd::LightMode::BREATH;
          break;
        }
        case scout_msgs::ScoutLightCmd::LIGHT_CUSTOM:
        {
          cmd.front_mode = ScoutLightCmd::LightMode::CUSTOM;
          cmd.front_custom_value = msg->front_custom_value;
          break;
        }
        }

        switch (msg->rear_mode)
        {
        case scout_msgs::ScoutLightCmd::LIGHT_CONST_OFF:
        {
          cmd.rear_mode = ScoutLightCmd::LightMode::CONST_OFF;
          break;
        }
        case scout_msgs::ScoutLightCmd::LIGHT_CONST_ON:
        {
          cmd.rear_mode = ScoutLightCmd::LightMode::CONST_ON;
          break;
        }
        case scout_msgs::ScoutLightCmd::LIGHT_BREATH:
        {
          cmd.rear_mode = ScoutLightCmd::LightMode::BREATH;
          break;
        }
        case scout_msgs::ScoutLightCmd::LIGHT_CUSTOM:
        {
          cmd.rear_mode = ScoutLightCmd::LightMode::CUSTOM;
          cmd.rear_custom_value = msg->rear_custom_value;
          break;
        }
        }

        scout_->SetLightCommand(cmd);
      }
      else
      {
        scout_->DisableLightCmdControl();
      }
    }
    else
    {
      std::cout << "simulated robot received light control cmd" << std::endl;
    }
  }

  void ScoutROSMessenger::PublishStateToROS()
  {
    current_time_ = ros::Time::now();
    double dt = (current_time_ - last_time_).toSec();

    static bool init_run = true;
    if (init_run)
    {
      last_time_ = current_time_;
      init_run = false;
      return;
    }

    auto state = scout_->GetScoutState();

    // publish scout state message
    scout_msgs::ScoutStatus status_msg;
    scout_msgs::ScoutBmsStatus bms_status;

    status_msg.header.stamp = current_time_;

    status_msg.linear_velocity = state.linear_velocity;
    status_msg.angular_velocity = state.angular_velocity;

    status_msg.base_state = state.base_state;
    status_msg.control_mode = state.control_mode;
    status_msg.fault_code = state.fault_code;
    status_msg.battery_voltage = state.battery_voltage;

    for (int i = 0; i < 4; ++i)
    {
      status_msg.motor_states[i].current = state.actuator_states[i].motor_current;
      status_msg.motor_states[i].rpm = state.actuator_states[i].motor_rpm;
      status_msg.motor_states[i].temperature = state.actuator_states[i].motor_temperature;
      status_msg.motor_states[i].motor_pose = state.actuator_states[i].motor_pulses;
      status_msg.driver_states[i].driver_state = state.actuator_states[i].driver_state;
      status_msg.driver_states[i].driver_voltage = state.actuator_states[i].driver_voltage;
      status_msg.driver_states[i].driver_temperature = state.actuator_states[i].driver_temperature;
    }

    status_msg.light_control_enabled = state.light_control_enabled;
    status_msg.front_light_state.mode = state.front_light_state.mode;
    status_msg.front_light_state.custom_value =
        state.front_light_state.custom_value;
    status_msg.rear_light_state.mode = state.rear_light_state.mode;
    status_msg.rear_light_state.custom_value =
        state.front_light_state.custom_value;

    bms_status.SOC = state.SOC;
    bms_status.SOH = state.SOH;
    bms_status.battery_voltage = state.bms_battery_voltage;
    bms_status.battery_current = state.battery_current;
    bms_status.battery_temperature = state.battery_temperature;
    bms_status.Alarm_Status_1 = state.Alarm_Status_1;
    bms_status.Alarm_Status_2 = state.Alarm_Status_2;
    bms_status.Warning_Status_1 = state.Warning_Status_1;
    bms_status.Warning_Status_2 = state.Warning_Status_2;
    BMS_status_publisher_.publish(bms_status);

    status_publisher_.publish(status_msg);


    // publish odometry and tf
    PublishOdometryToROS(state.linear_velocity, state.angular_velocity, dt);

    // record time for next integration
    last_time_ = current_time_;
  }

  void ScoutROSMessenger::PublishSimStateToROS(double linear, double angular)
  {
    current_time_ = ros::Time::now();

    double dt = (current_time_ - last_time_).toSec();

    static bool init_run = true;
    if (init_run)
    {
      last_time_ = current_time_;
      init_run = false;
      return;
    }

    // publish scout state message
    scout_msgs::ScoutStatus status_msg;

    status_msg.header.stamp = current_time_;

    status_msg.linear_velocity = linear;
    status_msg.angular_velocity = angular;

    status_msg.base_state = 0x00;
    status_msg.control_mode = 0x01;
    status_msg.fault_code = 0x00;
    status_msg.battery_voltage = 29.5;

    // for (int i = 0; i < 4; ++i)
    // {
    //     status_msg.motor_states[i].current = state.motor_states[i].current;
    //     status_msg.motor_states[i].rpm = state.motor_states[i].rpm;
    //     status_msg.motor_states[i].temperature =
    //     state.motor_states[i].temperature;
    // }

    status_msg.light_control_enabled = false;
    // status_msg.front_light_state.mode = state.front_light_state.mode;
    // status_msg.front_light_state.custom_value =
    // state.front_light_state.custom_value; status_msg.rear_light_state.mode =
    // state.rear_light_state.mode; status_msg.rear_light_state.custom_value =
    // state.front_light_state.custom_value;

    status_publisher_.publish(status_msg);

    // publish odometry and tf
    PublishOdometryToROS(linear, angular, dt);

    // record time for next integration
    last_time_ = current_time_;
  }

  void ScoutROSMessenger::PublishOdometryToROS(double linear, double angular,
                                               double dt)
  {
    // perform numerical integration to get an estimation of pose
    linear_speed_ = linear;
    angular_speed_ = angular;

    double d_x = linear_speed_ * std::cos(theta_) * dt;
    double d_y = linear_speed_ * std::sin(theta_) * dt;
    double d_theta = angular_speed_ * dt;

    position_x_ += d_x;
    position_y_ += d_y;
    theta_ += d_theta;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_);

    // publish tf transformation
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time_;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;

    tf_msg.transform.translation.x = position_x_;
    tf_msg.transform.translation.y = position_y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom_quat;

    tf_broadcaster_.sendTransform(tf_msg);

    // publish odometry and tf messages
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time_;
    odom_msg.header.frame_id = "testes";
    odom_msg.child_frame_id = base_frame_;

    odom_msg.pose.pose.position.x = position_x_;
    odom_msg.pose.pose.position.y = position_y_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;

    odom_msg.twist.twist.linear.x = linear_speed_;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = angular_speed_;

    odom_publisher_.publish(odom_msg);
  }
} // namespace westonrobot