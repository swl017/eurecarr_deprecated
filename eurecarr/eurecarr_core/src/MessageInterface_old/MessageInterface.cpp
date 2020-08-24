/**
 * @file MessageInterface.cpp
 * @author Seungwook Lee <seungwook1024@gmail.com>
 * @date February, 2020
 * @copyright 2020 KAIST
 * @brief Odometry node
 *
 * @details An interface node that transforms messages from arduino to appropriate ROS standard/custom messages
 **/


#include "MessageInterface.h"

namespace eurecarr_core
{
MessageInterface::MessageInterface()
{
    ros::NodeHandle nh;

    //nh.getParam("") // param for controller priority 
    /**
    * 'mppi_controller': 1
    'constantSpeedController': 2
    'waypointFollower': 3
    'joystick': 4
    'OCS': 5
    'RC': 7
     */

    // subscribe from arduino
    wheel_speed_lf_sub_ = nh.subscribe("wheel_speed_lf", 1, &MessageInterface::wheelSpeedlfCallback, this);      // subscriber for wheel speeds <- Arduino Serial node
    wheel_speed_rf_sub_ = nh.subscribe("wheel_speed_rf", 1, &MessageInterface::wheelSpeedrfCallback, this);
    wheel_speed_lr_sub_ = nh.subscribe("wheel_speed_lr", 1, &MessageInterface::wheelSpeedlrCallback, this);
    wheel_speed_rr_sub_ = nh.subscribe("wheel_speed_rr", 1, &MessageInterface::wheelSpeedrrCallback, this);
    steer_debug_sub_ = nh.subscribe("steer_debug", 1, &MessageInterface::steerDebugCallback, this);
    throttle_debug_sub_ = nh.subscribe("throttle_debug", 1, &MessageInterface::throttleDebugCallback, this);
    // subscribe from autorally(to repeat to arduino)
    //automan_autorally_sub_ = nh.subscribe("automan", 1, &MessageInterface::automanCallback, this);
    runstop_autorally_sub_ = nh.subscribe("runstop", 1, &MessageInterface::runstopCallback, this);     
    chassis_command_joy_sub_ = nh.subscribe("joystick/chassisCommand", 1, &MessageInterface::chassisCommandJoyCallback, this); 
    chassis_command_mppi_sub_ = nh.subscribe("mppi_controller/chassisCommand", 1, &MessageInterface::chassisCommandMPPICallback, this);
    // chassis_command_wpt_sub_ = nh.subscribe("waypoint_follower/chassisCommand", 1, &MessageInterface::chassisCommandWptCallback, this);
    // publishers to conform to [autorally_msgs::*]
    wheel_speeds_pub_ = nh.advertise<autorally_msgs::wheelSpeeds>("wheelSpeeds", 1);    // pub at chassisCommandCallback
    // publishers to arduino
    automan_arduino_pub_ = nh.advertise<std_msgs::Bool>("automan_std", 1);              // pub at automanCallback
    runstop_arduino_pub_ = nh.advertise<std_msgs::Bool>("runstop_std", 1);              // pub at runstopCallback
    pwm_steer_cmd_pub_ = nh.advertise<std_msgs::Float64>("pwm_steer_cmd", 1);           // pub at chassisCommandCallback
    pwm_throttle_cmd_pub_ = nh.advertise<std_msgs::Float64>("pwm_throttle_cmd",1);      // pub at chassisCommandCallback
    // publisher to autorally
    chassis_state_pub_ = nh.advertise<autorally_msgs::chassisState>("chassisState", 1); // pub at chassisCommandCallback


}

MessageInterface::~MessageInterface()
{
}

void MessageInterface::wheelSpeedlfCallback(const std_msgs::Float64Ptr& ws_lf_msg)
{
    double ws = ws_lf_msg->data;
    speed_lf_ = ws;
}
void MessageInterface::wheelSpeedrfCallback(const std_msgs::Float64Ptr& ws_rf_msg)
{
    double ws = ws_rf_msg->data;
    speed_rf_ = ws;
}
void MessageInterface::wheelSpeedlrCallback(const std_msgs::Float64Ptr& ws_lr_msg)
{
    double ws = ws_lr_msg->data;
    speed_lr_ = ws;
}
void MessageInterface::wheelSpeedrrCallback(const std_msgs::Float64Ptr& ws_rr_msg)
{
    double ws = ws_rr_msg->data;
    speed_rr_ = ws;
}
void MessageInterface::steerDebugCallback(const std_msgs::Float64 steer)
{
    steer_debug_ = steer;
}
void MessageInterface::throttleDebugCallback(const std_msgs::Float64 throttle)
{
    throttle_debug_ = throttle;
}

void MessageInterface::automanCallback(const std_msgs::Bool automan_msg)
{
    automan_bool_ = automan_msg;
    automan_arduino_pub_.publish(automan_bool_);
}
void MessageInterface::runstopCallback(autorally_msgs::runstop runstop_msg)
{
    double runstop = runstop_msg.motionEnabled;
    runstop_bool_.data = runstop;

    runstop_arduino_pub_.publish(runstop_bool_);
}


void MessageInterface::chassisCommandMPPICallback(autorally_msgs::chassisCommand chassis_command_msg)
{
    if(chassis_command_msg.sender == "mppi_controller")
    {
        control_sender_ = "mppi_controller";
        // collect
        steering_command_mppi_ = chassis_command_msg.steering;
        throttle_command_mppi_ = chassis_command_msg.throttle;
    }

}

// void MessageInterface::chassisCommandWptCallback(autorally_msgs::chassisCommand chassis_command_msg)
// {
//     if(chassis_command_msg.sender == "waypoint_follower")
//     {
//         control_sender_ = "waypoint_follower";
//         // collect
//         steering_command_wpt_ = chassis_command_msg.steering;
//         throttle_command_wpt_ = chassis_command_msg.throttle;
//     }

// }

void MessageInterface::chassisCommandJoyCallback(autorally_msgs::chassisCommand chassis_command_msg)
{
    double steer, throttle;
    // collect
    if(control_sender_ == "mppi_controller")
    {
        steer = steering_command_mppi_;
        throttle = throttle_command_mppi_;
    }
    // else if(control_sender_ == "waypoint_follower")
    else{
        steer = chassis_command_msg.steering;
        throttle = chassis_command_msg.throttle;
    }
    // form
    float fb_ratio = 1;
    if(steer<0){
        fb_ratio = 0.5;
    }

    pwm_steer_cmd_.data = SERVO_ALPHA_S * steer + SERVO_BETA_S;
    pwm_throttle_cmd_.data = fb_ratio * SERVO_ALPHA_T * throttle + SERVO_BETA_T;
    // publish to Arduino
    pwm_steer_cmd_pub_.publish(pwm_steer_cmd_);
    pwm_throttle_cmd_pub_.publish(pwm_throttle_cmd_);
    // publish to autorally
    chassis_state_.autonomousEnabled = automan_bool_.data;
    chassis_state_.runstopMotionEnabled = runstop_bool_.data;
    if(control_sender_ != "mppi_controller")
    {
        chassis_state_.throttleCommander = "joystick";
        chassis_state_.steeringCommander = "joystick";
    }
    else {
        chassis_state_.throttleCommander = "mppi_controller";
        chassis_state_.steeringCommander = "mppi_controller";
        control_sender_ = "a";
    }
    chassis_state_.steering = (steer_debug_.data - SERVO_BETA_S)/(SERVO_ALPHA_S);
    chassis_state_.throttle = (throttle_debug_.data - SERVO_BETA_T)/(fb_ratio * SERVO_ALPHA_T);
    chassis_state_pub_.publish(chassis_state_);
    wheel_speeds_.lbSpeed = speed_lr_;
    wheel_speeds_.lfSpeed = speed_lf_;
    wheel_speeds_.rbSpeed = speed_rr_;
    wheel_speeds_.rfSpeed = speed_rf_;
    wheel_speeds_pub_.publish(wheel_speeds_);
}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "MessageInterface");
  eurecarr_core::MessageInterface MessageInterface;
  ros::spin();
  return 0;
}
