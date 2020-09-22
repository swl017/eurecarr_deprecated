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
    nh.param("isSteerInvert", isSteerInvert_, false);
    nh.param("isThrottleInvert", isThrottleInvert_, false);
    // nh.param("isOptitrack", isOptitrack_, false);
    nh.param("steerGain", s_ratio_, 1.0);
    nh.param("throttleGain", t_ratio_, 1.0);
    nh.param("maxThrottle", maxThrottle_, 1.0);
    nh.param("isBChassis", isBChassis_, false);
    nh.param("isWhiteCar", isWhiteCar_, false);

    if(isSteerInvert_){
        s_ratio_ = -s_ratio_;
    }
    if(isThrottleInvert_){
        t_ratio_ = -t_ratio_;
    }
    if(isWhiteCar_){
        steerTrim_ = -0.01;
    }

    if(isBChassis_){
        // Optitrack
        SERVO_ALPHA_S = 500; ///< Coefficient for calculating steering angle
        SERVO_ALPHA_T = 500; ///< Coefficient for calculating steering angle
        SERVO_BETA_S = 1500; ///< Coefficient for calculating steering angle
        SERVO_BETA_T = 1500; ///< Coefficient for calculating steering angle
        PWM_T_MAX = 1700;
        PWM_T_MIN = 1400;
    }
    if(isWhiteCar_){
        // Optitrack
        SERVO_ALPHA_S = 300; ///< Coefficient for calculating steering angle
        SERVO_ALPHA_T = 500; ///< Coefficient for calculating steering angle
        SERVO_BETA_S = 1500; ///< Coefficient for calculating steering angle
        SERVO_BETA_T = 1500; ///< Coefficient for calculating steering angle
        PWM_T_MAX = 1700;
        PWM_T_MIN = 1400;
    }
    else{
        SERVO_ALPHA_S = 500; ///< Coefficient for calculating steering angle
        SERVO_ALPHA_T = 500; ///< Coefficient for calculating steering angle
        SERVO_BETA_S = 1500; ///< Coefficient for calculating steering angle
        SERVO_BETA_T = 1500; ///< Coefficient for calculating steering angle
        PWM_T_MAX = 1600;
        PWM_T_MIN = 1250;
    }


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
    chassis_command_wpt_sub_ = nh.subscribe("waypointFollower/chassisCommand", 1, &MessageInterface::chassisCommandWptCallback, this);
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
    // if(chassis_command_msg.sender == "mppi_controller")
    // {
    //     control_sender_ = "mppi_controller";
    //     // collect
    //     steering_command_mppi_ = chassis_command_msg.steering;
    //     throttle_command_mppi_ = chassis_command_msg.throttle;
    // }

        control_sender_ = "mppi_controller";
        // collect
        steering_command_mppi_ = chassis_command_msg.steering;
        throttle_command_mppi_ = chassis_command_msg.throttle;


    counter_ = 0;
}

void MessageInterface::chassisCommandWptCallback(autorally_msgs::chassisCommand chassis_command_msg)
{
    if(chassis_command_msg.sender == "waypoint_follower")
    {
        control_sender_ = "waypoint_follower";
        // collect
        steering_command_wpt_ = chassis_command_msg.steering;
        throttle_command_wpt_ = chassis_command_msg.throttle;
    }
    counter_ = 0;
}

void MessageInterface::chassisCommandJoyCallback(autorally_msgs::chassisCommand chassis_command_msg)
{
    // collect
    if(control_sender_ == "mppi_controller")
    {
        // steer_ = steering_command_wpt_;
        steer_ = -steering_command_mppi_;
        throttle_ = -throttle_command_mppi_;
        // throttle_ = chassis_command_msg.throttle;
        throttle_ = std::min(std::max(-maxThrottle_, throttle_),maxThrottle_);
        if(chassis_command_msg.throttle > 0.2){
            throttle_ = 0.2;
        }
        if(chassis_command_msg.throttle < -0.2){
            throttle_ = -0.2;
        }

    }
    else if(control_sender_ == "waypoint_follower")
    {
        // steer_ = chassis_command_msg.steering;
        steer_ = steering_command_wpt_;
        throttle_ = throttle_command_wpt_;
        // throttle_ = -chassis_command_msg.throttle;
        if(chassis_command_msg.throttle > 0.2){
            throttle_ = 0.2;
        }
        if(chassis_command_msg.throttle < -0.2){
            throttle_ = -0.2;
        }

    }
    if(counter_>40){
        steer_ = chassis_command_msg.steering;
        throttle_ = -chassis_command_msg.throttle;
        control_sender_ = "joystick";
    }
    if(isWhiteCar_){
        steer_ += steerTrim_;
    }
    if(throttle_ > 0.43){
        throttle_ = 0.43;
    }

    pwm_steer_cmd_.data = s_ratio_ * SERVO_ALPHA_S * steer_ + SERVO_BETA_S;
    pwm_throttle_cmd_.data = t_ratio_ * SERVO_ALPHA_T * throttle_ + SERVO_BETA_T;
    if(pwm_throttle_cmd_.data > PWM_T_MAX){
        pwm_throttle_cmd_.data = PWM_T_MAX;
    }
    else if(pwm_throttle_cmd_.data < PWM_T_MIN){
        pwm_throttle_cmd_.data = PWM_T_MIN;
    }
    // publish to Arduino
    pwm_steer_cmd_pub_.publish(pwm_steer_cmd_);
    pwm_throttle_cmd_pub_.publish(pwm_throttle_cmd_);
    // publish to autorally
    chassis_state_.autonomousEnabled = automan_bool_.data;
    chassis_state_.runstopMotionEnabled = runstop_bool_.data;
    if(control_sender_ != "mppi_controller")
    {
        if(control_sender_ == "waypoint_follower"){
            chassis_state_.throttleCommander = "waypoint_follower";
            chassis_state_.steeringCommander = "waypoint_follower";
        }
        else{
            chassis_state_.throttleCommander = "joystick";
            chassis_state_.steeringCommander = "joystick";
        }
    }
    else {
        chassis_state_.throttleCommander = "mppi_controller";
        chassis_state_.steeringCommander = "mppi_controller";
        control_sender_ = "a";
    }
    chassis_state_.steering = (steer_debug_.data - SERVO_BETA_S)/(s_ratio_ * SERVO_ALPHA_S);
    chassis_state_.throttle = (throttle_debug_.data - SERVO_BETA_T)/(t_ratio_ * SERVO_ALPHA_T);
    chassis_state_pub_.publish(chassis_state_);
    wheel_speeds_.lbSpeed = speed_lr_;
    wheel_speeds_.lfSpeed = speed_lf_;
    wheel_speeds_.rbSpeed = speed_rr_;
    wheel_speeds_.rfSpeed = speed_rf_;
    wheel_speeds_pub_.publish(wheel_speeds_);
    if(counter_<10000){
        counter_++;
    }
}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "MessageInterface");
  eurecarr_core::MessageInterface MessageInterface;
  ros::spin();
  return 0;
}
