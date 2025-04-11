#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include <chrono>
#include <memory>
#include <cmath>
#include <thread>
#include <iostream>
#include <string>

#include "omni_bot_navigation/omni_bot_kinematics.hpp"


// thread lock
std::mutex scoped_lock;

struct BotProperties{
    double d = 0.1; //[m] diameter
    double L = 0.3; //[m] bot base length
    double W = 0.3; //[m] bot base width
    double t = 0.036; //[m] wheel width

}bot_prop;
BodyVel vel_cmd;
 

void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg){
    RCLCPP_INFO(rclcpp::get_logger("omni_bot_controller_node"), "%s", std::string(10, '=').c_str());
    if (!msg->position.empty() && !msg->velocity.empty() && !msg->effort.empty()) {
        for (size_t i = 0; i < 4; i++) {
            RCLCPP_INFO(rclcpp::get_logger("omni_bot_controller_node"), "%s_position: [%lf]", msg->name[i].c_str(), msg->position[i]);
            RCLCPP_INFO(rclcpp::get_logger("omni_bot_controller_node"), "%s_velocity: [%lf]", msg->name[i].c_str(), msg->velocity[i]);
            RCLCPP_INFO(rclcpp::get_logger("omni_bot_controller_node"), "%s_effort: [%lf]", msg->name[i].c_str(), msg->effort[i]);
        }
    } else {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Received empty joint state message.");
    }
    RCLCPP_INFO(rclcpp::get_logger("omni_bot_controller_node"), "%s", std::string(10, '=').c_str());
}

void command_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
    vel_cmd.avg_u = msg->linear.x;
    vel_cmd.avg_v = msg->linear.y;
    vel_cmd.avg_r = msg->angular.z;
    RCLCPP_INFO(rclcpp::get_logger("omni_bot_controller_node"), "%s", std::string(10, '=').c_str());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Body Frame Velocity: [%lf %lf %lf]", vel_cmd.avg_u, vel_cmd.avg_v, vel_cmd.avg_r);
    RCLCPP_INFO(rclcpp::get_logger("omni_bot_controller_node"), "%s", std::string(10, '=').c_str());
}


int main(int argc, char** argv){

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("omni_bot_controller_node", "");
    // defining kinematics
    FOmniKinematics omni_bot(bot_prop.d, bot_prop.L, bot_prop.W, bot_prop.t);

    // subscription to joint states to keep track of the lower layer joint_states (pos, vel and efforts)
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub;
    joint_states_sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", rclcpp::SystemDefaultsQoS(),
        joint_states_callback 
    );
    // subscription to twist msgs to get body frame velocities
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    twist_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel",
        rclcpp::SystemDefaultsQoS(),
        command_vel_callback
    );
    
    // publisher to joint_command to send the lower-layer signals/commands to the joints after kinematics processing
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_pub;
    joint_command_pub = node->create_publisher<sensor_msgs::msg::JointState>("joint_command", 10);

    // creating a timer function to send the control commands to the joints
    uint timer_period = 20; // in ms ---> 1000/T Hz;
    rclcpp::TimerBase::SharedPtr timer_;
    timer_ = node->create_wall_timer(
        std::chrono::milliseconds(timer_period), 
        [joint_command_pub, &omni_bot, node](){
            
            sensor_msgs::msg::JointState joint_cmd;
            Omega wheel_angular_vel = omni_bot.getRPM(vel_cmd.avg_u, vel_cmd.avg_v, vel_cmd.avg_r);
            RCLCPP_INFO(rclcpp::get_logger("omni_bot_controller_node"), "[%lf %lf %lf %lf]RPM", wheel_angular_vel.rpm1, wheel_angular_vel.rpm2, wheel_angular_vel.rpm3, wheel_angular_vel.rpm4);
            omni_bot.convert_to_rads(wheel_angular_vel); // converting RPM to rad/s
            RCLCPP_INFO(rclcpp::get_logger("omni_bot_controller_node"), "[%lf %lf %lf %lf]rad/s", wheel_angular_vel.rpm1, wheel_angular_vel.rpm2, wheel_angular_vel.rpm3, wheel_angular_vel.rpm4);
            
            joint_cmd.header.stamp = node->get_clock()->now();
            joint_cmd.name = {"Revolute_1", "Revolute_2", "Revolute_3", "Revolute_4"};  // Add joint names
            joint_cmd.velocity.resize(4);  // Resize vector properly
            joint_cmd.velocity[0] = wheel_angular_vel.rpm1;
            joint_cmd.velocity[1] = wheel_angular_vel.rpm2;
            joint_cmd.velocity[2] = wheel_angular_vel.rpm3;
            joint_cmd.velocity[3] = wheel_angular_vel.rpm4;

            joint_command_pub->publish(joint_cmd);
            
        }    
    );


    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}