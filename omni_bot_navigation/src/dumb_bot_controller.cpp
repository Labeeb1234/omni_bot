#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <iostream>
#include <string>
#include <chrono>
#include <utility>
#include <cmath>

#include "omni_bot_navigation/omni_bot_kinematics.hpp"

/*
    A Routine code to control dumb bots in the environment to act as dynamic obstacle for the intelligent one

*/

class DumbBotRoutine: public rclcpp::Node{
private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double x_pos_, y_pos_;
    double max_vel_, vel_ = 0.0;

    struct BotProperties{
        double d = 0.1; //[m] diameter
        double L = 0.3; //[m] bot base length
        double W = 0.3; //[m] bot base width
        double t = 0.036; //[m] wheel width
    
    }bot_prop;
    FOmniKinematics omni_bot;
    Omega wheel_angular_vel_;

public:
    DumbBotRoutine(const std::string& node_namespace = ""): Node(node_namespace), omni_bot(bot_prop.d, bot_prop.L, bot_prop.W, bot_prop.t){
        this->declare_parameter<double>("max_velocity", 1.0);
        this->max_vel_ = this->get_parameter("max_velocity").as_double();

        joint_command_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "~/joint_command",
            10
        );
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "~/odometry/unfiltered",
            10,
            std::bind(&DumbBotRoutine::odom_callback, this, std::placeholders::_1)
        );

        int timer_period = 100; // in ms;
        timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_period), std::bind(&DumbBotRoutine::routine_loop, this));

    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
        this->x_pos_ = msg->pose.pose.position.x;
        this->y_pos_ = msg->pose.pose.position.y;
    }

    void routine_loop(){
        rclcpp::Time current_time = this->get_clock()->now();

        sensor_msgs::msg::JointState joint_cmd;
        RCLCPP_INFO(this->get_logger(), "dumb_botY: [%f]", this->y_pos_);

        this->vel_ += 0.5*0.1;
        wheel_angular_vel_ = omni_bot.getRPM(0.0, std::min(this->vel_, this->max_vel_), 0.0);
        RCLCPP_INFO(this->get_logger(), "[%lf %lf %lf %lf]RPM", wheel_angular_vel_.rpm1, wheel_angular_vel_.rpm2, wheel_angular_vel_.rpm3, wheel_angular_vel_.rpm4);
        omni_bot.convert_to_rads(wheel_angular_vel_);
        RCLCPP_INFO(this->get_logger(), "[%lf %lf %lf %lf]rad/s", wheel_angular_vel_.rpm1, wheel_angular_vel_.rpm2, wheel_angular_vel_.rpm3, wheel_angular_vel_.rpm4);

        joint_cmd.header.stamp = this->get_clock()->now();
        joint_cmd.name = {"Revolute_1", "Revolute_2", "Revolute_3", "Revolute_4"};  
        joint_cmd.velocity.resize(4); 
        joint_cmd.velocity[0] = wheel_angular_vel_.rpm1;
        joint_cmd.velocity[1] = wheel_angular_vel_.rpm2;
        joint_cmd.velocity[2] = wheel_angular_vel_.rpm3;
        joint_cmd.velocity[3] = wheel_angular_vel_.rpm4;
        joint_command_pub_->publish(joint_cmd);
   
    }

};


int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node1 = std::make_shared<DumbBotRoutine>("dumb_bot");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node1);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}