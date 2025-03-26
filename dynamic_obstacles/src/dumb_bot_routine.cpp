#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <iostream>
#include <string>
#include <chrono>
#include <utility>

/*
    A Routine code to control dumb bots in the environment to act as dynamic obstacle for the intelligent one

*/


class DumbBotRoutine: public rclcpp::Node{
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double x_pos = 0.0;
    double max_speed_;


public:
    DumbBotRoutine(const std::string& node_namespace = ""): Node(node_namespace){
        this->declare_parameter<double>("max_speed", 1.0);
        max_speed_ = this->get_parameter("max_speed").as_double();


        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("~/cmd_vel", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "~/odometry/unfiltered",
            10,
            std::bind(&DumbBotRoutine::odom_callback, this, std::placeholders::_1)
        );

        int timer_period = 100; // in ms;
        this->create_wall_timer(std::chrono::milliseconds(timer_period), std::bind(&DumbBotRoutine::routine_loop, this));

    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
        this->x_pos = msg->pose.pose.position.x;
        
    }

    void routine_loop(){
        geometry_msgs::msg::Twist cmd_msg;
        // RCLCPP_INFO(this->get_logger(), "dumb_botX: [%f]", this->x_pos);
        RCLCPP_INFO(this->get_logger(), "max_speed: [%lf]", this->max_speed_);


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