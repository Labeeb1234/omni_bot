#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

using namespace std;

class PalletJackActor: public rclcpp::Node{
private:
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
rclcpp::TimerBase::SharedPtr timer_;

int timer_period_ms = 1 ; // 1000 [Hz]
geometry_msgs::msg::Twist init_cmd_; 
double jack_x = 0.0, jack_y = 0.0;

public:
PalletJackActor(): Node("palletjack_actor_node"){
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("pallet_jack/cmd_vel", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("pallet_jack/odom", rclcpp::SystemDefaultsQoS(), bind(&PalletJackActor::pose_callback, this, placeholders::_1));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_period_ms), bind(&PalletJackActor::actor_loop, this));
    
    // giving an initial push to the pallet jack
    init_cmd_.linear.x=-5.0;
    init_cmd_.linear.y=0.0;
    init_cmd_.linear.z=0.0;
    init_cmd_.angular.x=0.0;
    init_cmd_.angular.y=0.0;
    init_cmd_.angular.z=0.0;
    this->twist_pub_->publish(init_cmd_);
}

void pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
    this->jack_x = msg->pose.pose.position.x;
    this->jack_y = msg->pose.pose.position.y;
}

void actor_loop(){
    // RCLCPP_INFO(this->get_logger(), "[x: %f, y: %f]", this->jack_x, this->jack_y);
    double current_time = this->get_clock()->now().seconds();
    geometry_msgs::msg::Twist cmd;
    if(this->jack_y <= -1.0){
        cmd.linear.x=-0.15;
        cmd.linear.y=0.0;
        cmd.linear.z=0.0;
        cmd.angular.x=0.0;
        cmd.angular.y=0.0;
        cmd.angular.z=0.0;
        this->twist_pub_->publish(cmd);
    }
}

};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = make_shared<PalletJackActor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
