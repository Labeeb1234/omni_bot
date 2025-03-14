#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <chrono>
#include <memory>
#include <cmath>

#include "omni_bot_navigation/omni_bot_kinematics.hpp"


struct BotProperties{
    double d = 0.1; //[m] diameter
    double L = 0.3; //[m] bot base length
    double W = 0.3; //[m] bot base width
    double t = 0.036; //[m] wheel width

}bot_prop;
BodyVel vel_cmd; 

void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg){
    if (!msg->position.empty() && !msg->velocity.empty() && !msg->effort.empty()) {
        for (size_t i = 0; i < 4; i++) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s_position: [%lf]", msg->name[i].c_str(), msg->position[i]);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s_velocity: [%lf]", msg->name[i].c_str(), msg->velocity[i]);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s_effort: [%lf]", msg->name[i].c_str(), msg->effort[i]);
        }
    } else {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Received empty joint state message.");
    }
}

void command_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg){
    vel_cmd = 
}


int main(int argc, char** argv){
    // defining kinematics
    FOmniKinematics omni_bot = FOmniKinematics(bot_prop.d, bot_prop.L, bot_prop.W, bot_prop.t);
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("omni_bot_controller_node");

    // subscription to joint states to keep track of the lower layer joint_states (pos, vel and efforts)
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub;
    joint_states_sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", rclcpp::SystemDefaultsQoS(),
        joint_states_callback 
    );
    // subscription to twist msgs to get body frame velocities
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
    twist_sub_ = node->create_subscription(
        "cmd_vel",
        rclcpp::SystemDefaultsQoS(),
        command_vel_callback
    )
    

    // publisher to joint_command to send the lower-layer signals/commands to the joints after kinematics processing
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_pub;
    joint_command_pub = node->create_publisher<sensor_msgs::msg::JointState>(
        "joint_command", 10
    );

    // creating a timer function to send the control commands to the joints
    uint timer_period = 20; // in ms ---> 1000/T Hz;
    rclcpp::TimerBase::SharedPtr timer_;
    timer_ = node->create_wall_timer(
        std::chrono::milliseconds(timer_period), 
        [](){

        }    
    );


    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}