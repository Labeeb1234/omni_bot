#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <algorithm>
#include <vector>
#include <chrono>
#include <iostream>
// #include <thread> (uncomment if required)



// a make shift cpp nav2 commander code for pose setting 
class NavCommander: public rclcpp::Node{
private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_update_pub_;
    // rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr click_point_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double x_ = 0.0, y_ = 0.0;

public:
    NavCommander(): Node("nav_commander_node"){
        // pose_sub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 1);
        // click_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("clicked_point", 10, std::bind(&NavCommander::click_point_callback, this, std::placeholders::_1));
        goal_update_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_update", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(5), std::bind(&NavCommander::dyanmic_point_loop, this));
    }


    void dyanmic_point_loop(){
        geometry_msgs::msg::PoseStamped pose;
        
        pose.header.stamp = this->get_clock()->now();
        pose.header.frame_id = "map";
        pose.pose.position.x = this->x_;
        pose.pose.position.y = this->y_;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;

        goal_update_pub_->publish(pose);
       
        RCLCPP_INFO(this->get_logger(), "dp: [x: %lf, y: %lf]", this->x_, this->y_);
        this->x_--;
        
    }

    // void click_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg){
    //     geometry_msgs::msg::PoseStamped pose;
    //     pose.header.stamp = this->get_clock()->now(); 
    //     pose.header.frame_id = "map"; 
    //     pose.pose.position = msg->point;
    //     pose.pose.orientation.x = 0.0;
    //     pose.pose.orientation.y = 0.0;
    //     pose.pose.orientation.z = 0.0;
    //     pose.pose.orientation.w = 1.0;

    //     goal_update_pub_->publish(pose);

    // }

    // void set_goal_pose(const std::vector<double> pose){
    //     geometry_msgs::msg::PoseStamped msg;
    //     msg.header.stamp = this->get_clock()->now();
    //     msg.header.frame_id = "map";
    //     msg.pose.position.x = pose[0];
    //     msg.pose.position.y = pose[1];

    //     double yaw = pose[2]; // in [rad];
    //     tf2::Quaternion q;
    //     q.setEuler(yaw, 0.0, 0.0);
        
    //     msg.pose.orientation.x = q.getX();
    //     msg.pose.orientation.y = q.getY();
    //     msg.pose.orientation.z = q.getZ();
    //     msg.pose.orientation.w = q.getW();

    //     this->pose_pub_->publish(msg);

    //     RCLCPP_INFO(this->get_logger(), "setting goal pose to: [%lf, %lf, %lf]", pose[0], pose[1], pose[2]);
    //     RCLCPP_INFO(this->get_logger(), "[yaw: %.2lf]", yaw);
    //     // RCLCPP_INFO(this->get_logger(), "qx, qy, qz, qw: [%lf, %lf, %lf, %lf]", q.getX(), q.getY(), q.getZ(), q.getW());
    //     rclcpp::sleep_for(std::chrono::milliseconds(100)); // 10Hz
    // }


};


int main(int argc, char** argv){
    rclcpp::init(argc, argv);

    // if (argc < 4) {
    //     std::cerr << "Usage: program_name x y yaw (float values)" << std::endl;
    //     return 1;
    // }

    // std::vector<double> pose(3);
    // for(int i=0; i < 3; i++){
    //     pose[i] = std::stod(argv[i+1]);
    // }

    auto node = std::make_shared<NavCommander>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    executor.spin();


    // auto start = node->get_clock()->now();
    // while ((node->get_clock()->now() - start).seconds() < 1.0) {
    //     executor.spin_some();
    //     rclcpp::sleep_for(std::chrono::milliseconds(100));
    // }

    // node->set_goal_pose(pose);
  
    // start = node->get_clock()->now();
    // while ((node->get_clock()->now() - start).seconds() < 2.0) {
    //     executor.spin_some();
    //     rclcpp::sleep_for(std::chrono::milliseconds(100));
    // }

    rclcpp::shutdown();

    return 0;

}


