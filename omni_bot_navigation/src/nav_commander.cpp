#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <algorithm>
#include <vector>
#include <chrono>
// #include <thread> (uncomment if required)



// a make shift cpp nav2 commander code for pose setting 
class NavCommander: public rclcpp::Node{
private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

public:
    NavCommander(): Node("nav_commander_node"){
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 1);
    }

    void set_goal_pose(const std::vector<double> pose){
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "map";
        msg.pose.position.x = pose[0];
        msg.pose.position.y = pose[1];

        double yaw = pose[2]; // in [rad];
        tf2::Quaternion q;
        q.setEuler(yaw, 0.0, 0.0);
        
        msg.pose.orientation.x = q.getX();
        msg.pose.orientation.y = q.getY();
        msg.pose.orientation.z = q.getZ();
        msg.pose.orientation.w = q.getW();

        this->pose_pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "setting goal pose to: [%lf, %lf, %lf]", pose[0], pose[1], pose[2]);
        RCLCPP_INFO(this->get_logger(), "[yaw: %.2lf]", yaw);
        // RCLCPP_INFO(this->get_logger(), "qx, qy, qz, qw: [%lf, %lf, %lf, %lf]", q.getX(), q.getY(), q.getZ(), q.getW());
        rclcpp::sleep_for(std::chrono::milliseconds(100)); // 10Hz
    }


};


int main(int argc, char** argv){
    rclcpp::init(argc, argv);

    if (argc < 4) {
        std::cerr << "Usage: program_name x y yaw (float values)" << std::endl;
        return 1;
    }

    std::vector<double> pose(3);
    for(int i=0; i < 3; i++){
        pose[i] = std::stod(argv[i+1]);
    }

    auto node = std::make_shared<NavCommander>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto start = node->get_clock()->now();
    while ((node->get_clock()->now() - start).seconds() < 1.0) {
        executor.spin_some();
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    node->set_goal_pose(pose);
  
    start = node->get_clock()->now();
    while ((node->get_clock()->now() - start).seconds() < 2.0) {
        executor.spin_some();
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    rclcpp::shutdown();

    return 0;

}


