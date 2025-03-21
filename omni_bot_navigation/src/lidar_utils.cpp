#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <vector>
#include <algorithm>

using namespace std;


class LidarUtils: public rclcpp::Node{
private:
rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

public:
LidarUtils(): Node("lidar_utils_node"){
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(), bind(&LidarUtils::scan_callback, this, placeholders::_1)
    );
}

~LidarUtils(){}


void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    vector<float> ranges = msg->ranges;
    RCLCPP_INFO(this->get_logger(), "number of beams/samples: %d", (int) ranges.size());
}


};


int main(int argc, char** argv){
    rclcpp::init(argc, argv);

    auto node = make_shared<LidarUtils>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}