#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "cliff_detector/cliff_detector_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cliff_detector::CliffDetectorNode>());
    rclcpp::shutdown();
    return 0;
    
}