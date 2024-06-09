#ifndef CLIFF_REACTION_NODE_HPP_
#define CLIFF_REACTION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_msgs/msg/string.hpp>


class CliffReactionNode : public rclcpp::Node
{
private:
    /* data */
public:
    CliffReactionNode();
    ~CliffReactionNode();

    CliffReactionNode(const CliffReactionNode &) = delete;
    CliffReactionNode & operator=(const CliffReactionNode &) = delete;

    void pointsCallback(const geometry_msgs::msg::PolygonStamped::ConstSharedPtr& msg);
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg);
    void setThreshold(double t);
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> & parameters);

protected:
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr points_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr warning_pub;
    int im_size = 0;
    double threshold;
    OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
};

#endif