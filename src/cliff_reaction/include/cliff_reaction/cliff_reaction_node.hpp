#ifndef CLIFF_REACTION_NODE_HPP_
#define CLIFF_REACTION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_msgs/msg/string.hpp>


class CliffReactionNode : public rclcpp::Node
{
public:
    CliffReactionNode();
    ~CliffReactionNode();

    CliffReactionNode(const CliffReactionNode &) = delete;
    CliffReactionNode & operator=(const CliffReactionNode &) = delete;

    /**
     * @brief Callback used when a new list of cliff points is published
     * 
     * Compares the number of cliff pixels to the total number of pixels in the image.
     * If it is above a set threshold, a warning message is published.
     * 
     * @param msg Message containing detected cliff points
     */
    void pointsCallback(const geometry_msgs::msg::PolygonStamped::ConstSharedPtr& msg);
    /**
     * @brief Callback used when camera info is published
     * 
     * Take image length and height to compute total size of the image.
     * note: Not used currently, image size is set manually
     * 
     * @param msg Message containing camera info
     */
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg);
    /**
     * @brief setter for the cliff threshold
     * 
     * Represents a percentage of the total pixels present in the image.
     * If the number of cliff pixels surpasses this percentage, a cliff is present in the image
     * 
     * @param t Double between 0.0 and 1.0 representing maximum amount of cliff pixels tolerated
     */
    void setThreshold(double t);
    /**
     * @brief setter for the image size in pixels
     * @param image_size image size in pixels
     */
    void setImageSize(int image_size);
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> & parameters);

protected:
    /// @brief Subscriber for cliff points
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr points_sub;
    /// @brief Subscriber for camera info
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub;
    /// @brief Publisher for warning messages
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr warning_pub;
    /// @brief Total number of pixels in the image
    int im_size = 0;
    /// @brief Cliff threshold
    double threshold;
    
    OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
};

#endif