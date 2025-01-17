#ifndef IMAGE_TURN_NODE_HPP_
#define IMAGE_TURN_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace image_turn
{
    class ImageTurnNode : public rclcpp::Node
    {
    public:
        ImageTurnNode();

    protected:
        /**
         * @brief callback used when a new image is published to publish an image tilted by 90 degrees clockwise
         * 
         * @param img image provide by transport_image
         * 
        */
        void imgCallback(const sensor_msgs::msg::Image::ConstSharedPtr & img);

        /**
         * @brief callback used when a new camera info is publishe to modify K and P matrices
         * 
         * @param camera_info message published on camera_info topic
         * 
        */
        void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info);

        image_transport::CameraSubscriber image_sub_;
        image_transport::Publisher image_pub_;

        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    };
}
#endif