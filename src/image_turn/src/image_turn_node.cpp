#include "image_turn/image_turn_node.hpp"

namespace image_turn
{
    ImageTurnNode::ImageTurnNode()
    : Node("image_turn_node")
    {
        // publisher for image turned by 90 degrees
        image_pub_ = image_transport::create_publisher(this, "turned_image");

        // subscriber for the original image
        image_sub_ = image_transport::create_camera_subscription(this, "/camera/depth/image_rect_raw",
            std::bind(& ImageTurnNode::imgCallback, this, std::placeholders::_1), "raw");

        // subscriber for camera information
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>("/camera/depth/camera_info", 10, 
            std::bind(& ImageTurnNode::cameraInfoCallback, this, std::placeholders::_1));

        // publisher for camera information
        camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("turned_camera_info", 10);
        
        RCLCPP_INFO(this->get_logger(), "Image turn node initialised");
    }

    void ImageTurnNode::imgCallback(const sensor_msgs::msg::Image::ConstSharedPtr & img)
    {

        //convert ROS Image message to OpenCV image
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
        }
        catch(cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Rotate image by 90 degrees
        cv::Mat rotated_image;
        cv::rotate(cv_ptr->image, rotated_image, cv::ROTATE_90_CLOCKWISE);

        // publish the rotated message
        sensor_msgs::msg::Image::SharedPtr rotated_image_msg = cv_bridge::CvImage(img->header, img->encoding, rotated_image).toImageMsg();
        image_pub_.publish(rotated_image_msg);
    }

    void ImageTurnNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info)
    {
        // Create a new CameraInfo message
        auto rotated_camera_info_msg = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);

        // Swap the width and height
        std::swap(rotated_camera_info_msg->width, rotated_camera_info_msg->height);
        cv::Mat K = cv::Mat(3, 3, CV_64F, (void*)camera_info->k.data()).clone();

        RCLCPP_INFO(this->get_logger(), "(1, 1) %f", K.at<double>(1,2));
        cv::Mat K_rotated = cv::Mat::zeros(3, 3, CV_64F);
        // Switch focals
        K_rotated.at<double>(0,0) = K.at<double>(1,1);
        K_rotated.at<double>(1,1) = K.at<double>(0,0);

        // Adjust principal point
        K_rotated.at<double>(0,2) = rotated_camera_info_msg->width - K.at<double>(1,2);
        K_rotated.at<double>(1,2) = K.at<double>(0,2);
        K_rotated.at<double>(2, 2) = 1.0;

        // Update projection matrix P
        cv::Mat Rt = cv::Mat::zeros(3, 4, CV_64F);
        Rt.at<double>(0,0) = 1.0;
        Rt.at<double>(1,1) = 1.0;
        Rt.at<double>(2,2) = 1.0;

        cv::Mat P = K_rotated * Rt;
        
        // Update the rotated CameraInfo message
        std::copy(K_rotated.begin<double>(), K_rotated.end<double>(), rotated_camera_info_msg->k.begin());
        std::copy(P.begin<double>(), P.end<double>(), rotated_camera_info_msg->p.begin());

        // Publish the rotated CameraInfo
        camera_info_pub_->publish(*rotated_camera_info_msg);
    }

}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<image_turn::ImageTurnNode>());
    rclcpp::shutdown();
    return 0;
}