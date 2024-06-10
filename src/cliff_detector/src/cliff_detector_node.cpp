#include "cliff_detector/cliff_detector_node.hpp"

namespace cliff_detector
{
    CliffDetectorNode::CliffDetectorNode()
    : Node("cliff_detector")
    {
        
        params_callback_handle_ = add_on_set_parameters_callback(
            std::bind(& CliffDetectorNode::parametersCallback, this, std::placeholders::_1)
        );
        // Declare all node parameters
        declare_parameter("range_min", 0.5);
        declare_parameter("range_max", 5.0);
        declare_parameter("depth_img_row_step", 2);
        declare_parameter("depth_img_col_step", 2);
        declare_parameter("cam_model_update", false);
        declare_parameter("sensor_mount_height", 0.4);
        declare_parameter("sensor_tilt_angle", 0.0);
        declare_parameter("ground_margin", 0.05);
        declare_parameter("block_size", 2);
        declare_parameter("publish_depth", false);
        declare_parameter("used_depth_height", 200);
        declare_parameter("block_points_thresh", 10);
        declare_parameter("block_row_start", 0);
        declare_parameter("block_col_start", 0);
        declare_parameter("block_row_end", 100000);
        declare_parameter("block_col_end", 100000);

        // publisher for the debug image
        pub_ = image_transport::create_publisher(this, "depth");

        // publisher for stair points msg
        pub_points_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("points", 2);

        // subscribe to the images published by the camera
        // TODO: choose correct topic
        image_sub_ = image_transport::create_camera_subscription(this, "image", 
            std::bind(& CliffDetectorNode::depthCb, this, std::placeholders::_1, std::placeholders::_2), "raw");

        RCLCPP_INFO(this->get_logger(), "cliff detector node initialized");

    }

    CliffDetectorNode::~CliffDetectorNode()
    {
        image_sub_.shutdown();
    }

    void CliffDetectorNode::depthCb(
        const sensor_msgs::msg::Image::ConstSharedPtr & image,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info)
    {
        try
        {
            auto msg = detector_.detectCliff(image, info);
            RCLCPP_INFO(this->get_logger(), "block row start: %d", detector_.getBlockRowStart());
            RCLCPP_INFO(this->get_logger(), "block col start: %d", detector_.getBlockColStart());
            RCLCPP_INFO(this->get_logger(), "block row end: %d", detector_.getBlockRowEnd());
            RCLCPP_INFO(this->get_logger(), "block col end: %d", detector_.getBlockColEnd());
            pub_points_->publish(msg);
            if (detector_.getPublishDepthEnable())
            {
                pub_.publish(detector_.getDebugDepthImage());
            }
        }
        catch(std::runtime_error & e)
        {
            RCLCPP_INFO(this->get_logger(), "could not perform cliff detection: %s", e.what());
        }
    }

    rcl_interfaces::msg::SetParametersResult CliffDetectorNode::parametersCallback(
        const std::vector<rclcpp::Parameter> & parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        try {
            for (const auto & parameter : parameters) {
                if (parameter.get_name() == "range_min") {
                    detector_.setMinRange(parameter.as_double());
                } else if (parameter.get_name() == "range_max") {
                    detector_.setMaxRange(parameter.as_double());
                } else if (parameter.get_name() == "depth_img_row_step") {
                    detector_.setDepthImgStepRow(parameter.as_int());
                } else if (parameter.get_name() == "depth_img_col_step") {
                    detector_.setDepthImgStepCol(parameter.as_int());
                } else if (parameter.get_name() == "cam_model_update") {
                    detector_.setCamModelUpdate(parameter.as_bool());
                } else if (parameter.get_name() == "sensor_mount_height") {
                    detector_.setSensorMountHeight(parameter.as_double());
                } else if (parameter.get_name() == "sensor_tilt_angle") {
                    detector_.setSensorTiltAngle(parameter.as_double());
                } else if (parameter.get_name() == "ground_margin") {
                    detector_.setGroundMargin(parameter.as_double());
                } else if (parameter.get_name() == "block_size") {
                    detector_.setBlockSize(parameter.as_int());
                } else if (parameter.get_name() == "publish_depth") {
                    detector_.setPublishDepthEnable(parameter.as_bool());
                } else if (parameter.get_name() == "used_depth_height") {
                    detector_.setUsedDepthHeight(parameter.as_int());
                } else if (parameter.get_name() == "block_points_thresh") {
                    detector_.setBlockPointsThresh(parameter.as_int());
                } else if (parameter.get_name() == "block_row_start") {
                    detector_.setBlockRowStart(parameter.as_int());
                } else if (parameter.get_name() == "block_col_start") {
                    detector_.setBlockColStart(parameter.as_int());
                } else if (parameter.get_name() == "block_row_end") {
                    detector_.setBlockRowEnd(parameter.as_int());
                } else if (parameter.get_name() == "block_col_end") {
                    detector_.setBlockColEnd(parameter.as_int());
                }
            }
            detector_.setParametersConfigurated(false);
        } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), e.what());
        }

        return result;
    }
}