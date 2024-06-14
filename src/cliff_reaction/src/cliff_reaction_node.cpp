#include "cliff_reaction/cliff_reaction_node.hpp"

CliffReactionNode::CliffReactionNode()
: Node("cliff_reaction")
{
    params_callback_handle_ = add_on_set_parameters_callback(
            std::bind(& CliffReactionNode::parametersCallback, this, std::placeholders::_1));

    declare_parameter("threshold", 0.5);

    points_sub = this->create_subscription<geometry_msgs::msg::PolygonStamped>("points", 2, 
        std::bind(& CliffReactionNode::pointsCallback, this, std::placeholders::_1));

    //camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>("turned_camera_info", 10,
    //    std::bind(& CliffReactionNode::cameraInfoCallback, this, std::placeholders::_1));

    warning_pub = this->create_publisher<std_msgs::msg::String>("warning", 10);
    RCLCPP_INFO(this->get_logger(), "cliff reaction node initialised");
}

CliffReactionNode::~CliffReactionNode(){}

rcl_interfaces::msg::SetParametersResult CliffReactionNode::parametersCallback(
        const std::vector<rclcpp::Parameter> & parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    try {
        for (const auto & parameter : parameters) {
            if (parameter.get_name() == "threshold") {
                setThreshold(parameter.as_double());
            } else if (parameter.get_name() == "image_size") {
                setImageSize(parameter.as_int());
            } 
        }
    } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), e.what());
    }
    
    return result;
}

void CliffReactionNode::setImageSize(int image_size){
    if (im_size < 0)
    {
        im_size = 0;
    } else
    {
        im_size = image_size;
    }
}

void CliffReactionNode::setThreshold(double t)
{
    if (t < 0.0)
    {
        threshold = 0.0;
    } else if (t > 1.0)
    {
        threshold = 1.0;
    } else
    {
        threshold = t;
    }
}

void CliffReactionNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg)
{
    int h = msg->height;
    int w = msg->width;

    im_size = h*w;
}

void CliffReactionNode::pointsCallback(const geometry_msgs::msg::PolygonStamped::ConstSharedPtr& msg)
{
    if (im_size)
    {
        int cliff_pts_num = msg->polygon.points.size();
        double cliff_percent = static_cast<double>(cliff_pts_num)/static_cast<double>(im_size);
        RCLCPP_INFO(this->get_logger(), "cliff percentage: %f", cliff_percent);
        if (cliff_percent > threshold)
        {
            auto warning = std_msgs::msg::String();
            warning.data = "CLIFF DETECTED";
            warning_pub->publish(warning);
        }
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CliffReactionNode>());
    rclcpp::shutdown();
    return 0;

}