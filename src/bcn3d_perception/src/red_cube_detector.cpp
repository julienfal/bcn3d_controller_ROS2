#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <functional>
#include <opencv2/opencv.hpp>

class CameraSubscriber : public rclcpp::Node
{
public:
    CameraSubscriber() : Node("camera_subscriber")
    {
        subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/d435/color/image_raw", 10, std::bind(&CameraSubscriber::image_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
        this->get_logger(), "Subscribed to /d435/color/image_raw topic. Waiting for images...");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), "Received an image! %u x %u | encoding: %s", msg->width, msg->height, msg->encoding.c_str());

        cv_bridge::CvImagePtr cv_ptr;
        try{
            cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        } catch (cv_bridge::Exception& e){
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        cv::imshow("Camera View", cv_ptr->image);
        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CameraSubscriber>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
