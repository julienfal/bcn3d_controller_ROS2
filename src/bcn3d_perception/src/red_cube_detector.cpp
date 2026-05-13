#include "bcn3d_perception/filter_color.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "image_geometry/pinhole_camera_model.h"


class CameraSubscriber : public rclcpp::Node
{
public:
    CameraSubscriber() : Node("camera_subscriber")
    {
        subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/d435/color/image_raw", 10, std::bind(&CameraSubscriber::image_callback, this, std::placeholders::_1));

        subscriber_intrinsic_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/d435/color/camera_info", 10, std::bind(&CameraSubscriber::camera_info_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
        this->get_logger(), "Subscribed to /d435/color/image_raw topic. Waiting for images...");
    }

    cv::Mat frame, frame_HSV, frame_threshold;


private:

    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg){
        camera_model_.fromCameraInfo(msg);
        has_camera_info_ = true;

        RCLCPP_INFO_ONCE(
            this->get_logger(),
            "Received camera info: fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
            camera_model_.fx(),
            camera_model_.fy(),
            camera_model_.cx(),
            camera_model_.cy());
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), "Received an image! %u x %u | encoding: %s", msg->width, msg->height, msg->encoding.c_str());

        if (!has_camera_info_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Waiting for camera_info...");
            return;
            }
        cv_bridge::CvImagePtr cv_ptr;
        try{
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e){
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        cv::cvtColor(cv_ptr->image, frame_HSV, cv::COLOR_BGR2HSV);

        cv::Mat red_mask_low;
        cv::Mat red_mask_high;
        cv::inRange(frame_HSV, cv::Scalar(0, 80, 40), cv::Scalar(12, 255, 255), red_mask_low);
        cv::inRange(frame_HSV, cv::Scalar(170, 80, 40), cv::Scalar(180, 255, 255), red_mask_high);
        cv::bitwise_or(red_mask_low, red_mask_high, frame_threshold);

        cv::imshow("Camera View", cv_ptr->image);
        cv::imshow("red cube", frame_threshold);

        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscriber_intrinsic_;
    image_geometry::PinholeCameraModel camera_model_;
    bool has_camera_info_ = false;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CameraSubscriber>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
