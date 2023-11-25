#include <cstdio>
#include <iostream>
#include <string>
#include <memory>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <sstream> 

constexpr int sensor_mode = 4;
constexpr float fps = 30.00;
constexpr int width = 1640;
constexpr int height = 1232;


class CSICameraPublisher : public rclcpp::Node {
public:
    CSICameraPublisher()
        : Node("camera") {
        
        declare_parameter("num_cameras", 1);

        get_parameter("num_cameras", num_cameras_);

        // Create an image publisher
        left_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("left/image_raw", 10);

        if(num_cameras_ > 1) {
          right_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("right/image_raw", 10);
        }

        // Initialize the camera
        if (!initCamera()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize the camera.");
            rclcpp::shutdown();
        }

        // Start capturing and publishing
        captureAndPublish();
    }

  ~CSICameraPublisher() {
    deinitCamera();
  }

  void deinitCamera() {
    if(camera_0_.isOpened()) {
        camera_0_.release();
    }

    if(num_cameras_ > 1 && camera_1_.isOpened()){
      camera_1_.release();
    }
  }

private:

    std::string buildLaunchStr(int port, int mode, int width, int height, int fps) {
        std::ostringstream ss;
        ss << "nvarguscamerasrc sensor_id=" << port << " sensor_mode=" << mode << " ! video/x-raw(memory:NVMM), width=(int)" << width << ", height=(int)" << height << ",format=(string)NV12, framerate=(fraction)" << fps << "/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
        return ss.str();
    }

    bool initCamera() {
        // Open the camera using OpenCV

        camera_0_ = cv::VideoCapture(buildLaunchStr(0, 3, 1640, 1232, 30));
    

        if (!camera_0_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the camera.");
            return false;
        }

        if(num_cameras_ > 1) {
          camera_1_ = cv::VideoCapture(buildLaunchStr(1, 3, 1640, 1232, 30));

          if (!camera_1_.isOpened()) {
              RCLCPP_ERROR(this->get_logger(), "Failed to open the camera.");
              return false;
          }
        }
        return true;
    }

    void captureAndPublish() {
        // Main loop to capture and publish images
        while (rclcpp::ok()) {
            cv::Mat frame_0;
            camera_0_ >> frame_0;
            if (!frame_0.empty()) {
                // Convert OpenCV image to ROS2 sensor message
                auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_0).toImageMsg();

                // Publish the image
                if(num_cameras_ > 1) {
                  right_image_publisher_->publish(*img_msg);
                } else {
                  left_image_publisher_->publish(*img_msg);
                }
            }

            if(num_cameras_ > 1) {
              cv::Mat frame_1;
              camera_1_ >> frame_1;
              if (!frame_1.empty()) {
                  // Convert OpenCV image to ROS2 sensor message
                  auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_1).toImageMsg();
                  
                  // Publish the image
                  left_image_publisher_->publish(*img_msg);
              }
            }

            // Sleep for a while to control the publishing rate
            rclcpp::sleep_for(std::chrono::milliseconds(33));
        }
    }

    int num_cameras_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_image_publisher_;
    
    rclcpp::Clock clock_;
    cv::VideoCapture camera_0_;
    cv::VideoCapture camera_1_;
    
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CSICameraPublisher>();

    rclcpp::spin(node);
    node->deinitCamera();
    rclcpp::shutdown();

    return 0;
}

/*

[mono_node-1] GST_ARGUS: 3264 x 2464 FR = 21.000000 fps Duration = 47619048 ; Analog Gain range min 1.000000, max 10.625000; Exposure Range min 13000, max 683709000;
[mono_node-1] 
[mono_node-1] GST_ARGUS: 3264 x 1848 FR = 28.000001 fps Duration = 35714284 ; Analog Gain range min 1.000000, max 10.625000; Exposure Range min 13000, max 683709000;
[mono_node-1] 
[mono_node-1] GST_ARGUS: 1920 x 1080 FR = 29.999999 fps Duration = 33333334 ; Analog Gain range min 1.000000, max 10.625000; Exposure Range min 13000, max 683709000;
[mono_node-1] 
[mono_node-1] GST_ARGUS: 1640 x 1232 FR = 29.999999 fps Duration = 33333334 ; Analog Gain range min 1.000000, max 10.625000; Exposure Range min 13000, max 683709000;
[mono_node-1] 
[mono_node-1] GST_ARGUS: 1280 x 720 FR = 59.999999 fps Duration = 16666667 ; Analog Gain range min 1.000000, max 10.625000; Exposure Range min 13000, max 683709000;
[mono_node-1] 
[mono_node-1] GST_ARGUS: 1280 x 720 FR = 120.000005 fps Duration = 8333333 ; Analog Gain range min 1.000000, max 10.625000; Exposure Range min 13000, max 683709000;
*/