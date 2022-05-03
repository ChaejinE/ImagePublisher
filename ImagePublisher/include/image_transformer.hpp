#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/compressed_image.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <chrono>

class ImageTransFormer
{
public:
    ImageTransFormer(std::string camera_id, std::filesystem::path video_file_path,
                     rclcpp::Clock::SharedPtr ros_clock, int start_frame_number, int using_frame_number);
    ~ImageTransFormer();

    sensor_msgs::msg::CompressedImage get_message();
    std::string get_fps();
    std::vector<sensor_msgs::msg::CompressedImage> video_to_msg_list();

private:
    std::string m_camera_id;
    std::filesystem::path m_video_file_path;
    rclcpp::Clock::SharedPtr m_ros_clock;
    std::vector<sensor_msgs::msg::CompressedImage> m_img_msg_list;
    std::chrono::system_clock::time_point m_fps_time;
    int m_start_frame_number;
    int m_using_frame_number;
    int m_frame_cnt;
    int m_image_idx;
};
