#include "rclcpp/rclcpp.hpp"
#include "rclcpp/create_timer.hpp"
#include <sensor_msgs/msg/compressed_image.hpp>
#include "image_transformer.hpp"

#include <map>
#include <filesystem>
#include <iostream>
#include <functional>

class ImagePublisher : public rclcpp::Node
{
public:
    explicit ImagePublisher(const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions());
    virtual ~ImagePublisher();

    void declare_params();
    void publish_callback(std::string &camera_id);
    void fps_callback();

private:
    std::vector<std::string> m_camera_id_list;
    std::filesystem::path m_video_dir_path;
    std::string m_video_file_name;
    int8_t m_fps;
    int m_using_frame_number;
    int m_start_frame_number;

    std::map<std::string, ImageTransFormer> m_image_transformer_map;
    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr> m_image_publisher_map;
    std::map<std::string, rclcpp::TimerBase::SharedPtr> m_publisher_timer_map;
    rclcpp::TimerBase::SharedPtr m_fps_timer;

    void read_params();
};