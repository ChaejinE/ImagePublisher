#include "image_publisher.hpp"

using namespace std::chrono_literals;

ImagePublisher::ImagePublisher(const rclcpp::NodeOptions &node_options)
    : Node("video_publisher", node_options)
{
    this->declare_params();
    this->read_params();

    for (std::string &camera_id : m_camera_id_list)
    {
        std::filesystem::path video_file_path = m_video_dir_path / camera_id / m_video_file_name;
        if (!(std::filesystem::exists(video_file_path)))
        {
            std::cout << video_file_path << " does not exist" << std::endl;
            continue;
        }

        ImageTransFormer transofrmer(camera_id, video_file_path, this->get_clock(),
                                     m_start_frame_number, m_using_frame_number);
        m_image_transformer_map[camera_id] = transofrmer;

        auto pub_callback = [this, &camera_id]() -> void
        { publish_callback(camera_id); };
        m_image_publisher_map[camera_id] = this->create_publisher<sensor_msgs::msg::CompressedImage>(camera_id + "/image/compressed", 1);
        m_publisher_timer_map[camera_id] = this->create_wall_timer(std::chrono::seconds(1 / m_fps), pub_callback);

        auto timer_callback = [this]() -> void
        { fps_callback(); };
        m_fps_timer = this->create_wall_timer(std::chrono::seconds(5), timer_callback);
    }
}

void ImagePublisher::declare_params()
{
    this->declare_parameter("camera_id_list", "/cam_f/image/compressed");
    this->declare_parameter("video_dir_path", nullptr);
    this->declare_parameter("video_file_name", nullptr);
    this->declare_parameter("fps", 15);
    this->declare_parameter("using_frame_number", -1);
    this->declare_parameter("start_frame_number", 0);
}

void ImagePublisher::read_params()
{
    m_camera_id_list = this->get_parameter("camera_id_list").get_value<std::vector<std::string>>();
    m_video_dir_path = this->get_parameter("video_file_path").get_value<std::filesystem::path>();
    m_video_file_name = this->get_parameter("video_file_name").get_value<std::string>();
    m_fps = this->get_parameter("fps").get_value<int8_t>();
    m_using_frame_number = this->get_parameter("using_frame_number").get_value<int>();
    m_start_frame_number = this->get_parameter("start_frame_number").get_value<int>();
}

void ImagePublisher::publish_callback(std::string &camera_id)
{
    m_image_publisher_map[camera_id]->publish(m_image_transformer_map[camera_id].get_message());
}

void ImagePublisher::fps_callback()
{
    for (std::pair<std::string, ImageTransFormer> kv : m_image_transformer_map)
    {
        ImageTransFormer img_transformer = kv.second;
        std::string fps = img_transformer.get_fps();
        RCLCPP_INFO(this->get_logger(), "FPS : '%s'", fps.c_str());
    }
}