#include "image_transformer.hpp"

ImageTransFormer::ImageTransFormer(std::string camera_id, std::filesystem::path video_file_path,
                                   rclcpp::Clock::SharedPtr ros_clock, int start_frame_number, int using_frame_number)
    : m_camera_id(camera_id), m_video_file_path(video_file_path), m_ros_clock(ros_clock),
      m_start_frame_number(start_frame_number), m_using_frame_number(using_frame_number), m_fps_time(std::chrono::system_clock::now()),
      m_frame_cnt(0), m_image_idx(0), m_img_msg_list(this->video_to_msg_list()){};

ImageTransFormer::~ImageTransFormer(){};

sensor_msgs::msg::CompressedImage ImageTransFormer::get_message()
{
  if (m_image_idx >= m_img_msg_list.size())
  {
    auto time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::cout << m_camera_id << ' ' << std::ctime(&time) << " : go back to start" << std::endl;
    m_image_idx = 0;
  }

  sensor_msgs::msg::CompressedImage msg = m_img_msg_list[m_image_idx];
  msg.header.stamp = m_ros_clock->now();
  msg.header.frame_id = "test_image_publisher_" + std::to_string(m_start_frame_number);

  ++m_image_idx;
  ++m_frame_cnt;

  return msg;
};

std::string ImageTransFormer::get_fps()
{
  m_frame_cnt = 0;
  m_fps_time = std::chrono::system_clock::now();
  return "";
};

std::vector<sensor_msgs::msg::CompressedImage> ImageTransFormer::video_to_msg_list()
{
  auto cap = cv::VideoCapture(m_video_file_path.u8string());

  cap.set(cv::CAP_PROP_POS_FRAMES, m_start_frame_number);

  if (m_using_frame_number < 0)
    m_using_frame_number = cap.get(cv::CAP_PROP_FRAME_COUNT);

  const cv::Mat frame;
  cv_bridge::CvImagePtr cv_ptr;
  std::vector<sensor_msgs::msg::CompressedImage> img_msg_list(m_using_frame_number);
  for (auto i = 0; i <= m_using_frame_number; ++i)
  {
    if (!cap.isOpened())
      break;

    cap.read(frame);

    try
    {
      cv_bridge::CvImage cvImage;
      cvImage.encoding = sensor_msgs::image_encodings::BGR8;
      cvImage.image = frame;
      img_msg_list.emplace_back(*cvImage.toCompressedImageMsg());
    }
    catch (cv_bridge::Exception &e)
    {
      std::cout << e.what() << std::endl;
    }
  }

  return img_msg_list;
};