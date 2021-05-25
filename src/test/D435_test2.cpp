#include <ros/ros.h>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

int main(void)
{
  rs2::colorizer color_map;

  rs2::config cfg;
  cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
  cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);

  rs2::pipeline pipe;
  pipe.start(cfg);

  while(cv::waitKey(1) == -1) {
    rs2::frameset frames = pipe.wait_for_frames();
    rs2::frame color_frame = frames.get_color_frame();
    rs2::frame depth_frame = color_map(frames.get_depth_frame());

    cv::Mat color(cv::Size(1280, 720), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat depth(cv::Size(1280, 720), CV_8UC3, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

    cv::namedWindow("color", cv::WINDOW_AUTOSIZE);
    cv::imshow("color", color);
    cv::namedWindow("depth", cv::WINDOW_AUTOSIZE);
    cv::imshow("depth", depth);
  }
  return 0;
}