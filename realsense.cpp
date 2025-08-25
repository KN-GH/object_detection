#include "realsense.hpp"
#include <iostream>
#include <vector>
#include <numeric>

RealsenseOperatorClass::RealsenseOperatorClass(int width, int height)
    : _height(height), _width(width), _align(RS2_STREAM_DEPTH), _depth_frame(nullptr)
{
    this->_cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, 30);
    this->_cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, 30);
}

RealsenseOperatorClass::~RealsenseOperatorClass()
{}

int RealsenseOperatorClass::start_camera()
{
    try
    {
        this->_profile = this->_pipe.start(this->_cfg);
        auto color_sensor = this->_profile.get_device().query_sensors()[1];
        color_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);

        auto color_profile = rs2::video_stream_profile(this->_profile.get_stream(RS2_STREAM_COLOR));
        this->_fx = color_profile.get_intrinsics().fx;

        this->_depth_intrinsics = this->_profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
        return EXIT_SUCCESS;
    }
    catch (const rs2::error &e)
    {
        std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}

void RealsenseOperatorClass::stop_camera()
{
    this->_pipe.stop();
}

int RealsenseOperatorClass::get_image(cv::Mat& image)
{
    try
    {
        rs2::frameset frames = _pipe.wait_for_frames();
        auto aligned_frames = _align.process(frames);
        rs2::video_frame color_frame = aligned_frames.get_color_frame();
        this->_depth_frame = aligned_frames.get_depth_frame();
        if (!color_frame || !this->_depth_frame)
            return EXIT_FAILURE;
        cv::Mat bgr_image(cv::Size(this->_width, this->_height), CV_8UC3, (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);
        image = bgr_image;
        return EXIT_SUCCESS;
    }
    catch (const rs2::error &e)
    {
        std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}

int RealsenseOperatorClass::get_point3d_and_length(float point3d[3], int x, int y, float& length, int length_in_pixel)
{
    float depth = this->_depth_frame.get_distance(x, y);
    float pixel[2] = {static_cast<float>(x), static_cast<float>(y)};
    rs2_deproject_pixel_to_point(point3d, &(this->_depth_intrinsics), pixel, depth);
    length = depth * length_in_pixel / this->_fx;
    return EXIT_SUCCESS;
}