#include <iostream>
#include <vector>
#include <numeric>

#include <librealsense2/rs.hpp> // RealSense SDK
#include <opencv2/opencv.hpp>   // OpenCV (core, highgui, imgproc, objdetectを含む)

#define MAX_DISTANCE 10.0f
int main() try {
    const int width = 1280;
    const int height = 720;

    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, 30);
    rs2::pipeline_profile profile = pipe.start(cfg);

    // 位置合わせオブジェクトの作成
    rs2::align align(RS2_STREAM_COLOR);

    rs2::depth_sensor depth_sensor = profile.get_device().first<rs2::depth_sensor>();
    float depth_scale = depth_sensor.get_depth_scale();

    while (true) {
        rs2::frameset frames = pipe.wait_for_frames();
        auto aligned_frames = align.process(frames);
        rs2::video_frame color_frame = aligned_frames.get_color_frame();
        rs2::depth_frame depth_frame = aligned_frames.get_depth_frame();
        if (!color_frame || !depth_frame) continue;
        
        // 0 ~ 255の深度を表すグレースケールを出力
        cv::Mat depth_gray_image_16bit(cv::Size(width, height), CV_16UC1, (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);
        double alpha = 255.0f * (depth_scale / MAX_DISTANCE);
        cv::Mat depth_gray_image_8bit;
        depth_gray_image_16bit.convertTo(depth_gray_image_8bit, CV_8U, alpha);
        
        // モルフォロジー変換 オープニング
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::Mat depth_opened;
        morphologyEx(depth_gray_image_8bit, depth_opened, cv::MORPH_OPEN, kernel);
        cv::Mat depth_closed;
        morphologyEx(depth_opened, depth_closed, cv::MORPH_CLOSE, kernel);
        
        cv::imshow("Depth image", depth_closed);
        if (cv::waitKey(1) == 'q') break;
    }

    pipe.stop();
    cv::destroyAllWindows();
    return EXIT_SUCCESS;
}
// エラーハンドリング
catch (const rs2::error & e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}