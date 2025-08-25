#include <iostream>
#include <vector>
#include <numeric>

#include <librealsense2/rs.hpp> // RealSense SDK
#include <opencv2/opencv.hpp>   // OpenCV (core, highgui, imgproc, objdetectを含む)

// ピクセル座標(2D)と深度から3D座標を取得する
// point_3dは3つの要素を持つfloat配列へのポインタであること
void get_point_3d(float point_3d[3], const rs2_intrinsics& intrinsics, const rs2::depth_frame& depth_frame, int x, int y) {
    float distance = depth_frame.get_distance(x, y);
    if (distance > 0) {
        float pixel[2] = { static_cast<float>(x), static_cast<float>(y) };
        rs2_deproject_pixel_to_point(point_3d, &intrinsics, pixel, distance);
    } else {
        // 深度が0の場合は無効な座標として0をセット
        point_3d[0] = 0; point_3d[1] = 0; point_3d[2] = 0;
    }
}
// ピクセル単位の長さをメートル単位の長さに変換する (ピンホールカメラモデル)
float get_length_from_pixel(float depth, float length_in_pixel, float focal_length_x) {
    return (depth * length_in_pixel) / focal_length_x;
}

int main() try {
    // --- 1. 定数と変数の初期化 ---

    const int width = 1280;
    const int height = 720;

    // 青色検出のHSV範囲 (H: 80-110, S: 100-255, V: 200-255)
    cv::Scalar lower_red(80, 100, 200);
    cv::Scalar upper_red(110, 255, 255);

    // --- 2. RealSenseの初期化 ---
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, 30);
    rs2::pipeline_profile profile = pipe.start(cfg);

    // ★★★ 自動露出を無効にする ★★★
    auto color_sensor = profile.get_device().query_sensors()[1]; // 0: Stereo Module, 1: RGB Camera
    color_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);

    // ★★★ カラーカメラの焦点距離(fx)を取得 (半径の計算用) ★★★
    auto color_profile = rs2::video_stream_profile(profile.get_stream(RS2_STREAM_COLOR));
    float fx = color_profile.get_intrinsics().fx;

    // 深度カメラの内部パラメータを取得 (3D座標への変換用)
    rs2_intrinsics depth_intrinsics = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
    
    // 位置合わせオブジェクトの作成
    rs2::align align(RS2_STREAM_COLOR);

    while (true) {
        rs2::frameset frames = pipe.wait_for_frames();
        auto aligned_frames = align.process(frames);
        rs2::video_frame color_frame = aligned_frames.get_color_frame();
        rs2::depth_frame depth_frame = aligned_frames.get_depth_frame();
        if (!color_frame || !depth_frame) continue;

        cv::Mat bgr_image(cv::Size(width, height), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat hsv_image;
        cv::cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);

        cv::Mat binary_image;
        cv::inRange(hsv_image, lower_red, upper_red, binary_image);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        cv::Mat circle_image = bgr_image.clone();

        for (const auto& cnt : contours) {
            cv::Point2f center;
            float radius_in_pixel;
            cv::minEnclosingCircle(cnt, center, radius_in_pixel);
            
            float obj_point_3d[3];
            get_point_3d(obj_point_3d, depth_intrinsics, depth_frame, static_cast<int>(center.x), static_cast<int>(center.y));

            if (radius_in_pixel > 30 && obj_point_3d[2] > 0) {
                cv::circle(circle_image, center, static_cast<int>(radius_in_pixel), cv::Scalar(0, 255, 0), 2);

                // 半径をピクセルからメートルに変換
                float radius_in_meters = get_length_from_pixel(obj_point_3d[2], radius_in_pixel, fx);

                std::cout << "obj_point_from_origin: ["
                          << obj_point_3d[0] << ", "
                          << obj_point_3d[1] << ", "
                          << obj_point_3d[2] << "], "
                          << "radius: " << radius_in_meters << " meters" << std::endl;
            }
        }

        cv::imshow("Enclosing Circles", circle_image);
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