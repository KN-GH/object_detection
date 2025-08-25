#ifndef __REALSENSE_HPP_
#define __REALSENSE_HPP_

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>  

class RealsenseOperatorClass {
    public:
      RealsenseOperatorClass(int width = 1280, int height = 720);
      ~RealsenseOperatorClass();
      int start_camera();
      void stop_camera();
      int get_image(cv::Mat& image);
      int get_point3d_and_length(float point3d[3], int x, int y, float& length, int pixel);

    private:
        int _width, _height;
        rs2::pipeline _pipe;
        rs2::config _cfg;
        
        rs2::pipeline_profile _profile;

        float _fx;

        // 深度カメラの内部パラメータを取得 (3D座標への変換用)
        rs2_intrinsics _depth_intrinsics;
        
        // 位置合わせオブジェクトの作成
        rs2::align _align;

        rs2::depth_frame _depth_frame;
};

#endif