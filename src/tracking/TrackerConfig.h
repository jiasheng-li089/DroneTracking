#pragma once

#include <map>
#include <string>

#include <opencv2/opencv.hpp>

namespace tracking {

struct MarkerParameter {
    int id;
    double angle;
    float size;

    cv::Mat drone_marker_pose = cv::Mat::eye(4, 4, CV_64F);  // 4x4 transformation matrix from marker frame to drone body frame

    cv::Mat obj_points;

    static MarkerParameter create(int id, double angle, float size, cv::Mat drone_marker_pose = cv::Mat::eye(4, 4, CV_64F)) {
        auto result = MarkerParameter{id, angle, size, drone_marker_pose, cv::Mat(4, 1, CV_32FC3)};

        float half_size = size / 2.0f;
        result.obj_points.at<cv::Vec3f>(0, 0) = cv::Vec3f(-half_size, half_size, 0);   // Top-left
        result.obj_points.at<cv::Vec3f>(1, 0) = cv::Vec3f(half_size, half_size, 0);    // Top-right
        result.obj_points.at<cv::Vec3f>(2, 0) = cv::Vec3f(half_size, -half_size, 0);   // Bottom-right
        result.obj_points.at<cv::Vec3f>(3, 0) = cv::Vec3f(-half_size, -half_size, 0);  // Bottom-left

        return result;
    }
};

struct CameraParameters {
    cv::Mat K;  // Intrinsic matrix
    cv::Mat D;  // Distortion coefficients
    cv::Mat pose; // 4x4 transformation matrix from camera frame to world frame (benchmark marker frame)

    bool calibrated = false;  // Flag to indicate if the camera has been calibrated

    std::string serial;  // Camera serial number for identification
};

class TrackerConfig {
   public:
    TrackerConfig(const std::string& config_file_path);

    ~TrackerConfig() = default;

    std::map<int, MarkerParameter> get_marker_parameters() const;

    MarkerParameter get_benchmark_parameter() const;

    cv::aruco::DetectorParameters get_aruco_detector_parameters() const;

    cv::aruco::Dictionary get_aruco_dictionary() const;

    std::vector<CameraParameters> get_camera_calibration_parameters() const;

   private:
    cv::FileStorage m_fs;

};

};  // namespace tracking
