#pragma once

#include <map>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>

namespace tracking {

struct MarkerParameter {
    double angle;
    float size;
};

class TrackerConfig {
   public:
    TrackerConfig(const std::string& config_file_path);

    ~TrackerConfig() = default;

    std::map<int, MarkerParameter> get_marker_parameters() const;

    cv::aruco::DetectorParameters get_aruco_detector_parameters() const;

    cv::aruco::Dictionary get_aruco_dictionary() const;

    const std::map<std::string, std::map<std::string, cv::Mat>>& get_camera_calibration_parameters() const;

   private:
    cv::FileStorage m_fs;

    mutable std::map<std::string, std::map<std::string, cv::Mat>> m_cameras_parameters;
};

};  // namespace tracking
