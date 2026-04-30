#pragma once

#include <yaml-cpp/yaml.h>

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

    // Add configuration parameters as needed, e.g., camera parameters, tracking settings, etc.

    std::map<int, MarkerParameter> get_marker_parameters() const;

    cv::aruco::DetectorParameters get_aruco_detector_parameters() const;

    cv::aruco::Dictionary get_aruco_dictionary() const;

   private:
    YAML::Node m_config;
};

};  // namespace tracking
