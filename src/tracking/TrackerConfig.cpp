#include "TrackerConfig.h"

#include <stdexcept>

namespace tracking {

TrackerConfig::TrackerConfig(const std::string& config_file_path) {
    m_fs.open(config_file_path, cv::FileStorage::READ);
    if (!m_fs.isOpened())
        throw std::runtime_error("Failed to open config file: " + config_file_path);
}

std::map<int, MarkerParameter> TrackerConfig::get_marker_parameters() const {
    std::map<int, MarkerParameter> marker_parameters;

    cv::FileNode marker_node = m_fs["marker"];
    if (marker_node.empty() || !marker_node.isMap())
        throw std::runtime_error("Invalid or missing 'marker' configuration");

    for (const auto& entry : marker_node) {
        int id = std::stoi(entry.name());
        double angle;
        float size;
        entry["angle"] >> angle;
        entry["size"] >> size;
        marker_parameters[id] = MarkerParameter{angle, size};
    }

    return marker_parameters;
}

cv::aruco::DetectorParameters TrackerConfig::get_aruco_detector_parameters() const {
    cv::aruco::DetectorParameters params;

    cv::FileNode det_node = m_fs["detector"];
    if (det_node.empty() || !det_node.isMap())
        throw std::runtime_error("Invalid or missing 'detector' configuration");

    cv::FileNode params_node = det_node["parameters"];
    if (params_node.empty() || !params_node.isMap())
        throw std::runtime_error("Invalid or missing 'aruco_detector_parameters' configuration");

    if (auto n = params_node["adaptiveThreshWinSizeMin"]; !n.empty()) n >> params.adaptiveThreshWinSizeMin;
    if (auto n = params_node["adaptiveThreshWinSizeMax"]; !n.empty()) n >> params.adaptiveThreshWinSizeMax;
    if (auto n = params_node["adaptiveThreshWinSizeStep"]; !n.empty()) n >> params.adaptiveThreshWinSizeStep;
    if (auto n = params_node["adaptiveThreshConstant"]; !n.empty()) n >> params.adaptiveThreshConstant;
    if (auto n = params_node["minMarkerPerimeterRate"]; !n.empty()) n >> params.minMarkerPerimeterRate;
    if (auto n = params_node["maxMarkerPerimeterRate"]; !n.empty()) n >> params.maxMarkerPerimeterRate;
    if (auto n = params_node["polygonalApproxAccuracyRate"]; !n.empty()) n >> params.polygonalApproxAccuracyRate;
    if (auto n = params_node["minCornerDistanceRate"]; !n.empty()) n >> params.minCornerDistanceRate;
    if (auto n = params_node["minDistanceToBorder"]; !n.empty()) n >> params.minDistanceToBorder;
    if (auto n = params_node["errorCorrectionRate"]; !n.empty()) n >> params.errorCorrectionRate;
    // Add more parameters as needed

    return params;
}

cv::aruco::Dictionary TrackerConfig::get_aruco_dictionary() const {
    cv::FileNode det_node = m_fs["detector"];
    if (det_node.empty() || !det_node.isMap())
        throw std::runtime_error("Invalid or missing 'detector' configuration");

    cv::FileNode dict_node = det_node["dictionary"];
    if (dict_node.empty() || !dict_node.isString())
        throw std::runtime_error("Invalid or missing 'dictionary' configuration in 'detector'");

    std::string dict_name;
    dict_node >> dict_name;

    static const std::map<std::string, int> dict_name_map = {
        {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL},
        {"DICT_4X4_50",   cv::aruco::DICT_4X4_50},
        {"DICT_4X4_100",  cv::aruco::DICT_4X4_100},
        {"DICT_4X4_250",  cv::aruco::DICT_4X4_250},
        {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
        {"DICT_5X5_50",   cv::aruco::DICT_5X5_50},
        {"DICT_5X5_100",  cv::aruco::DICT_5X5_100},
        {"DICT_5X5_250",  cv::aruco::DICT_5X5_250},
        {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
        {"DICT_6X6_50",   cv::aruco::DICT_6X6_50},
        {"DICT_6X6_100",  cv::aruco::DICT_6X6_100},
        {"DICT_6X6_250",  cv::aruco::DICT_6X6_250},
        {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
        {"DICT_7X7_50",   cv::aruco::DICT_7X7_50},
        {"DICT_7X7_100",  cv::aruco::DICT_7X7_100},
        {"DICT_7X7_250",  cv::aruco::DICT_7X7_250},
        {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000},
    };

    auto it = dict_name_map.find(dict_name);
    if (it == dict_name_map.end())
        throw std::runtime_error("Unsupported ArUco dictionary: " + dict_name);

    return cv::aruco::getPredefinedDictionary(it->second);
}

const std::map<std::string, std::map<std::string, cv::Mat>>& TrackerConfig::get_camera_calibration_parameters() const {
    if (m_cameras_parameters.empty()) {
        std::string camera1_serial, camera2_serial;
        m_fs["camera1_serial"] >> camera1_serial;
        m_fs["camera2_serial"] >> camera2_serial;

        cv::FileNode cameras_node = m_fs["cameras"];
        if (cameras_node.empty() || !cameras_node.isMap())
            throw std::runtime_error("Invalid or missing 'cameras' configuration");

        for (const auto& serial : {camera1_serial, camera2_serial}) {
            cv::FileNode cam_node = cameras_node[serial];
            if (cam_node.empty() || !cam_node.isMap())
                throw std::runtime_error("Missing or invalid camera configuration for: " + serial);

            std::map<std::string, cv::Mat> cam_params;

            cv::Mat K, D;
            cam_node["K"] >> K;
            cam_node["D"] >> D;

            if (K.empty() || K.rows != 3 || K.cols != 3)
                throw std::runtime_error("Invalid or missing 'K' matrix for camera: " + serial);
            if (D.empty())
                throw std::runtime_error("Invalid or missing 'D' vector for camera: " + serial);

            cam_params["K"] = K;
            cam_params["D"] = D;
            m_cameras_parameters[serial] = std::move(cam_params);
        }
    }
    return m_cameras_parameters;
}

}  // namespace tracking
