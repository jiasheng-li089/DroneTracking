#include "TrackerConfig.h"

#include <stdexcept>

namespace tracking {

TrackerConfig::TrackerConfig(const std::string& config_file_path) {
    m_fs.open(config_file_path, cv::FileStorage::READ);
    if (!m_fs.isOpened()) throw std::runtime_error("Failed to open config file: " + config_file_path);
}

std::map<int, MarkerParameter> TrackerConfig::get_marker_parameters() const {
    std::map<int, MarkerParameter> marker_parameters;

    cv::FileNode marker_node = m_fs["marker"];
    if (marker_node.empty() || !marker_node.isSeq())
        throw std::runtime_error("Invalid or missing 'marker' configuration");

    for (const auto& entry : marker_node) {
        int id;
        double angle;
        float size;
        entry["id"] >> id;
        entry["angle"] >> angle;
        entry["size"] >> size;
        marker_parameters[id] = MarkerParameter::create(id, angle, size);
    }

    return std::move(marker_parameters);
}

cv::aruco::DetectorParameters TrackerConfig::get_aruco_detector_parameters() const {
    cv::aruco::DetectorParameters params;

    cv::FileNode det_node = m_fs["detector"];
    if (det_node.empty() || !det_node.isMap()) throw std::runtime_error("Invalid or missing 'detector' configuration");

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
    if (det_node.empty() || !det_node.isMap()) throw std::runtime_error("Invalid or missing 'detector' configuration");

    cv::FileNode dict_node = det_node["dictionary"];
    if (dict_node.empty() || !dict_node.isString())
        throw std::runtime_error("Invalid or missing 'dictionary' configuration in 'detector'");

    std::string dict_name;
    dict_node >> dict_name;

    static const std::map<std::string, int> dict_name_map = {
        {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL},
        {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
        {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
        {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
        {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
        {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
        {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
        {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
        {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
        {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
        {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
        {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
        {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
        {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
        {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
        {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
        {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000},
    };

    auto it = dict_name_map.find(dict_name);
    if (it == dict_name_map.end()) throw std::runtime_error("Unsupported ArUco dictionary: " + dict_name);

    return cv::aruco::getPredefinedDictionary(it->second);
}

std::vector<CameraParameters> TrackerConfig::get_camera_calibration_parameters() const {
    auto result = std::vector<CameraParameters>{};
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

        cv::Mat K, D, T, R;
        cam_node["K"] >> K;
        cam_node["D"] >> D;
        // cam_node["T"] >> T;
        // cam_node["R"] >> R;

        if (K.empty() || K.rows != 3 || K.cols != 3)
            throw std::runtime_error("Invalid or missing 'K' matrix for camera: " + serial);
        if (D.empty() || D.rows != 1 || D.cols != 5)
            throw std::runtime_error("Invalid or missing 'D' vector for camera: " + serial);
        // if (T.empty() || T.rows != 3 || T.cols != 1)
        //     throw std::runtime_error("Invalid or missing 'T' vector for camera: " + serial);
        // if (R.empty() || R.rows != 3 || R.cols != 3)
        //     throw std::runtime_error("Invalid or missing 'R' matrix for camera: " + serial);
 
        result.push_back(CameraParameters{K, D, T, R, false, serial});
    }
    return std::move(result);
}

MarkerParameter TrackerConfig::get_benchmark_parameter() const {
    cv::FileNode benchmark_node = m_fs["benchmark"];
    if (benchmark_node.empty() || !benchmark_node.isMap())
        throw std::runtime_error("Invalid or missing 'benchmark' configuration");

    int id;
    double angle;
    float size;
    benchmark_node["id"] >> id;
    benchmark_node["angle"] >> angle;
    benchmark_node["size"] >> size;

    return MarkerParameter::create(id, angle, size);
}

}  // namespace tracking
