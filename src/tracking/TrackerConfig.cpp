#include "TrackerConfig.h"

namespace tracking {

TrackerConfig::TrackerConfig(const std::string& config_file_path) { m_config = YAML::LoadFile(config_file_path); }

std::map<int, MarkerParameter> TrackerConfig::get_marker_parameters() const {
    std::map<int, MarkerParameter> marker_parameters;

    if (m_config["marker"] && m_config["marker"].IsMap()) {
        for (const auto& entry : m_config["marker"]) {
            int id = entry.first.as<int>();
            double angle = entry.second["angle"].as<double>();
            float size = entry.second["size"].as<float>();
            marker_parameters[id] = MarkerParameter{angle, size};
        }
    } else {
        throw std::runtime_error("Invalid or missing 'marker' configuration");
    }

    return marker_parameters;
}

cv::aruco::DetectorParameters TrackerConfig::get_aruco_detector_parameters() const {
    cv::aruco::DetectorParameters params;

    if (m_config["detector"] && m_config["detector"].IsMap()) {
        const auto& det_params = m_config["detector"];
        if (det_params["parameters"] && det_params["parameters"].IsMap()) {
            const auto& params_node = det_params["parameters"];
            if (params_node["adaptiveThreshWinSizeMin"])
                params.adaptiveThreshWinSizeMin = params_node["adaptiveThreshWinSizeMin"].as<int>();
            if (params_node["adaptiveThreshWinSizeMax"])
                params.adaptiveThreshWinSizeMax = params_node["adaptiveThreshWinSizeMax"].as<int>();
            if (params_node["adaptiveThreshWinSizeStep"])
                params.adaptiveThreshWinSizeStep = params_node["adaptiveThreshWinSizeStep"].as<int>();
            if (params_node["adaptiveThreshConstant"])
                params.adaptiveThreshConstant = params_node["adaptiveThreshConstant"].as<double>();
            if (params_node["minMarkerPerimeterRate"])
                params.minMarkerPerimeterRate = params_node["minMarkerPerimeterRate"].as<double>();
            if (params_node["maxMarkerPerimeterRate"])
                params.maxMarkerPerimeterRate = params_node["maxMarkerPerimeterRate"].as<double>();
            if (params_node["polygonalApproxAccuracyRate"])
                params.polygonalApproxAccuracyRate = params_node["polygonalApproxAccuracyRate"].as<double>();
            if (params_node["minCornerDistanceRate"])
                params.minCornerDistanceRate = params_node["minCornerDistanceRate"].as<double>();
            if (params_node["minDistanceToBorder"])
                params.minDistanceToBorder = params_node["minDistanceToBorder"].as<int>();
            if (params_node["errorCorrectionRate"])
                params.errorCorrectionRate = params_node["errorCorrectionRate"].as<double>();
            // Add more parameters as needed
        } else {
            throw std::runtime_error("Invalid or missing 'aruco_detector_parameters' configuration");
        }
    } else {
        throw std::runtime_error("Invalid or missing 'detector' configuration");
    }

    return params;
};

cv::aruco::Dictionary TrackerConfig::get_aruco_dictionary() const {
    if (m_config["detector"] && m_config["detector"].IsMap()) {
        auto detector_node = m_config["detector"];
        if (!detector_node["dictionary"] || !detector_node["dictionary"].IsScalar()) {
            throw std::runtime_error("Invalid or missing 'dictionary' configuration in 'detector'");
        }
        std::string dict_name = detector_node["dictionary"].as<std::string>();

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
        if (it != dict_name_map.end()) {
            return cv::aruco::getPredefinedDictionary(it->second);
        } else {
            throw std::runtime_error("Unsupported ArUco dictionary: " + dict_name);
        }
    } else {
        throw std::runtime_error("Invalid or missing 'detector' configuration");
    }
}

}  // namespace tracking