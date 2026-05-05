#pragma once

#include <spdlog/fmt/ranges.h>

#include <QJsonDocument>
#include <QJsonObject>
#include <QObject>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

class rs2::frameset;

namespace tracking {

struct ObjectPose {
    double x, y, z;           // Position in world coordinates
    double roll, pitch, yaw;  // Orientation in world coordinates

    std::string to_json() const {
        return fmt::format(
            R"({{"position": {{"x": {:.4f}, "y": {:.4f}, "z": {:.4f}}}, "orientation": {{"roll": {:.4f}, "pitch": {:.4f}, "yaw": {:.4f}}}}})",
            x, y, z, roll, pitch, yaw);
    };

    static ObjectPose from_json(const std::string& json_str) {
        QJsonObject root = QJsonDocument::fromJson(QByteArray::fromStdString(json_str)).object();
        QJsonObject pos = root["position"].toObject();
        QJsonObject ori = root["orientation"].toObject();
        return ObjectPose{pos["x"].toDouble(),    pos["y"].toDouble(),     pos["z"].toDouble(),
                          ori["roll"].toDouble(), ori["pitch"].toDouble(), ori["yaw"].toDouble()};
    }
};

struct CameraParameters;  // Forward declaration

struct MarkerParameter;  // Forward declaration

class TrackerConfig;

class VisionTracker : public QObject {
    Q_OBJECT
   public:
    VisionTracker(std::shared_ptr<tracking::TrackerConfig> config, QObject* parent = nullptr);

    ~VisionTracker();

    void process_frames(const int camera_id, const std::string& serial, const rs2::frameset& frames);

   signals:
    void error_occurred(const QString& error_message);
    void frames_received(std::vector<std::tuple<int, std::string, QImage>> frames);
    void publish_message(const std::string message);
    void update_camera_status(std::string serial, std::string status);

   private:
    cv::Mat preprocess_frame(const std::string& serial, const rs2::frameset& frame);

    bool calibrate_camera(CameraParameters& cam_params, std::vector<int>& marker_ids,
                          std::vector<std::vector<cv::Point2f>>& marker_corners);

    std::map<int, MarkerParameter> m_marker_parameters;

    cv::aruco::ArucoDetector m_aruco_detector;

    std::shared_ptr<tracking::TrackerConfig> m_config;

    std::vector<CameraParameters> m_camera_parameters;

    std::unique_ptr<MarkerParameter> m_benchmark_parameter;

    std::map<std::string, cv::Mat> m_cache_frames;
};
}  // namespace tracking