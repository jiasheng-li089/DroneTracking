#pragma once

#include <map>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/video/tracking.hpp>
#include <spdlog/fmt/ranges.h>

#include <QJsonDocument>
#include <QJsonObject>
#include <QObject>
#include <opencv2/opencv.hpp>
#include <string>
#include <unordered_map>

#include "../common/common.h"

namespace tracking {

struct ObjectPose {
  double x, y, z;          // Position in world coordinates
  double roll, pitch, yaw; // Orientation in world coordinates

  long timestamp =
      0; // Timestamp of the pose estimation, can be used for synchronization

  std::string to_json() const {
    return fmt::format(
        R"({{"position": {{"x": {:.4f}, "y": {:.4f}, "z": {:.4f}}}, "orientation": {{"roll": {:.4f}, "pitch": {:.4f}, "yaw": {:.4f}}}, "timestamp": {}}})",
        x, y, z, roll, pitch, yaw, timestamp);
  };

  static ObjectPose from_json(const std::string &json_str) {
    QJsonObject root =
        QJsonDocument::fromJson(QByteArray::fromStdString(json_str)).object();
    QJsonObject pos = root["position"].toObject();
    QJsonObject ori = root["orientation"].toObject();
    return ObjectPose{pos["x"].toDouble(),     pos["y"].toDouble(),
                      pos["z"].toDouble(),     ori["roll"].toDouble(),
                      ori["pitch"].toDouble(), ori["yaw"].toDouble(),
                      current_timestamp_ms()};
  }

  static ObjectPose from_t_and_r(const cv::Vec3d &t, const cv::Vec3d &r) {
    return ObjectPose{
        t[0], t[1], t[2], r[0], r[1], r[2], current_timestamp_ms()};
  }
};

struct CameraParameters; // Forward declaration

struct MarkerParameter; // Forward declaration

class TrackerConfig;

struct ComputationData {
  int64 last_frame_time;
  int64 last_computed_time;
  ObjectPose computed_pose;
  cv::KalmanFilter kf;
  cv::Mat show_marker_frame;
};

class VisionTracker : public QObject {
  Q_OBJECT
public:
  VisionTracker(std::shared_ptr<tracking::TrackerConfig> config,
                QObject *parent = nullptr);

  ~VisionTracker();

  void process_frames(const int camera_id, const std::string &serial,
                      ulong frame_id, const cv::Mat &frame);

signals:
  void error_occurred(const QString &error_message);
  void frame_received(int, const std::string &, ulong, const cv::Mat &);
  void publish_message(const std::string message);
  void update_camera_status(std::string serial, std::string status);

private:
  bool calibrate_camera(bool log, CameraParameters &cam_params,
                        std::vector<int> &marker_ids,
                        std::vector<cv::Vec3d> &rvecs,
                        std::vector<cv::Vec3d> &tvecs);

  std::map<int, MarkerParameter> m_marker_parameters;

  cv::aruco::ArucoDetector m_aruco_detector;

  std::shared_ptr<tracking::TrackerConfig> m_config;

  std::unordered_map<std::string, CameraParameters> m_camera_parameters;

  std::unique_ptr<MarkerParameter> m_benchmark_parameter;

  std::map<std::string, ComputationData> m_computation_data;

};

} // namespace tracking
