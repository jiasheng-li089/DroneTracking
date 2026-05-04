#pragma once

#include <QObject>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

class rs2::frameset;

namespace tracking {

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

   private:
    cv::Mat preprocess_frame(const std::string& serial, const rs2::frameset& frame);

    std::map<int, MarkerParameter> m_marker_parameters;

    cv::aruco::ArucoDetector m_aruco_detector;

    std::shared_ptr<tracking::TrackerConfig> m_config;

    std::vector<CameraParameters> m_camera_parameters;

    std::map<std::string, cv::Mat> m_cache_frames;
};
}  // namespace tracking