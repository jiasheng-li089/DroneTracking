#pragma once

#include <QObject>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

class rs2::frameset;

namespace tracking {

class TrackerConfig;
class VisionTracker : public QObject {
    Q_OBJECT
   public:
    VisionTracker(std::shared_ptr<tracking::TrackerConfig> config, QObject* parent = nullptr);

    ~VisionTracker();

    void process_frames(const int camera_id, const std::string& serial, const rs2::frameset& frames);

   private:
    std::map<int, double> m_aurcode_angles;

    std::map<int, cv::aruco::ArucoDetector> m_aruco_detectors;

    std::shared_ptr<tracking::TrackerConfig> m_config;
};
}  // namespace tracking