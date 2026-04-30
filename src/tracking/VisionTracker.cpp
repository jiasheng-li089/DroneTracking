# include "VisionTracker.h"
#include "TrackerConfig.h"

#include <spdlog/spdlog.h>

namespace tracking
{
    
VisionTracker::VisionTracker(std::shared_ptr<tracking::TrackerConfig> config, QObject *parent) : m_config(config), QObject(parent) {

    // Initialize ArUco detectors based on the configuration
    try {
        auto marker_params = m_config->get_marker_parameters();
        auto detector_params = m_config->get_aruco_detector_parameters();
        auto dictionary = m_config->get_aruco_dictionary();

        for (const auto& [id, params] : marker_params) {
            cv::aruco::ArucoDetector detector(dictionary, detector_params);
            m_aruco_detectors[id] = detector;
            m_aurcode_angles[id] = params.angle;
        }
    } catch (const std::exception& ex) {
        spdlog::error("Error initializing VisionTracker: {}", ex.what());
        throw;  // Rethrow to indicate initialization failure
    }
}

VisionTracker::~VisionTracker() = default;

void VisionTracker::process_frames(const int camera_id, const std::string & serial, const rs2::frameset& frames) {
    // detect the possible position of the target (aruco) related to the camera.


    // get the real distance of the target (aruco) from the camera


    // based on the real distance and the related position, calculate the relative position of the target (aruco) to the camera


    // average the relative positions of the target (aruco) to the cameras to get a stable position estimation of the target (aruco)
}
} // namespace tracking
