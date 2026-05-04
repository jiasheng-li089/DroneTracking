#include "VisionTracker.h"

#include <spdlog/fmt/ranges.h>
#include <spdlog/spdlog.h>

#include <QImage>

#include "TrackerConfig.h"

namespace tracking {

VisionTracker::VisionTracker(std::shared_ptr<tracking::TrackerConfig> config, QObject* parent)
    : m_config(config), QObject(parent) {
    // Initialize ArUco detectors based on the configuration
    try {
        m_marker_parameters = m_config->get_marker_parameters();
        auto detector_params = m_config->get_aruco_detector_parameters();
        auto dictionary = m_config->get_aruco_dictionary();

        m_aruco_detector = cv::aruco::ArucoDetector(dictionary, detector_params);
        m_camera_parameters = m_config->get_camera_calibration_parameters();
    } catch (const std::exception& ex) {
        spdlog::error("Error initializing VisionTracker: {}", ex.what());
        throw;  // Rethrow to indicate initialization failure
    }
}

VisionTracker::~VisionTracker() = default;

void VisionTracker::process_frames(const int camera_id, const std::string& serial, const rs2::frameset& frames) {
    auto log_enable = frames.get_frame_number() % 30 == 0;

    // detect the possible position of the target (aruco) related to the camera.
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;
    auto rgb_frame = preprocess_frame(serial, frames);

    m_aruco_detector.detectMarkers(rgb_frame, marker_corners, marker_ids, rejected_candidates);

    if (log_enable)
        spdlog::debug("detected {} markers from cameraId ({}): [{}]", marker_corners.size(), serial,
                      fmt::join(marker_ids, ", "));

    // get the intrinsic and distortion parameters of the camera
    CameraParameters cam_params;

    auto it = m_camera_parameters.begin();
    for (; it != m_camera_parameters.end(); ++it) {
        if (it->serial == serial) {
            cam_params = *it;
            break;
        }
    }
    if (cam_params.K.empty() || cam_params.D.empty()) {
        if (log_enable) spdlog::warn("Camera parameters not found for serial: {}", serial);
        return;
    }

    // filter out all known markers, their related corners and their parameters (angle and size)
    std::vector<std::vector<cv::Point2f>> known_marker_corners;
    std::vector<MarkerParameter> known_marker_parameters;
    for (size_t i = 0; i < marker_ids.size(); ++i) {
        int id = marker_ids[i];
        auto param_it = m_marker_parameters.find(id);
        if (param_it != m_marker_parameters.end()) {
            known_marker_corners.push_back(marker_corners[i]);
            known_marker_parameters.push_back(param_it->second);
        } else {
            if (log_enable) spdlog::debug("Unknown marker detected with ID: {} from cameraId ({})", id, serial);
        }
    }
    if (known_marker_corners.empty()) {
        if (log_enable) spdlog::debug("No known markers detected from cameraId ({}), skipping pose estimation", serial);
        return;
    }
    auto size = known_marker_corners.size();
    if (log_enable)
        spdlog::debug("Processing {} known markers from cameraId ({}), IDs: [{}]", size, serial,
                      fmt::join(marker_ids, ", "));

    std::vector<cv::Vec3d> rvecs(size), tvecs(size);

    for (size_t i = 0; i < size; ++i) {
        const auto& marker_param = known_marker_parameters[i];
        const auto& marker_corner = known_marker_corners[i];

        // solvePnP to get the relative position of the target (aruco) to the camera
        cv::solvePnP(marker_param.obj_points, marker_corner, cam_params.K, cam_params.D, rvecs.at(i), tvecs.at(i));
    }

    if (log_enable)
        spdlog::debug("Pose estimation completed for cameraId ({}), marker IDs: [{}]", serial,
                      fmt::join(marker_ids, ", "));
    for (size_t i = 0; i < size; i++) {
        if (log_enable)
            spdlog::debug("Marker ID: {}, rvec: [{:.2f}, {:.2f}, {:.2f}], tvec: [{:.2f}, {:.2f}, {:.2f}]",
                          marker_ids[i], rvecs[i][0], rvecs[i][1], rvecs[i][2], tvecs[i][0], tvecs[i][1], tvecs[i][2]);
    }

    // generate a new image and mark the detected markers on the image, then emit the signal to update the GUI
    cv::Mat output_image = rgb_frame.clone();
    for (size_t i = 0; i < size; ++i) {
        cv::aruco::drawDetectedMarkers(output_image, std::vector<std::vector<cv::Point2f>>{known_marker_corners[i]},
                                       std::vector<int>{marker_ids[i]});
    }
    QImage qimg(output_image.data, output_image.cols, output_image.rows, static_cast<int>(output_image.step),
                QImage::Format_RGB888);
    emit frames_received(std::vector<std::tuple<int, std::string, QImage>>{{camera_id + 200, serial, qimg.copy()}});

    // convert marker positions from camera frame to world frame and log
    for (size_t i = 0; i < size; ++i) {
        // cam_params.R, cam_params.T: P_camera = R * P_world + T
        // inverted:                   P_world  = R^T * (P_camera - T)
        cv::Mat pos_world = cam_params.R.t() * (cv::Mat(tvecs.at(i)) - cam_params.T);
        if (log_enable)
            spdlog::info("Marker ID: {}, camera: {}, world pos (m): [{:.3f}, {:.3f}, {:.3f}]", marker_ids[i], serial,
                         pos_world.at<double>(0), pos_world.at<double>(1), pos_world.at<double>(2));
    }

    // average the relative positions of the target (aruco) to the cameras to get a stable position estimation of the
    // target (aruco)
}

cv::Mat VisionTracker::preprocess_frame(const std::string& serial, const rs2::frameset& frame) {
    // convert the rgb frame from realsense to opencv format
    auto color = frame.get_color_frame();

    cv::Mat rgb_frame;
    auto it = m_cache_frames.find(serial);
    if (it != m_cache_frames.end()) {
        rgb_frame = it->second;
        // check if the size of the cache is same as the new frame, if not, update the cache
        if (rgb_frame.cols != color.get_width() || rgb_frame.rows != color.get_height()) {
            rgb_frame = cv::Mat(color.get_height(), color.get_width(), CV_8UC3, (void*)color.get_data()).clone();
            m_cache_frames[serial] = rgb_frame;
        } else {
            // update the data of the cache frame
            std::memcpy(rgb_frame.data, color.get_data(), color.get_width() * color.get_height() * 3);
        }
    } else {
        // does not exist in cache, create a new one and store it in cache
        rgb_frame = cv::Mat(color.get_height(), color.get_width(), CV_8UC3, (void*)color.get_data()).clone();
        m_cache_frames[serial] = rgb_frame;
    }

    return rgb_frame;
}
}  // namespace tracking
