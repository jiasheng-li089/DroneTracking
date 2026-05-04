#include "VisionTracker.h"

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

    // ignore all unknown markers for now, but log them for debugging
    std::vector<int> known_marker_ids;
    std::vector<std::vector<cv::Point2f>> known_marker_corners;
    std::vector<MarkerParameter> known_marker_parameters;
    for (size_t i = 0; i < marker_ids.size(); ++i) {
        int id = marker_ids[i];
        auto param_it = m_marker_parameters.find(id);
        if (param_it != m_marker_parameters.end()) {
            known_marker_corners.push_back(marker_corners[i]);
            known_marker_parameters.push_back(param_it->second);
            known_marker_ids.push_back(id);
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
                      fmt::join(known_marker_ids, ", "));

    std::vector<cv::Vec3d> rvecs(size), tvecs(size);

    for (size_t i = 0; i < size; ++i) {
        const auto& marker_param = known_marker_parameters[i];
        const auto& marker_corner = known_marker_corners[i];

        // solvePnP to get the relative position of the target (aruco) to the camera
        cv::solvePnP(marker_param.obj_points, marker_corner, cam_params.K, cam_params.D, rvecs.at(i), tvecs.at(i));
    }

    if (log_enable)
        spdlog::debug("Pose estimation completed for cameraId ({}), marker IDs: [{}]", serial,
                      fmt::join(known_marker_ids, ", "));
    for (size_t i = 0; i < size; i++) {
        if (log_enable)
            spdlog::debug("Marker ID: {}, rvec: [{:.2f}, {:.2f}, {:.2f}], tvec: [{:.2f}, {:.2f}, {:.2f}]",
                          known_marker_ids[i], rvecs[i][0], rvecs[i][1], rvecs[i][2], tvecs[i][0], tvecs[i][1], tvecs[i][2]);
    }

    std::vector<cv::Point2f> all_centers_2d(size);

    // generate a new image and mark the detected markers on the image, then emit the signal to update the GUI
    cv::Mat output_image = rgb_frame.clone();
    for (size_t i = 0; i < size; ++i) {
        std::vector<cv::Point3f> center_3d = {cv::Point3f(0, 0, 0)};
        std::vector<cv::Point2f> center_2d;
        cv::projectPoints(center_3d, rvecs.at(i), tvecs.at(i), cam_params.K, cam_params.D, center_2d);
        all_centers_2d[i] = center_2d.at(0);
        cv::circle(output_image, center_2d.at(0), 5, cv::Scalar(0, 255, 0), -1);

        cv::aruco::drawDetectedMarkers(output_image, std::vector<std::vector<cv::Point2f>>{known_marker_corners[i]},
                                       std::vector<int>{known_marker_ids[i]});
    }
    QImage qimg(output_image.data, output_image.cols, output_image.rows, static_cast<int>(output_image.step),
                QImage::Format_RGB888);
    emit frames_received(std::vector<std::tuple<int, std::string, QImage>>{{camera_id + 200, serial, qimg.copy()}});

    // there might be several markers detected from the same camera,
    // for now, only use the first one to calculate the position and orientation of the target (aruco) in world frame,

    std::vector<cv::Vec3d> new_rvecs(size), new_tvecs(size);
    if (serial == m_camera_parameters[0].serial) {
        // current camera is the benckmark camera, which is used as the world frame reference, so the relative position
        // is the same as the world position
        new_rvecs = rvecs;
        new_tvecs = tvecs;
    } else {
        // current camera is not the benchmark camera, need to transform the pose to the one relative to the benchmark
        // camera using the R and T between the two cameras
        for (size_t i = 0; i < size; ++i) {
            // cam_params.R, cam_params.T: direct relative transform to the other camera
            // P_other = R * P_current + T
            cv::Mat p_other = cam_params.R * cv::Mat(tvecs.at(i)) + cam_params.T;
            new_tvecs.at(i) = cv::Vec3d(p_other);

            cv::Mat R_marker_current;
            cv::Rodrigues(rvecs.at(i), R_marker_current);
            cv::Rodrigues(cam_params.R * R_marker_current, new_rvecs.at(i));
        }
    }

    // compute marker pose relative to the camera and log
    for (size_t i = 0; i < size; ++i) {
        const auto& tvec = new_tvecs.at(i);

        cv::Mat R_marker;
        cv::Rodrigues(new_rvecs.at(i), R_marker);

        // ZYX Euler angles (yaw-pitch-roll) in degrees
        double yaw = std::atan2(R_marker.at<double>(1, 0), R_marker.at<double>(0, 0)) * 180.0 / CV_PI;
        double pitch =
            std::atan2(-R_marker.at<double>(2, 0), std::hypot(R_marker.at<double>(2, 1), R_marker.at<double>(2, 2))) *
            180.0 / CV_PI;
        double roll = std::atan2(R_marker.at<double>(2, 1), R_marker.at<double>(2, 2)) * 180.0 / CV_PI;

        ObjectPose pose{tvec[0], tvec[1], tvec[2], roll, pitch, yaw};

        if (i == 0)  // Only publish the first detected marker's pose for simplicity, can be extended to multiple
                     // markers if needed
            emit publish_message(pose.to_json());

        if (log_enable) {
            double distance = std::sqrt(tvec[0]*tvec[0] + tvec[1]*tvec[1] + tvec[2]*tvec[2]);
            spdlog::info(
                "Marker ID: {}, camera: {}, pos (m): [{:.3f}, {:.3f}, {:.3f}], "
                "dist: {:.3f} m, rot (deg) yaw={:.1f} pitch={:.1f} roll={:.1f}",
                known_marker_ids[i], serial, pose.x, pose.y, pose.z, distance, pose.yaw, pose.pitch, pose.roll);

            // detect the depth from the depth frame at the center of the marker and log it for debugging
            auto depth_frame = frames.get_depth_frame();
            if (depth_frame) {
                float depth_value = depth_frame.get_distance(static_cast<int>(all_centers_2d[i].x), static_cast<int>(all_centers_2d[i].y));
                spdlog::info("Depth value at marker center: {:.3f} m", depth_value);    
            }
        }
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
