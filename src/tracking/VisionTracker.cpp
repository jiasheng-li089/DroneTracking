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

        m_benchmark_parameter = std::make_unique<MarkerParameter>(m_config->get_benchmark_parameter());
    } catch (const std::exception& ex) {
        spdlog::error("Error initializing VisionTracker: {}", ex.what());
        throw;  // Rethrow to indicate initialization failure
    }
}

VisionTracker::~VisionTracker() = default;

bool VisionTracker::calibrate_camera(CameraParameters& cam_params, std::vector<int>& marker_ids,
                                     std::vector<std::vector<cv::Point2f>>& marker_corners) {
    // TODO implement the camera calibration logic using the benchmark marker

    std::vector<cv::Point2f> benchmark_corners;

    auto it = std::find(marker_ids.begin(), marker_ids.end(), m_benchmark_parameter->id);
    if (it == marker_ids.end()) {
        spdlog::debug("Benchmark marker with ID {} not detected, cannot calibrate camera with serial: {}",
                      m_benchmark_parameter->id, cam_params.serial);
        return false;
    }
    benchmark_corners = marker_corners[std::distance(marker_ids.begin(), it)];

    cv::Vec3d rvec, tvec;
    cv::solvePnP(m_benchmark_parameter->obj_points, benchmark_corners, cam_params.K, cam_params.D, rvec, tvec);

    // solvePnP gives marker→camera transform: P_cam = R * P_marker + t
    // Invert to get camera→world (marker frame): P_world = R^T * P_cam - R^T * t
    cv::Mat R_mat;
    cv::Rodrigues(rvec, R_mat);
    cam_params.R = R_mat.t();
    cam_params.T = -cam_params.R * cv::Mat(tvec);

    double yaw = std::atan2(cam_params.R.at<double>(1, 0), cam_params.R.at<double>(0, 0)) * 180.0 / CV_PI;
    double pitch = std::atan2(-cam_params.R.at<double>(2, 0),
                              std::hypot(cam_params.R.at<double>(2, 1), cam_params.R.at<double>(2, 2))) *
                   180.0 / CV_PI;
    double roll = std::atan2(cam_params.R.at<double>(2, 1), cam_params.R.at<double>(2, 2)) * 180.0 / CV_PI;

    spdlog::info("Camera {} orientation in world (deg): yaw={:.4f} pitch={:.4f} roll={:.4f}", cam_params.serial, yaw,
                 pitch, roll);

    emit update_camera_status(fmt::format("Camera #{}", cam_params.serial),
                              fmt::format("Calibrated: x={:.4f}, y={:.4f}, z={:.4f}, "
                                          "yaw={:.4f}, pitch={:.4f}, roll={:.4f}",
                                          cam_params.T.at<double>(0), cam_params.T.at<double>(1),
                                          cam_params.T.at<double>(2), yaw, pitch, roll));

    spdlog::info("Camera {} calibrated from benchmark marker {}", cam_params.serial, m_benchmark_parameter->id);
    return true;
}

void VisionTracker::process_frames(const int camera_id, const std::string& serial, const rs2::frameset& frames) {
    auto log_enable = frames.get_frame_number() % 30 == 0;

    // check if the camera has detected the benchmark marker and calculate its position and orientation in the world
    // frame yet, if not, calibrate it first
    CameraParameters* cam_params = nullptr;

    for (auto& param : m_camera_parameters) {
        if (param.serial == serial) {
            cam_params = &param;
            break;
        }
    }

    if (!cam_params || cam_params->K.empty() || cam_params->D.empty()) {
        if (log_enable) spdlog::warn("Camera parameters not found for serial: {}", serial);
        return;
    }

    // obtain the position of the camera related to the benchmark marker if have not yet.
    auto rgb_frame = preprocess_frame(serial, frames);
    // detect the possible position of the target (aruco) related to the camera.
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;

    m_aruco_detector.detectMarkers(rgb_frame, marker_corners, marker_ids, rejected_candidates);

    if (!cam_params->calibrated) {
        emit update_camera_status(fmt::format("Camera #{}", cam_params->serial), "Calibrating");

        if (!calibrate_camera(*cam_params, marker_ids, marker_corners)) {
            return;
        }
        cam_params->calibrated = true;
    }

    // only start tracking when the camera position is known.

    if (log_enable)
        spdlog::debug("detected {} markers from cameraId ({}): [{}]", marker_corners.size(), serial,
                      fmt::join(marker_ids, ", "));

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
        } else if (id == m_benchmark_parameter->id) {
            known_marker_corners.push_back(marker_corners[i]);
            known_marker_parameters.push_back(*m_benchmark_parameter);
            known_marker_ids.push_back(id);
        } else {
            if (log_enable) spdlog::debug("Unknown marker detected with ID: {} from cameraId ({})", id, serial);
        }
    }
    if (known_marker_corners.empty()) {
        // TODO notify the GUI that current tracking status is "Lost"
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
        cv::solvePnP(marker_param.obj_points, marker_corner, cam_params->K, cam_params->D, rvecs.at(i), tvecs.at(i));
    }

    if (log_enable)
        spdlog::debug("Pose estimation completed for cameraId ({}), marker IDs: [{}]", serial,
                      fmt::join(known_marker_ids, ", "));
    for (size_t i = 0; i < size; i++) {
        if (log_enable)
            spdlog::debug("Marker ID: {}, rvec: [{:.2f}, {:.2f}, {:.2f}], tvec: [{:.2f}, {:.2f}, {:.2f}]",
                          known_marker_ids[i], rvecs[i][0], rvecs[i][1], rvecs[i][2], tvecs[i][0], tvecs[i][1],
                          tvecs[i][2]);
    }

    std::vector<cv::Point2f> all_centers_2d(size);

    // generate a new image and mark the detected markers on the image, then emit the signal to update the GUI
    cv::Mat output_image = rgb_frame.clone();
    for (size_t i = 0; i < size; ++i) {
        std::vector<cv::Point3f> center_3d = {cv::Point3f(0, 0, 0)};
        std::vector<cv::Point2f> center_2d;
        cv::projectPoints(center_3d, rvecs.at(i), tvecs.at(i), cam_params->K, cam_params->D, center_2d);
        all_centers_2d[i] = center_2d.at(0);
        cv::circle(output_image, center_2d.at(0), 5, cv::Scalar(0, 255, 0), -1);

        cv::aruco::drawDetectedMarkers(output_image, std::vector<std::vector<cv::Point2f>>{known_marker_corners[i]},
                                       std::vector<int>{known_marker_ids[i]});
    }
    QImage qimg(output_image.data, output_image.cols, output_image.rows, static_cast<int>(output_image.step),
                QImage::Format_RGB888);
    emit frames_received(std::vector<std::tuple<int, std::string, QImage>>{{camera_id + 200, serial, qimg.copy()}});

    std::vector<cv::Mat> world_R_matrices;
    std::vector<cv::Vec3d> world_tvecs;

    // transform each marker pose from camera frame to world (benchmark marker) frame
    // cam_params->R and cam_params->T are calibrated per-camera, so apply once for all cameras
    for (size_t i = 0; i < size; ++i) {
        cv::Mat tvec_world_mat = cam_params->R * cv::Mat(tvecs.at(i)) + cam_params->T;
        cv::Vec3d tvec_world(tvec_world_mat);

        cv::Mat R_marker_cam;
        cv::Rodrigues(rvecs.at(i), R_marker_cam);
        cv::Mat R_marker_world = cam_params->R * R_marker_cam;

        // ZYX Euler angles (yaw-pitch-roll) in degrees
        double yaw = std::atan2(R_marker_world.at<double>(1, 0), R_marker_world.at<double>(0, 0)) * 180.0 / CV_PI;
        double pitch = std::atan2(-R_marker_world.at<double>(2, 0),
                                  std::hypot(R_marker_world.at<double>(2, 1), R_marker_world.at<double>(2, 2))) *
                       180.0 / CV_PI;
        double roll = std::atan2(R_marker_world.at<double>(2, 1), R_marker_world.at<double>(2, 2)) * 180.0 / CV_PI;

        if (known_marker_ids[i] != m_benchmark_parameter->id) {
            world_R_matrices.push_back(R_marker_world.clone());
            world_tvecs.push_back(tvec_world);
        }

        ObjectPose pose{tvec_world[0], tvec_world[1], tvec_world[2], roll, pitch, yaw};
        emit update_camera_status(fmt::format("Marker #{}_{}", serial, known_marker_ids[i]),
                                  fmt::format("x={:.4f}, y={:.4f}, z={:.4f}, "
                                              "yaw={:.4f}, pitch={:.4f}, roll={:.4f}",
                                              pose.x, pose.y, pose.z, pose.yaw, pose.pitch, pose.roll));

        if (log_enable) {
            double distance = std::sqrt(tvec_world[0] * tvec_world[0] + tvec_world[1] * tvec_world[1] +
                                        tvec_world[2] * tvec_world[2]);
            spdlog::info(
                "Marker ID: {}, camera: {}, pos (m): [{:.4f}, {:.4f}, {:.4f}], "
                "dist: {:.3f} m, rot (deg) yaw={:.4f} pitch={:.4f} roll={:.4f}",
                known_marker_ids[i], serial, pose.x, pose.y, pose.z, distance, pose.yaw, pose.pitch, pose.roll);

#ifdef ENABLE_DEPTH_CAMERA
            // detect the depth from the depth frame at the center of the marker and log it for debugging
            auto depth_frame = frames.get_depth_frame();
            if (depth_frame) {
                float depth_value = depth_frame.get_distance(static_cast<int>(all_centers_2d[i].x),
                                                             static_cast<int>(all_centers_2d[i].y));
                spdlog::info("Depth value at marker center: {:.4f} m", depth_value);
            }
#endif
        }
    }

    // Average marker poses to compute drone body pose.
    // Position: arithmetic mean of all world tvecs.
    // Rotation: undo each marker's known body-relative angle (around Z), sum the resulting
    //           rotation matrices, then re-orthogonalize via SVD (geodesic mean on SO(3)).
    cv::Vec3d t_drone_sum(0, 0, 0);
    cv::Mat R_drone_sum = cv::Mat::zeros(3, 3, CV_64F);

    for (size_t i = 0; i < world_R_matrices.size(); ++i) {
        t_drone_sum += world_tvecs[i];

        // undo the marker's angular offset relative to drone body (rotation around Z)
        double angle_rad = -known_marker_parameters[i].angle * CV_PI / 180.0;
        double c = std::cos(angle_rad), s = std::sin(angle_rad);
        cv::Mat Rz = (cv::Mat_<double>(3, 3) << c, -s, 0, s, c, 0, 0, 0, 1);
        R_drone_sum += world_R_matrices[i] * Rz;
    }

    double n = static_cast<double>(world_R_matrices.size());
    cv::Vec3d t_drone_avg = t_drone_sum / n;

    // re-orthogonalize the summed rotation matrix to the nearest proper rotation
    cv::Mat U, S, Vt;
    cv::SVD::compute(R_drone_sum, S, U, Vt);
    cv::Mat R_drone_avg = U * Vt;
    if (cv::determinant(R_drone_avg) < 0) R_drone_avg = -R_drone_avg;

    double avg_yaw   = std::atan2(R_drone_avg.at<double>(1, 0), R_drone_avg.at<double>(0, 0)) * 180.0 / CV_PI;
    double avg_pitch = std::atan2(-R_drone_avg.at<double>(2, 0),
                                   std::hypot(R_drone_avg.at<double>(2, 1), R_drone_avg.at<double>(2, 2))) * 180.0 / CV_PI;
    double avg_roll  = std::atan2(R_drone_avg.at<double>(2, 1), R_drone_avg.at<double>(2, 2)) * 180.0 / CV_PI;

    ObjectPose averaged_pose{t_drone_avg[0], t_drone_avg[1], t_drone_avg[2], avg_roll, avg_pitch, avg_yaw};
    emit update_camera_status(fmt::format("Drone Pose #{}", serial), fmt::format("x={:.4f}, y={:.4f}, z={:.4f}, yaw={:.4f}, pitch={:.4f}, roll={:.4f}",
                                  averaged_pose.x, averaged_pose.y, averaged_pose.z,
                                  averaged_pose.yaw, averaged_pose.pitch, averaged_pose.roll));

    // publish the stabilized drone pose to the network
    emit publish_message(averaged_pose.to_json());
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
