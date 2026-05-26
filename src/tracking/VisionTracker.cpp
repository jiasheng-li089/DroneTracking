#include "VisionTracker.h"

#include <cstring>
#include <opencv2/core.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <qtmetamacros.h>
#include <spdlog/spdlog.h>

#include <QImage>

#include "TrackerConfig.h"
#include "common/common.h"
#include "config.h"
#include "spdlog/fmt/bundled/format.h"

namespace tracking {

void update_measurement_matrix(cv::Mat& measurement_matrix, float dt) {
  measurement_matrix.at<float>(0, 3) = dt;
  measurement_matrix.at<float>(1, 4) = dt;
  measurement_matrix.at<float>(2, 5) = dt;
}

VisionTracker::VisionTracker(std::shared_ptr<tracking::TrackerConfig> config,
                             QObject *parent)
    : m_config(config), QObject(parent) {
  // Initialize ArUco detectors based on the configuration
  try {
    m_marker_parameters = m_config->get_marker_parameters();
    auto detector_params = m_config->get_aruco_detector_parameters();
    auto dictionary = m_config->get_aruco_dictionary();

    m_aruco_detector = cv::aruco::ArucoDetector(dictionary, detector_params);

    auto cam_params_vec = m_config->get_camera_calibration_parameters();
    for (const auto &param : cam_params_vec) {
      m_camera_parameters[param.serial] = param;

      // initilize the computation data
      ComputationData data;
      data.kf.init(6, 3, 0, CV_32F);
      data.last_computed_time = current_timestamp_ms();

      // transition matrix (A)
      data.kf.transitionMatrix = (cv::Mat_<float>(6, 6) <<
        1, 0, 0, 1, 0, 0,
        0, 1, 0, 0, 1, 0,
        0, 0, 1, 0, 0, 1,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1);
      
      // measurement matrix (H)
      data.kf.measurementMatrix = cv::Mat::zeros(3, 6, CV_32F);
      data.kf.measurementMatrix.at<float>(0, 0) = 1.0f;
      data.kf.measurementMatrix.at<float>(1, 1) = 1.0f;
      data.kf.measurementMatrix.at<float>(2, 2) = 1.0f;

      // process noise covariance (Q)
      cv::setIdentity(data.kf.processNoiseCov, cv::Scalar::all(1e-4));

      // measurement noise covariance (R)
      cv::setIdentity(data.kf.measurementNoiseCov, cv::Scalar::all(1e-4));
      
      // error covariance (P)
      cv::setIdentity(data.kf.errorCovPost, cv::Scalar::all(1));

      m_computation_data[param.serial] = data;
    }

    m_benchmark_parameter =
        std::make_unique<MarkerParameter>(m_config->get_benchmark_parameter());
  } catch (const std::exception &ex) {
    spdlog::error("Error initializing VisionTracker: {}", ex.what());
    throw; // Rethrow to indicate initialization failure
  }
}

VisionTracker::~VisionTracker() = default;

cv::Vec3d get_translation_from_pose(const cv::Mat &pose) {
  return cv::Vec3d(pose.at<double>(0, 3), pose.at<double>(1, 3),
                   pose.at<double>(2, 3));
}

/**
 * return the rotation from the pose in degrees (roll, pitch, yaw)
 */
cv::Vec3d get_rotation_from_pose_in_degrees(const cv::Mat &pose) {
  // 1. Extract the 3x3 rotation matrix R
  cv::Mat R = pose(cv::Rect(0, 0, 3, 3));
  R.convertTo(R, CV_64F); // Ensure double precision for trig calculations

  // 2. Prevent Gimbal Lock by checking the Y-axis pitch magnitude
  double sy = std::hypot(R.at<double>(0, 0), R.at<double>(1, 0));
  bool singular = sy < 1e-6;

  double roll, pitch, yaw;

  // 3. Extract angles directly from the Matrix R (Not a Rodrigues vector!)
  if (!singular) {
    roll = std::atan2(R.at<double>(2, 1), R.at<double>(2, 2));
    pitch = std::atan2(-R.at<double>(2, 0), sy);
    yaw = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0));
  } else {
    // Gimbal lock fallback
    roll = std::atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
    pitch = std::atan2(-R.at<double>(2, 0), sy);
    yaw = 0;
  }

  // 4. Convert to degrees and return
  return cv::Vec3d(roll * 180.0 / CV_PI, pitch * 180.0 / CV_PI,
                   yaw * 180.0 / CV_PI);
}

bool VisionTracker::calibrate_camera(bool log, CameraParameters &cam_params,
                                     std::vector<int> &marker_ids,
                                     std::vector<cv::Vec3d> &rvecs,
                                     std::vector<cv::Vec3d> &tvecs) {
  // implement the camera calibration logic using the benchmark marker
  auto it = std::find(marker_ids.begin(), marker_ids.end(),
                      m_benchmark_parameter->id);
  if (it == marker_ids.end()) {
    spdlog::debug("Benchmark marker with ID {} not detected, cannot calibrate "
                  "camera with serial: {}",
                  m_benchmark_parameter->id, cam_params.serial);
    return false;
  }
  auto index = std::distance(marker_ids.begin(), it);

  auto rvec = rvecs[index];
  auto tvec = tvecs[index];

  // The pose of the benchmark marker relative to the camera
  cv::Mat benchmark_cam_pose = cv::Mat::eye(4, 4, CV_64F);
  cv::Rodrigues(rvec, benchmark_cam_pose(cv::Rect(0, 0, 3, 3)));
  cv::Mat(tvec).copyTo(benchmark_cam_pose(cv::Rect(3, 0, 1, 3)));

  // Invert to get camera pose in benchmark marker frame
  cam_params.pose = benchmark_cam_pose.inv(); 

  if (log) {
    // extract rotation and translation from the camera pose
    cv::Vec3d t = get_translation_from_pose(cam_params.pose);
    cv::Vec3d r = get_rotation_from_pose_in_degrees(cam_params.pose);
    spdlog::info("Camera {} orientation in world (deg): yaw = {:.2f} pitch = "
                 "{:.2f} roll = {:.2f}",
                 cam_params.serial, r[2], r[1], r[0]);
  }

  // emit update_camera_status(
  //     fmt::format("Camera #{}", cam_params.serial),
  //     fmt::format("Calibrated: x = {:.2f}, y = {:.2f}, z = {:.2f}, "
  //                 "yaw = {:.2f}, pitch = {:.2f}, roll = {:.2f}",
  //                 t[0], t[1], t[2], r[2], r[1], r[0]));

  spdlog::info("Camera {} calibrated from benchmark marker {}",
               cam_params.serial, marker_ids[index]);
  return true;
}

void VisionTracker::process_frames(const int camera_id,
                                   const std::string &serial,
                                   const rs2::frameset &frames) {
  ComputationData &data = m_computation_data[serial];

  auto last_time = data.last_frame_time;
  auto current_timestamp = current_timestamp_ms();
  data.last_frame_time = current_timestamp;

  auto log_enable = frames.get_frame_number() % 30 == 1;

  if (log_enable && last_time > 0) {
    spdlog::debug(
        "Received frame from camera ({}), frame number: {}, fps: {:.2f}",
        serial, frames.get_frame_number(),
        1000.0 / (data.last_frame_time - last_time));
  }

  // check if the camera has detected the benchmark marker and calculate its
  // position and orientation in the world frame yet, if not, calibrate it first
  CameraParameters &cam_params = m_camera_parameters[serial];

  if (cam_params.K.empty() || cam_params.D.empty()) {
    if (log_enable)
      spdlog::warn("Camera parameters not found for serial: {}", serial);
    return;
  }

  // obtain the position of the camera related to the benchmark marker if have
  // not yet.
  auto rgb_frame = preprocess_frame(serial, data, frames);
  // detect the possible position of the target (aruco) related to the camera.
  std::vector<int> marker_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;

  m_aruco_detector.detectMarkers(rgb_frame, marker_corners, marker_ids,
                                 rejected_candidates);

  spdlog::debug("detected {} markers from cameraId ({}): [{}]",
                marker_corners.size(), serial, fmt::join(marker_ids, ", "));

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
      if (log_enable)
        spdlog::debug("Unknown marker detected with ID: {} from cameraId ({})",
                      id, serial);
    }
  }

  auto size = known_marker_corners.size();
  if (log_enable)
    spdlog::debug("Processing {} known markers from cameraId ({}), IDs: [{}]",
                  size, serial, fmt::join(known_marker_ids, ", "));

  // compute the rvec and tvec for each known marker
  std::vector<cv::Vec3d> rvecs(size), tvecs(size);
  for (size_t i = 0; i < size; ++i) {
    const auto &marker_param = known_marker_parameters[i];
    const auto &marker_corner = known_marker_corners[i];
#define USE_GENERIC_SOLVE_PNP 1
#ifdef USE_GENERIC_SOLVE_PNP
    std::vector<cv::Mat> rvecs_out, tvecs_out;
    std::vector<double> reproj_errors;
    cv::solvePnPGeneric(marker_param.obj_points, marker_corner, cam_params.K,
                        cam_params.D, rvecs_out, tvecs_out, false,
                        cv::SOLVEPNP_IPPE_SQUARE, cv::noArray(), cv::noArray(),
                        reproj_errors);

    // figure out the best solution based on the reprojection error, and use it
    // for the following processing.
    auto min_it = std::min_element(reproj_errors.begin(), reproj_errors.end());
    int best_index = std::distance(reproj_errors.begin(), min_it);
    rvecs.at(i) = rvecs_out.at(best_index);
    tvecs.at(i) = tvecs_out.at(best_index);

    spdlog::debug(
        "Marker ID: {}, reprojection errors for different solutions: [{}], "
        "selected solution index: {}, "
        "rvec: [{:.2f}, {:.2f}, {:.2f}], tvec: [{:.2f}, {:.2f}, {:.2f}]",
        known_marker_ids[i], fmt::join(reproj_errors, ", "), best_index,
        rvecs.at(i)[0], rvecs.at(i)[1], rvecs.at(i)[2], tvecs.at(i)[0],
        tvecs.at(i)[1], tvecs.at(i)[2]);
#else
    // solvePnP to get the relative position of the target (aruco) to the camera
    cv::solvePnP(marker_param.obj_points, marker_corner, cam_params.K,
                 cam_params.D, rvecs.at(i), tvecs.at(i), false,
                 cv::SOLVEPNP_IPPE_SQUARE);
#endif
  }

  // show detected markers on the image
  std::vector<cv::Point2f> all_centers_2d(size);
  // generate a new image and mark the detected markers on the image, then emit
  // the signal to update the GUI
  cv::Mat output_image = rgb_frame.clone();
  for (size_t i = 0; i < size; ++i) {
    std::vector<cv::Point3f> center_3d = {cv::Point3f(0, 0, 0)};
    std::vector<cv::Point2f> center_2d;
    cv::projectPoints(center_3d, rvecs.at(i), tvecs.at(i), cam_params.K,
                      cam_params.D, center_2d);
    all_centers_2d[i] = center_2d.at(0);
    cv::circle(output_image, center_2d.at(0), 5, cv::Scalar(0, 255, 0), -1);

    cv::aruco::drawDetectedMarkers(
        output_image,
        std::vector<std::vector<cv::Point2f>>{known_marker_corners[i]},
        std::vector<int>{known_marker_ids[i]});
  }
  QImage qimg(output_image.data, output_image.cols, output_image.rows,
              static_cast<int>(output_image.step), QImage::Format_RGB888);
  emit frames_received(std::vector<std::tuple<int, std::string, QImage>>{
      {camera_id + 200, serial, qimg.copy()}});

  if (known_marker_corners.empty() ||
      (known_marker_corners.size() == 1 &&
       known_marker_ids[0] == m_benchmark_parameter->id)) {
    // notify the GUI that current tracking status is "Lost"
    if (log_enable)
    spdlog::debug("No known markers detected from cameraId ({}), skipping "
                  "pose estimation",
                  serial);
    return;
  }

  // only start tracking when the camera position is known.
  if (!cam_params.calibrated) {
    emit update_camera_status(fmt::format("Camera #{}", cam_params.serial),
                              "Calibrating");

    if (!calibrate_camera(log_enable, cam_params, known_marker_ids, rvecs, tvecs)) {
      return;
    }
#ifdef CALIBRATE_ONCE
    cam_params.calibrated = true;
#endif
  }

  if (log_enable)
    spdlog::debug(
        "Pose estimation completed for cameraId ({}), marker IDs: [{}]", serial,
        fmt::join(known_marker_ids, ", "));
  for (size_t i = 0; i < size; i++) {
    cv::Mat marker_camera_pose = cv::Mat::eye(4, 4, CV_64F);
    cv::Rodrigues(rvecs[i], marker_camera_pose(cv::Rect(0, 0, 3, 3)));
    cv::Mat(tvecs[i]).copyTo(marker_camera_pose(cv::Rect(3, 0, 1, 3)));
    auto position = get_translation_from_pose(marker_camera_pose);
    auto rotation = get_rotation_from_pose_in_degrees(marker_camera_pose);
    if (log_enable)
      spdlog::debug("THIS ONE: Marker ID: {}, rvec: [{:.2f}, {:.2f}, {:.2f}], "
                    "tvec: [{:.2f}, {:.2f}, {:.2f}]",
                    known_marker_ids[i], rotation[0], rotation[1], rotation[2],
                    position[0], position[1], position[2]);
  }

  std::vector<cv::Mat> drone_world_poses;

  // transform each marker pose from camera frame to world (benchmark marker)
  // frame cam_params->R and cam_params->T are calibrated per-camera, so apply
  // once for all cameras
  for (size_t i = 0; i < size; ++i) {

    cv::Vec3d rvec = rvecs.at(i);
    cv::Vec3d tvec = tvecs.at(i);

    cv::Mat marker_camera_pose = cv::Mat::eye(4, 4, CV_64F);
    cv::Rodrigues(rvec, marker_camera_pose(cv::Rect(0, 0, 3, 3)));
    cv::Mat(tvec).copyTo(marker_camera_pose(cv::Rect(3, 0, 1, 3)));

    auto drone_marker_pose = known_marker_parameters[i].drone_marker_pose;

    // beside calculating the marker pose, some offset must be applied to the
    // marker pose to get the drone body pose. INFO: in theory, to get the pose
    // of the virtual marker, some offset should be applied to the marker pose.
    // However, the calculated orientation of the virtual marker is not
    // reliable, applying this offset may amplify the drift. Therefore, treat
    // the translation of the marker as the drone's translation directly.
    // Averaging multiple marker translations can help reduce the drift.
    cv::Mat drone_world_pose = cam_params.pose * marker_camera_pose;

    if (known_marker_ids[i] != m_benchmark_parameter->id) {

      // while the benchmark marker IS ALWAYS placed horizontally on the ground,
      // some offset around z axis can be applied to make the translation closer
      // to the drone's actual position.
      auto z_offset = drone_marker_pose.at<double>(3, 2);
      drone_world_pose.at<double>(3, 2) += z_offset;
      // skip the benchmark marker when calculating the drone pose,
      // since the benchmark marker is not attached to the drone and its
      // position is fixed in the world.
      drone_world_poses.push_back(drone_world_pose);
    }

#ifdef SHOW_SINGLE_MARKER_POSE
    // extract rotation and translation from the world pose
    cv::Vec3d t = get_translation_from_pose(drone_world_pose);
    cv::Vec3d r = get_rotation_from_pose_in_degrees(drone_world_pose);

    ObjectPose pose = ObjectPose::from_t_and_r(t, r);
    if (known_marker_ids[i] != m_benchmark_parameter->id)
      emit update_camera_status(
          fmt::format("Marker #{}_{}", serial, known_marker_ids[i]),
          fmt::format("x = {:.2f}, y = {:.2f}, z = {:.2f}, "
                      "yaw = {:.2f}, pitch = {:.2f}, roll = {:.2f}",
                      pose.x, pose.y, pose.z, pose.yaw, pose.pitch, pose.roll));

    // if (log_enable) {
    double distance = std::sqrt(t[0] * t[0] + t[1] * t[1] + t[2] * t[2]);
    spdlog::info(
        "Marker ID: {}, camera: {}, pos (m): [{:.2f}, {:.2f}, {:.2f}], "
        "dist: {:.3f} m, rot (deg) yaw = {:.2f} pitch = {:.2f} roll = {:.2f}",
        known_marker_ids[i], serial, pose.x, pose.y, pose.z, distance, pose.yaw,
        pose.pitch, pose.roll);

#ifdef ENABLE_DEPTH_CAMERA
    // detect the depth from the depth frame at the center of the marker and log
    // it for debugging
    auto depth_frame = frames.get_depth_frame();
    if (depth_frame) {
      float depth_value =
          depth_frame.get_distance(static_cast<int>(all_centers_2d[i].x),
                                   static_cast<int>(all_centers_2d[i].y));
      spdlog::info("Depth value at marker center: {:.2f} m", depth_value);
    }
#endif
    // }
#endif
  }

  if (drone_world_poses.size() <= 0) {
    // all computed poses are invalid, stop sending pose to the controller
    spdlog::warn("All computed poses are invalid, stop sending pose to the "
                 "controller");
    return;
  }

  cv::Vec3d avg_translation(0, 0, 0);
  cv::Vec3d avg_rotation(0, 0, 0);
  for (const auto &pose : drone_world_poses) {
    avg_translation += get_translation_from_pose(pose);
    avg_rotation += get_rotation_from_pose_in_degrees(pose);
  }
  avg_translation /= static_cast<int>(drone_world_poses.size());
  avg_rotation /= static_cast<int>(drone_world_poses.size());

  // TODO need to apply kalman filter to the computed translation
  float dt = (current_timestamp - data.last_computed_time) / 1000.0f;
  update_measurement_matrix(data.kf.measurementMatrix, dt);

  cv::Mat prediction = data.kf.predict();
  cv::Mat estimation = data.kf.correct((cv::Mat_<float>(3, 1) <<
    (float) avg_translation[0],
    (float) avg_translation[1],
    (float) avg_translation[2]
  ));

  cv::Vec3d smoothed_translation(
    estimation.at<float>(0, 0),
    estimation.at<float>(1, 0),
    estimation.at<float>(2, 0)
  );
  data.computed_pose = ObjectPose::from_t_and_r(smoothed_translation, avg_rotation);
  data.last_computed_time = current_timestamp;
  emit update_camera_status(
    fmt::format("Drone_{}", serial),
    fmt::format("x = {:.2f}, y = {:.2f}, z = {:.2f}", data.computed_pose.x, data.computed_pose.y, data.computed_pose.z)
  );

  // only read operation for m_computation_data, not need to lock the data
  cv::Vec3d t_sum(0, 0, 0);
  cv::Vec3d r_sum(0, 0, 0);
  double roll_sum = 0, pitch_sum = 0, yaw_sum = 0;
  double n = static_cast<double>(m_computation_data.size());

  for (auto &[cam_serial, tmp_data] : m_computation_data) {
    ObjectPose &pose = tmp_data.computed_pose;
    // the cached pose is too old, skip it
    if (current_timestamp - pose.timestamp > 100) {
      n--;
      continue;
    }
    t_sum += cv::Vec3d(pose.x, pose.y, pose.z);
    r_sum += cv::Vec3d(pose.roll, pose.pitch, pose.yaw);
  }

  cv::Vec3d t_avg = t_sum / n;
  cv::Vec3d r_avg = r_sum / n;

  ObjectPose final_pose = ObjectPose::from_t_and_r(t_avg, r_avg);
  emit publish_message(final_pose.to_json());
}

cv::Mat VisionTracker::preprocess_frame(const std::string &serial,
                                        ComputationData &data,
                                        const rs2::frameset &frame) {
  // convert the rgb frame from realsense to opencv format
  auto color = frame.get_color_frame();

  if (data.cache_frame.cols != color.get_width() ||
      data.cache_frame.rows != color.get_height()) {
    data.cache_frame = cv::Mat(color.get_height(), color.get_width(), CV_8UC3,
                               (void *)color.get_data())
                           .clone();
  } else {
    std::memcpy(data.cache_frame.data, color.get_data(),
                color.get_width() * color.get_height() * 3);
  }
  return data.cache_frame;
}
} // namespace tracking
