#include "GenericCamerManager.h"

#include <QObject>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <spdlog/spdlog.h>
#include <thread>
#include <vector>

#include "camera_util.h"
#include "config.h"

namespace CameraManager {

GenericCameraManager::GenericCameraManager(QObject *parent)
    : CameraManager(parent) {}

void GenericCameraManager::start_cameras() {
  if (m_running)
    return;
  spdlog::info("Starting GenericCameraManager...");
  m_running = true;
  std::vector<CameraMeta> camera_metas = get_all_camera_metas();
  for (const auto &meta : camera_metas) {
    auto serial = meta.serial;
    std::transform(serial.begin(), serial.end(), serial.begin(), ::tolower);
    auto non_zero_index = serial.find_first_not_of('0');
    if (non_zero_index != std::string::npos) {
      serial = serial.substr(non_zero_index);
    }

    if (meta.supported_resolutions.size() <= 0 || meta.supported_resolutions[0].fps.size() <= 0) {
      spdlog::warn("Camera (id: {}, serial: {}) has no supported resolutions or fps, skipping...", meta.id, serial);
      continue;
    }
    int width = meta.supported_resolutions[0].width;
    int height = meta.supported_resolutions[0].height;
    int fps = static_cast<int>(meta.supported_resolutions[0].fps[0]);
    spdlog::info("Start thread for camera (id: {}, serial: {}: using resolution {}x{} at {} fps", meta.id, serial, width, height, fps);
    std::unique_ptr<std::thread> thread = std::make_unique<std::thread>(
        &GenericCameraManager::camera_worker_thread, this, meta.id,
        serial, width, height, fps);
    m_threads.insert_or_assign(serial, std::move(thread));
  }
}

void GenericCameraManager::stop_cameras() {
  if (!m_running)
    return;

  spdlog::info("Stopping GenericCameraManager...");
  m_running = false;
  for (auto &thread : m_threads) {
    if (thread.second && thread.second->joinable()) {
      thread.second->join();
    }
  }
  m_threads.clear();
}

void GenericCameraManager::camera_worker_thread(
    const int camera_id, const std::string camera_serial, int width, int height, int fps) {
  spdlog::info("Starting worker thread for camera {}, serial: {}", camera_id,
               camera_serial);

  cv::VideoCapture cap(camera_id);

  if (!cap.isOpened()) {
    spdlog::error("Failed to open camera {}", camera_id);
    emit error_occurred(
        QString::fromStdString("Failed to open camera #{}").arg(camera_id));
    return;
  }

  cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
  cap.set(cv::CAP_PROP_FPS, fps);

  spdlog::info("Camera {} properties: width={}, height={}, fps={}", camera_id,
               cap.get(cv::CAP_PROP_FRAME_WIDTH),
               cap.get(cv::CAP_PROP_FRAME_HEIGHT), cap.get(cv::CAP_PROP_FPS));

  ulong frame_id = 0;
  cv::Mat frame, bgr_frame;
  while (m_running) {
    cap >> frame;
    frame_id++;
    if (frame.empty()) {
      spdlog::error("Failed to capture frame from camera {} for frame id: #{}",
                    camera_id, frame_id);
      break;
    }
    if (m_callback) {
      m_callback(camera_id, camera_serial, frame_id, frame);
    }
#ifdef ENABLE_VIDEO_UPDATE
    if (frame_id % 10 == 0) {
        cv::cvtColor(frame, bgr_frame, cv::COLOR_RGB2BGR);
        emit frame_received(camera_id, camera_serial, frame_id, bgr_frame);
    }
#endif
  }
  spdlog::debug("Camera {} worker thread stopped", camera_id);
}
} // namespace CameraManager