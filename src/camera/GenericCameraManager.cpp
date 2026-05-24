#include "GenericCamerManager.h"

#include <QObject>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <spdlog/spdlog.h>
#include <thread>
#include <vector>

#include "camera_util.h"

namespace CameraManager {

GenericCameraManager::GenericCameraManager(QObject *parent)
    : CameraManager(parent) {}

void GenericCameraManager::start_cameras() {
  if (m_running)
    return;
  spdlog::info("Starting GenericCameraManager...");
  m_running = true;
  std::vector<int> camera_ids = detect_rgb_cameras();
  for (int camera_id : camera_ids) {
    std::string camera_name = "Camera " + std::to_string(camera_id);
    std::unique_ptr<std::thread> thread = std::make_unique<std::thread>(
        &GenericCameraManager::camera_worker_thread, this, camera_id,
        camera_name);
    m_threads.insert_or_assign(camera_name, std::move(thread));
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
    const int camera_id, const std::string camera_serial) {
  spdlog::info("Starting worker thread for camera {}, serial: {}", camera_id,
               camera_serial);

  cv::VideoCapture cap(camera_id);

  if (!cap.isOpened()) {
    spdlog::error("Failed to open camera {}", camera_id);
    emit error_occurred(
        QString::fromStdString("Failed to open camera #{}").arg(camera_id));
    return;
  }

  cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
  cap.set(cv::CAP_PROP_FPS, 30);

  spdlog::info("Camera {} properties: width={}, height={}, fps={}", camera_id,
               cap.get(cv::CAP_PROP_FRAME_WIDTH),
               cap.get(cv::CAP_PROP_FRAME_HEIGHT), cap.get(cv::CAP_PROP_FPS));

  ulong frame_id = 0;
  cv::Mat frame, bgr_frame;
  while (m_running) {
    cv::Mat frame;
    cap >> frame;
    frame_id++;
    if (frame.empty()) {
      spdlog::error("Failed to capture frame from camera {} for frame id: #{}",
                    camera_id, frame_id);
      break;
    }
    cv::cvtColor(frame, bgr_frame, cv::COLOR_RGB2BGR);
    if (m_callback) {
      m_callback(camera_id, camera_serial, frame_id, bgr_frame);
    }

    emit frame_received(camera_id, camera_serial, frame_id, bgr_frame);
  }
  spdlog::debug("Camera {} worker thread stopped", camera_id);
}
} // namespace CameraManager