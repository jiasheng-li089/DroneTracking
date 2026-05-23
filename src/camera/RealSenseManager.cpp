#include "RealSenseManager.h"

#include <librealsense2/h/rs_sensor.h>

#include <chrono>
#include <librealsense2/rs.hpp>
#include <thread>

RealSenseManager::RealSenseManager(QObject *parent)
    : CameraManager::CameraManager(parent) {}

RealSenseManager::~RealSenseManager() { stop_cameras(); }

void RealSenseManager::start_cameras() {
  if (m_running)
    return;

  try {
    std::vector<std::string> serials;
    {
      rs2::context ctx;
      auto devices = ctx.query_devices();
      if (devices.size() == 0) {
        emit error_occurred("No RealSense devices found!");
        return;
      }
      spdlog::info("Found {} RealSense devices", devices.size());
      for (size_t i = 0; i < devices.size(); ++i)
        serials.push_back(devices[i].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
    } // ctx destroyed here, releasing device handles before pipelines open

    m_running = true;
    for (size_t i = 0; i < serials.size(); ++i) {
      spdlog::debug("Starting RealSense camera {} with serial {}", i,
                    serials[i]);
      std::unique_ptr<std::thread> thread =
          std::make_unique<std::thread>(&RealSenseManager::camera_worker_thread,
                                        this, static_cast<int>(i), serials[i]);
      m_threads.insert_or_assign(serials[i], std::move(thread));
    }
  } catch (const rs2::error &e) {
    emit error_occurred(QString("RealSense error: %1").arg(e.what()));
  } catch (const std::exception &e) {
    emit error_occurred(QString("Standard exception: %1").arg(e.what()));
  }
}

void RealSenseManager::stop_cameras() {
  m_running = false;
  for (auto &[serial, t] : m_threads) {
    if (t->joinable()) {
      t->join();
    } else {
      spdlog::warn("RealSense thread stopped and not joinable");
    }
  }
  m_threads.clear();
}

void RealSenseManager::camera_worker_thread(int cameraId, std::string serial) {
  spdlog::debug("Camera worker thread started for cameraId: {}, serial: {}",
                cameraId, serial);

  try {
    rs2::pipeline p;
    rs2::config cfg;
    cfg.enable_device(serial);
    // Request color stream
    cfg.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_RGB8, 30);

    for (int retry = 0; retry < 5; ++retry) {
      try {
        p.start(cfg);
        break;
      } catch (const rs2::error &e) {
        if (retry == 4)
          throw;
        spdlog::warn(
            "Camera {} pipeline start failed (attempt {}): {}, retrying...",
            cameraId, retry + 1, e.what());
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    }
    spdlog::debug("Camera {} (Serial: {}) pipeline started", cameraId, serial);

    while (m_running) {
      rs2::frameset frames;
      if (p.try_wait_for_frames(&frames,
                                50)) { // 50ms timeout to avoid busy wait
        const bool log_frame = frames.get_frame_number() % 30 == 0;

        if (log_frame) {
          spdlog::debug(
              "Frameset received from cameraId: {}, serial: {}, frame #{}",
              cameraId, serial, frames.get_frame_number());
        }

        rs2::frameset processed_frames = frames;

        std::vector<std::tuple<int, std::string, QImage>> frame_data;

        rs2::video_frame color = processed_frames.get_color_frame();

        if (color) {
          cv::Mat image = cv::Mat(color.get_height(), color.get_width(),
                                  CV_8UC3, const_cast<void *>(color.get_data()),
                                  color.get_stride_in_bytes());
          emit frame_received(cameraId, serial, frames.get_frame_number(),
                              image);

          if (m_callback) {
            m_callback(cameraId, serial, frames.get_frame_number(), image);
          }
          if (log_frame) {
            spdlog::debug("Color frame processed for cameraId: {}, serial: {}, "
                          "timestamp: {}",
                          cameraId, serial, color.get_timestamp());
          }
        }
      }
    }

    p.stop();

  } catch (const rs2::error &e) {
    emit error_occurred(
        QString("Camera %1 error: %2").arg(cameraId).arg(e.what()));
  }
}