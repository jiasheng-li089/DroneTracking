#include "RealSenseManager.h"

#include <QDebug>
#include <iostream>

#include <librealsense2/rs.hpp>

RealSenseManager::RealSenseManager(QObject *parent)
    : QObject(parent), m_running(false) {}

RealSenseManager::~RealSenseManager() { stopCameras(); }

void RealSenseManager::startCameras() {
  if (m_running)
    return;

  try {
    rs2::context ctx;
    auto devices = ctx.query_devices();
    if (devices.size() == 0) {
      emit errorOccurred("No RealSense devices found!");
      return;
    }

    m_running = true;

    for (size_t i = 0; i < devices.size(); ++i) {
      std::string serial = devices[i].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
      m_threads.emplace_back(&RealSenseManager::cameraWorkerThread, this,
                             static_cast<int>(i), serial);
    }
  } catch (const rs2::error &e) {
    emit errorOccurred(QString("RealSense error: %1").arg(e.what()));
  } catch (const std::exception &e) {
    emit errorOccurred(QString("Standard exception: %1").arg(e.what()));
  }
}

void RealSenseManager::stopCameras() {
  m_running = false;
  for (auto &t : m_threads) {
    if (t.joinable()) {
      t.join();
    } else {
      std::cout << "RealSense thread stopped and not joinable" << std::endl;
    }
  }
  m_threads.clear();
}

void RealSenseManager::captureInfraPhotos() { m_captureEpoch.fetch_add(1); }

void RealSenseManager::cameraWorkerThread(int cameraId, std::string serial) {
  uint64_t lastProcessedEpoch = m_captureEpoch.load();
  try {
    rs2::pipeline p;
    rs2::config cfg;
    cfg.enable_device(serial);
    // Request color stream
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
    // Request infra streams (1 and 2 usually for stereo)
    // Some older librealsense versions or specific cameras might not support
    // both at 640x480 at 30 fps If they fail to start, config fallback might be
    // needed, but this is standard for D435.
    cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);

    p.start(cfg);

    while (m_running) {
      rs2::frameset frames;
      if (p.try_wait_for_frames(&frames,
                                50)) { // 50ms timeout to avoid busy wait
        rs2::video_frame color = frames.get_color_frame();
        if (color) {
          QImage img((const uchar *)color.get_data(), color.get_width(),
                     color.get_height(), color.get_stride_in_bytes(),
                     QImage::Format_RGB888);

          // Emit deep copy so it thread-safely travels to the GUI thread
          emit frameReceived(cameraId, serial, img.copy());
        }

        uint64_t currentEpoch = m_captureEpoch.load();
        if (currentEpoch > lastProcessedEpoch) {
          lastProcessedEpoch = currentEpoch;

          rs2::video_frame ir1_frame = frames.get_infrared_frame(1);
          rs2::video_frame ir2_frame = frames.get_infrared_frame(2);

          QImage img_ir1, img_ir2;
          if (ir1_frame) {
            img_ir1 = QImage((const uchar *)ir1_frame.get_data(),
                             ir1_frame.get_width(), ir1_frame.get_height(),
                             ir1_frame.get_stride_in_bytes(),
                             QImage::Format_Grayscale8)
                          .copy();
          }
          if (ir2_frame) {
            img_ir2 = QImage((const uchar *)ir2_frame.get_data(),
                             ir2_frame.get_width(), ir2_frame.get_height(),
                             ir2_frame.get_stride_in_bytes(),
                             QImage::Format_Grayscale8)
                          .copy();
          }
          emit infraFramesCaptured(cameraId, serial, img_ir1, img_ir2);
        }
      }
    }

    p.stop();

  } catch (const rs2::error &e) {
    emit errorOccurred(
        QString("Camera %1 error: %2").arg(cameraId).arg(e.what()));
  }
}
