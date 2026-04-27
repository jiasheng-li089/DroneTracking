#include "RealSenseManager.h"
#include "logger.h"

#include <librealsense2/h/rs_sensor.h>
#include <librealsense2/rs.hpp>

RealSenseManager::RealSenseManager(QObject* parent) : QObject(parent), m_running(false) {}

RealSenseManager::~RealSenseManager() { stop_cameras(); }

void RealSenseManager::start_cameras() {
    if (m_running) return;

    try {
        rs2::context ctx;
        auto devices = ctx.query_devices();
        if (devices.size() == 0) {
            emit error_occurred("No RealSense devices found!");
            return;
        }

        m_running = true;

        for (size_t i = 0; i < devices.size(); ++i) {
            std::string serial = devices[i].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            m_threads.emplace_back(&RealSenseManager::camera_worker_thread, this, static_cast<int>(i), serial);
        }
    } catch (const rs2::error& e) {
        emit error_occurred(QString("RealSense error: %1").arg(e.what()));
    } catch (const std::exception& e) {
        emit error_occurred(QString("Standard exception: %1").arg(e.what()));
    }
}

void RealSenseManager::stop_cameras() {
    m_running = false;
    for (auto& t : m_threads) {
        if (t.joinable()) {
            t.join();
        } else {
            spdlog::warn("RealSense thread stopped and not joinable");
        }
    }
    m_threads.clear();
}

void RealSenseManager::camera_worker_thread(int cameraId, std::string serial) {
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
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
        cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);

        p.start(cfg);

        while (m_running) {
            rs2::frameset frames;
            if (p.try_wait_for_frames(&frames,
                                      50)) {  // 50ms timeout to avoid busy wait
                std::vector<std::tuple<int, std::string, QImage, rs2::depth_frame>> frame_data;

                rs2::video_frame color = frames.get_color_frame();

                if (color) {
                    QImage img((const uchar*)color.get_data(), color.get_width(), color.get_height(),
                               color.get_stride_in_bytes(), QImage::Format_RGB888);
                    frame_data.emplace_back(cameraId, serial, img.copy(), nullptr);
                }

                rs2::depth_frame depth = frames.get_depth_frame();
                if (depth) {
                    // Process depth frame if needed
                    QImage depthImg((const uchar*)depth.get_data(), depth.get_width(), depth.get_height(),
                                    depth.get_stride_in_bytes(), QImage::Format_Grayscale16);
                    frame_data.emplace_back(cameraId + 100, serial, depthImg.copy(), depth);
                }

                auto gray_frame = frames.get_infrared_frame();
                if (gray_frame) {
                    QImage grayImg((const uchar*)gray_frame.get_data(), gray_frame.get_width(), gray_frame.get_height(),
                                   gray_frame.get_stride_in_bytes(), QImage::Format_Grayscale8);
                    frame_data.emplace_back(cameraId + 200, serial, grayImg.copy(), nullptr);
                }

                if (m_frame_callback) {
                    m_frame_callback(cameraId, serial, frames);
                }

                if (frame_data.size() > 0) {
                    emit frames_received(std::move(frame_data));
                }
            }
        }

        p.stop();

    } catch (const rs2::error& e) {
        emit error_occurred(QString("Camera %1 error: %2").arg(cameraId).arg(e.what()));
    }
}

void RealSenseManager::set_frame_callback(std::function<void(const int, const std::string &, rs2::frameset)> callback) {
    m_frame_callback = std::move(callback);
}
