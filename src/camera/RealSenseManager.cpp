#include "RealSenseManager.h"

#include <librealsense2/h/rs_sensor.h>
#include <spdlog/spdlog.h>

#include <chrono>
#include <librealsense2/rs.hpp>

#include "logger.h"

RealSenseManager::RealSenseManager(QObject* parent) : QObject(parent), m_running(false) {}

RealSenseManager::~RealSenseManager() { stop_cameras(); }

void RealSenseManager::start_cameras() {
    if (m_running) return;

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
        }  // ctx destroyed here, releasing device handles before pipelines open

        m_running = true;
        for (size_t i = 0; i < serials.size(); ++i) {
            spdlog::debug("Starting RealSense camera {} with serial {}", i, serials[i]);
            m_threads.emplace_back(&RealSenseManager::camera_worker_thread, this, static_cast<int>(i), serials[i]);
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
    spdlog::debug("Camera worker thread started for cameraId: {}, serial: {}", cameraId, serial);

    try {
        rs2::pipeline p;
        rs2::config cfg;
        cfg.enable_device(serial);
        // Request color stream
        cfg.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_RGB8, 30);

#ifdef ENABLE_DEPTH_CAMERA
        // Request infra streams (1 and 2 usually for stereo)
        // Some older librealsense versions or specific cameras might not support
        // both at 640x480 at 30 fps If they fail to start, config fallback might be
        // needed, but this is standard for D435.
        cfg.enable_stream(RS2_STREAM_DEPTH, 1024, 768, RS2_FORMAT_Z16, 30);
        cfg.enable_stream(RS2_STREAM_INFRARED, 1024, 768, RS2_FORMAT_Y8, 30);
#endif

        for (int retry = 0; retry < 5; ++retry) {
            try {
                p.start(cfg);
                break;
            } catch (const rs2::error& e) {
                if (retry == 4) throw;
                spdlog::warn("Camera {} pipeline start failed (attempt {}): {}, retrying...", cameraId, retry + 1,
                             e.what());
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
        }
        spdlog::debug("Camera {} (Serial: {}) pipeline started", cameraId, serial);

#ifdef ENABLE_DEPTH_CAMERA
        rs2::align align_to_color(RS2_STREAM_COLOR);

        for (auto& sensor : p.get_active_profile().get_device().query_sensors()) {
            if (!sensor.is<rs2::depth_sensor>()) continue;
            for (int retry = 0; retry < 3; ++retry) {
                try {
                    if (sensor.supports(RS2_OPTION_MIN_DISTANCE)) sensor.set_option(RS2_OPTION_MIN_DISTANCE, 0.25f);
                    if (sensor.supports(RS2_OPTION_MAX_DISTANCE)) sensor.set_option(RS2_OPTION_MAX_DISTANCE, 6.5f);
                    break;
                } catch (const rs2::error& e) {
                    if (retry == 2) throw;
                    spdlog::warn("set_option failed (attempt {}): {}, retrying...", retry + 1, e.what());
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                    emit error_occurred(QString("Camera %1 option set failed (attempt %2): %3")
                                            .arg(cameraId)
                                            .arg(retry + 1)
                                            .arg(e.what()));
                }
            }
            break;
        }
#endif

        while (m_running) {
            rs2::frameset frames;
            if (p.try_wait_for_frames(&frames,
                                      50)) {  // 50ms timeout to avoid busy wait
                const bool log_frame = frames.get_frame_number() % 30 == 0;

                if (log_frame) {
                    spdlog::debug("Frameset received from cameraId: {}, serial: {}, frame #{}", cameraId, serial,
                                  frames.get_frame_number());
                }

                rs2::frameset processed_frames = frames;
#ifdef ENABLE_DEPTH_CAMERA
                if (frames.get_color_frame() && frames.get_depth_frame()) {
                    try {
                        processed_frames = align_to_color.process(frames);
                    } catch (const rs2::error& e) {
                        spdlog::warn("Alignment failed: {}", e.what());
                    } catch (const std::exception& e) {
                        spdlog::warn("Alignment threw exception: {}", e.what());
                    }
                }
#endif

                std::vector<std::tuple<int, std::string, QImage>> frame_data;

                rs2::video_frame color = processed_frames.get_color_frame();

                if (color) {
                    QImage img((const uchar*)color.get_data(), color.get_width(), color.get_height(),
                               color.get_stride_in_bytes(), QImage::Format_RGB888);
                    frame_data.emplace_back(cameraId, serial, img.copy());
                    if (log_frame) {
                        spdlog::debug("Color frame processed for cameraId: {}, serial: {}, timestamp: {}", cameraId,
                                      serial, color.get_timestamp());
                    }
                }

#ifdef ENABLE_DEPTH_CAMERA
                rs2::depth_frame depth = processed_frames.get_depth_frame();
                if (depth) {
                    // Process depth frame if needed
                    QImage depthImg((const uchar*)depth.get_data(), depth.get_width(), depth.get_height(),
                                    depth.get_stride_in_bytes(), QImage::Format_Grayscale16);
                    frame_data.emplace_back(cameraId + 100, serial, depthImg.copy());
                    if (log_frame) {
                        auto origin_depth = frames.get_depth_frame();
                        spdlog::debug(
                            "Depth frame processed for cameraId: {}, serial: {}, timestamp: {}, resolution: "
                            "{} x{}, "
                            "original resolution : {} x {} ", cameraId, serial, depth.get_timestamp(),
                            depth.get_width(),
                            depth.get_height(), origin_depth.get_width(), origin_depth.get_height());
                    }
                }

                rs2::video_frame gray_frame = processed_frames.first(RS2_STREAM_INFRARED);
                if (gray_frame) {
                    QImage grayImg((const uchar*)gray_frame.get_data(), gray_frame.get_width(), gray_frame.get_height(),
                                   gray_frame.get_stride_in_bytes(), QImage::Format_Grayscale8);
                    frame_data.emplace_back(cameraId + 200, serial, grayImg.copy());
                    if (log_frame) {
                        spdlog::debug("Infrared frame processed for cameraId: {}, serial: {}, timestamp: {}", cameraId,
                                      serial, gray_frame.get_timestamp());
                    }
                }
#endif

                if (frame_data.size() > 0) {
                    if (log_frame) {
                        spdlog::debug("Emitting frames_received signal for cameraId: {}, serial: {}, frame count: {}",
                                      cameraId, serial, frame_data.size());
                    }
                    emit frames_received(std::move(frame_data));
                }

                if (m_frame_callback) {
                    m_frame_callback(cameraId, serial, processed_frames);
                }
            }
        }

        p.stop();

    } catch (const rs2::error& e) {
        emit error_occurred(QString("Camera %1 error: %2").arg(cameraId).arg(e.what()));
    }
}

void RealSenseManager::set_frame_callback(std::function<void(const int, const std::string&, rs2::frameset)> callback) {
    m_frame_callback = std::move(callback);
}