#pragma once
#include <QObject>
#include <QImage>
#include <vector>
#include <thread>
#include <atomic>
#include <string>

#include <librealsense2/rs.hpp>

class RealSenseManager : public QObject {
    Q_OBJECT
public:
    explicit RealSenseManager(QObject* parent = nullptr);
    ~RealSenseManager() override;

    void start_cameras();
    void stop_cameras();

    void set_frame_callback(std::function<void(const int, const std::string &, rs2::frameset)> callback);

signals:
    void frames_received(std::vector<std::tuple<int, std::string, QImage, rs2::depth_frame>> frames);
    void error_occurred(const QString& err);

private:
    void camera_worker_thread(int cameraId, std::string serial);

    std::vector<std::thread> m_threads;
    std::atomic<bool> m_running;
    std::atomic<uint64_t> m_captureEpoch{0};
    std::function<void(const int, const std::string &, rs2::frameset)> m_frame_callback;
};
