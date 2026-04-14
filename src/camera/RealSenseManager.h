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

    void startCameras();
    void stopCameras();
    void captureInfraPhotos();

signals:
    void framesReceived(std::vector<std::tuple<int, std::string, QImage, rs2::depth_frame>> frames);
    void errorOccurred(const QString& err);

private:
    void cameraWorkerThread(int cameraId, std::string serial);

    std::vector<std::thread> m_threads;
    std::atomic<bool> m_running;
    std::atomic<uint64_t> m_captureEpoch{0};
};
