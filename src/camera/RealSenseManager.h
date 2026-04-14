#pragma once
#include <QObject>
#include <QImage>
#include <vector>
#include <thread>
#include <atomic>
#include <string>

class RealSenseManager : public QObject {
    Q_OBJECT
public:
    explicit RealSenseManager(QObject* parent = nullptr);
    ~RealSenseManager() override;

    void startCameras();
    void stopCameras();
    void captureInfraPhotos();

signals:
    void frameReceived(int cameraId, std::string serial, const QImage& image);
    void infraFramesCaptured(int cameraId, std::string serial, const QImage& ir1, const QImage& ir2);
    void errorOccurred(const QString& err);

private:
    void cameraWorkerThread(int cameraId, std::string serial);

    std::vector<std::thread> m_threads;
    std::atomic<bool> m_running;
    std::atomic<uint64_t> m_captureEpoch{0};
};
