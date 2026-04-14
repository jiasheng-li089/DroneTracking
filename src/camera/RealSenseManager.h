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

signals:
    void frameReceived(int cameraId, std::string serial, const QImage& image);
    void errorOccurred(const QString& err);

private:
    void cameraWorkerThread(int cameraId, std::string serial);

    std::vector<std::thread> m_threads;
    std::atomic<bool> m_running;
};
