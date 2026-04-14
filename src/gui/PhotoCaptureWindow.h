#pragma once

#include <QMainWindow>
#include <QGridLayout>
#include <QPushButton>
#include <QTextEdit>
#include <QMap>
#include <memory>

class CameraWidget;
class RealSenseManager;

class WebSocketClient;
class WebRtcManager;

class PhotoCaptureWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit PhotoCaptureWindow(QWidget *parent = nullptr);
    ~PhotoCaptureWindow() override;

private slots:
    void onStart();
    void onStop();
    void onCapturePhotos();
    void appendLog(const QString& message);

    void onFrameReceived(int cameraId, const QImage& img);
    void onCameraError(const QString& err);

private:
    void setupUi();

    std::unique_ptr<RealSenseManager> m_rsManager;

    QTextEdit * m_logTextEdit{};
    QWidget * m_start_btn{};
    QWidget * m_stop_btn{};
    QWidget * m_capture_btn{};

    QGridLayout * m_camera_container{};

    QMap<int, CameraWidget*> m_cameraWidgets;
};
