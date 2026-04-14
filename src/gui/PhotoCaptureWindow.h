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
    void onCapturePhotos() const;
    void appendLog(const QString& message) const;

    void onFrameReceived(int cameraId, const QImage& img);
    void onCameraError(const QString& err);

private:
    void setupUi();

    std::unique_ptr<RealSenseManager> m_rsManager;

    QPushButton* m_btnConnectSignaling{};
    QTextEdit * m_logTextEdit{};

    QWidget* m_camerasContainer{};
    QGridLayout* m_camerasLayout{};
    QMap<int, CameraWidget*> m_cameraWidgets;
};
