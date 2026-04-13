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

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

private slots:
    void onConnectSignalingClicked();
    void onStartWebRtcClicked();
    void appendLog(const QString& message);

    void onFrameReceived(int cameraId, const QImage& img);
    void onCameraError(const QString& err);

private:
    void setupUi();

    std::unique_ptr<WebSocketClient> m_webSocketClient;
    std::unique_ptr<WebRtcManager> m_webRtcManager;
    std::unique_ptr<RealSenseManager> m_rsManager;

    QPushButton* m_btnConnectSignaling;
    QPushButton* m_btnStartWebRtc;
    QTextEdit* m_logTextEdit;

    QWidget* m_camerasContainer;
    QGridLayout* m_camerasLayout;
    QMap<int, CameraWidget*> m_cameraWidgets;
};
