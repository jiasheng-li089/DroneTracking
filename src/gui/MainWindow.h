#pragma once

#include <QMainWindow>
#include <QVBoxLayout>
#include <QPushButton>
#include <QTextEdit>
#include <memory>

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

private:
    void setupUi();

    std::unique_ptr<WebSocketClient> m_webSocketClient;
    std::unique_ptr<WebRtcManager> m_webRtcManager;

    QPushButton* m_btnConnectSignaling;
    QPushButton* m_btnStartWebRtc;
    QTextEdit* m_logTextEdit;
};
