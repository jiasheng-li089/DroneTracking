#pragma once

#include <QMainWindow>
#include <QGridLayout>
#include <QPushButton>
#include <QTextEdit>
#include <QMap>
#include <memory>

#include <librealsense2/rs.hpp>

#include "../media/PhotoCaptureTask.h"

class CameraWidget;
class RealSenseManager;

class WebSocketClient;
class WebRtcManager;

class PhotoCaptureWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit PhotoCaptureWindow(std::string target_dir, QWidget *parent = nullptr);
    ~PhotoCaptureWindow() override;

private slots:
    void on_start();
    void on_stop();

    void on_capture_photos();
    void on_finalize();
    void append_log(const QString& message);

    void on_finalize_complete(bool success, const QString& message);
    void on_update_capture_status(bool capturing);

    void on_frame_received(std::vector<std::tuple<int, std::string, QImage, rs2::depth_frame>> frames);
    void on_camera_error(const QString& err);

private:
    void setup_ui();

    std::unique_ptr<RealSenseManager> m_rs_manager;
    std::unique_ptr<media::PhotoCaptureTask> m_capturer;

    QTextEdit * m_log_te{};
    QWidget * m_start_btn{};
    QWidget * m_stop_btn{};
    QWidget * m_capture_btn{};
    QWidget * m_finalize_btn{};

    QGridLayout * m_camera_container{};

    QMap<int, CameraWidget*> m_camera_widgets;

    std::set<std::string> m_camera_serials;
};
