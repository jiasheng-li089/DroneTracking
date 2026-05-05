#pragma once

#include <QMainWindow>
#include <QWidget>
#include <QMap>
#include <QTextEdit>
#include <QLabel>
#include <QGridLayout>

#include <librealsense2/rs.hpp>


class RealSenseManager;
class WebRtcManager;
class Signaling;

class CameraWidget;

namespace tracking {
    class VisionTracker;
}

class DroneTrackingWindow : public QMainWindow {
    Q_OBJECT

    public:
        explicit DroneTrackingWindow(std::string config_file, QWidget *parent = nullptr);
        ~DroneTrackingWindow() override;

    private:
        void setup_ui();

        void start_tracking();

        void stop_tracking();

        std::unique_ptr<RealSenseManager> m_rs_manager;
        std::unique_ptr<WebRtcManager> m_webrtc_manager;
        std::unique_ptr<tracking::VisionTracker> m_vision_tracker;

        QWidget * m_start_camera_btn{};
        QWidget * m_stop_camera_btn{};
        QWidget * m_start_tracking_btn{};
        QWidget * m_stop_tracking_btn{};

        QTextEdit * m_log_te{};

        QGridLayout * m_camera_container{};

        QGridLayout * m_labels_container{};

        QMap<int, CameraWidget*> m_camera_widgets;

        QMap<std::string, QLabel*> m_camera_labels;

        std::string m_config_file;

        std::set<std::string> m_camera_serials;

    signals:
        void update_widget_status(QWidget* sender, bool enable);
        void append_log(const QString& message);

    private slots:

        void on_button_clicked(QWidget* sender);

        void frames_received(std::vector<std::tuple<int, std::string, QImage>> frames);
        
        void error_occurred(const QString& err);

        void on_widget_status_update(QWidget* sender, bool enable);

        void on_webrtc_connection_state(bool connected);

        void on_update_camera_status(std::string serial, std::string status);
};