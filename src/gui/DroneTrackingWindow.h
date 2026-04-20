#pragma once

#include <QMainWindow>
#include <QWidget>
#include <QMap>
#include <QTextEdit>
#include <QGridLayout>

#include <librealsense2/rs.hpp>


class RealSenseManager;

class CameraWidget;

class DroneTrackingWindow : public QMainWindow {
    Q_OBJECT

    public:
        explicit DroneTrackingWindow(std::string config_file, QWidget *parent = nullptr);
        ~DroneTrackingWindow() override;

    private:
        void setup_ui();

        std::unique_ptr<RealSenseManager> m_rs_manager;

        QWidget * m_start_camera_btn{};
        QWidget * m_stop_camera_btn{};
        QWidget * m_start_tracking_btn{};
        QWidget * m_stop_tracking_btn{};

        QTextEdit * m_log_te{};

        QGridLayout * m_camera_container{};

        QMap<int, CameraWidget*> m_camera_widgets;

        std::string m_config_file;

    private slots:

        void on_button_clicked(QWidget* sender);

        void frames_received(std::vector<std::tuple<int, std::string, QImage, rs2::depth_frame>> frames);
        
        void error_occurred(const QString& err);
};