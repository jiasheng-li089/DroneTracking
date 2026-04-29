#include "DroneTrackingWindow.h"

#include <QBoxLayout>
#include <QPushButton>

#include <spdlog/spdlog.h>

#include "../camera/RealSenseManager.h"
#include "../network/WebRtcManager.h"
#include "../network/WebSocketSignaling.h"
#include "../tracking/VisionTracker.h"
#include "CameraWidget.h"

DroneTrackingWindow::DroneTrackingWindow(std::string config_file, QWidget* parent)
    : QMainWindow(parent), m_config_file(std::move(config_file)), m_rs_manager(std::make_unique<RealSenseManager>()) {

    m_webrtc_manager = std::make_unique<WebRtcManager>(std::make_unique<WebSocketSignaling>("ws://localhost:8188", "janus-protocol"));
    m_vision_tracker = std::make_unique<tracking::VisionTracker>();

    setup_ui();

    connect(this, &DroneTrackingWindow::update_widget_status, this, &DroneTrackingWindow::on_widget_status_update);
    connect(this, &DroneTrackingWindow::append_log, m_log_te, &QTextEdit::append);
    connect(m_webrtc_manager.get(), &WebRtcManager::on_connection_state, this, &DroneTrackingWindow::on_webrtc_connection_state);
    connect(m_rs_manager.get(), &RealSenseManager::frames_received, this, &DroneTrackingWindow::frames_received);
    connect(m_rs_manager.get(), &RealSenseManager::error_occurred, this, &DroneTrackingWindow::error_occurred);
}

DroneTrackingWindow::~DroneTrackingWindow() = default;

void DroneTrackingWindow::setup_ui() {
    this->setWindowTitle("Drone Tracking Control");
    this->resize(1600, 1200);

    auto root_widget = new QWidget(this);
    auto layout = new QVBoxLayout(root_widget);

    auto button_layout = new QHBoxLayout();
    m_start_camera_btn = new QPushButton("Start Camera", root_widget);
    m_stop_camera_btn = new QPushButton("Stop Camera", root_widget);
    m_start_tracking_btn = new QPushButton("Start Tracking", root_widget);
    m_stop_tracking_btn = new QPushButton("Stop Tracking", root_widget);

    std::vector<QWidget*> buttons = {m_start_camera_btn, m_stop_camera_btn, m_start_tracking_btn, m_stop_tracking_btn};
    auto button_id = 0;
    for (auto* btn : buttons) {
        btn->setProperty("button_id", button_id++);

        button_layout->addWidget(btn);
        connect(reinterpret_cast<QPushButton*>(btn), &QPushButton::clicked, this,
                [this, btn]() { on_button_clicked(btn); });

        btn->setEnabled(button_id % 2 == 1);
    }

    #ifdef DEBUG_CHANNEL
    auto debug_channel_btn = new QPushButton("Send Debug Message", root_widget);
    button_layout->addWidget(debug_channel_btn);
    connect(debug_channel_btn, &QPushButton::clicked, this, [this]() {
        m_webrtc_manager->sendMessage("Hello from debug channel!");
    });
    #endif

    layout->addLayout(button_layout);

    auto camera_widget = new QWidget(root_widget);
    m_camera_container = new QGridLayout(camera_widget);

    layout->addWidget(camera_widget);

    m_log_te = new QTextEdit(root_widget);
    m_log_te->setMaximumHeight(150);
    m_log_te->setReadOnly(true);
    layout->addWidget(m_log_te);

    this->setCentralWidget(root_widget);
}

void DroneTrackingWindow::on_widget_status_update(QWidget* sender, bool enable) { sender->setEnabled(enable); }

void DroneTrackingWindow::on_button_clicked(QWidget* sender) {
    switch (sender->property("button_id").toInt()) {
        case 0:
            m_log_te->append("Start Camera button clicked");
            m_rs_manager->start_cameras();
            on_widget_status_update(m_start_camera_btn, false);
            on_widget_status_update(m_stop_camera_btn, true);
            break;
        case 1:
            m_log_te->append("Stop Camera button clicked");
            m_rs_manager->stop_cameras();
            on_widget_status_update(m_start_camera_btn, true);
            on_widget_status_update(m_stop_camera_btn, false);
            break;
        case 2:
            m_log_te->append("Start Tracking button clicked");
            start_tracking();
            break;
        case 3:
            m_log_te->append("Stop Tracking button clicked");
            stop_tracking();
            break;
        default:
            break;
    }
}

void DroneTrackingWindow::on_webrtc_connection_state(bool connected) {
    if (connected) {
        m_log_te->append("WebRTC connection established");
        on_widget_status_update(m_start_tracking_btn, false);
        on_widget_status_update(m_stop_tracking_btn, true);

        m_rs_manager->set_frame_callback(std::bind(&tracking::VisionTracker::process_frames, m_vision_tracker.get(), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    } else {
        m_log_te->append("WebRTC connection lost");
        on_widget_status_update(m_start_tracking_btn, true);
        on_widget_status_update(m_stop_tracking_btn, false);
        m_rs_manager->set_frame_callback(nullptr);
    }


}

void DroneTrackingWindow::start_tracking() {
    m_log_te->append("Starting tracking...");

    on_widget_status_update(m_start_tracking_btn, false);
    on_widget_status_update(m_stop_tracking_btn, false);

    std::thread([this]() {

        try {
            m_webrtc_manager->connect();
        } catch (const std::exception& ex) {
            emit append_log(QString("Error starting tracking: ") + QString::fromStdString(ex.what()));
            emit update_widget_status(m_start_tracking_btn, true);
            emit update_widget_status(m_stop_tracking_btn, false);
            return;
        }
    }).detach();
}

void DroneTrackingWindow::stop_tracking() {
    m_log_te->append("Stopping tracking...");

    on_widget_status_update(m_start_tracking_btn, false);
    on_widget_status_update(m_stop_tracking_btn, false);

    std::thread([this]() {
        try {
            m_webrtc_manager->disconnect();
        } catch (const std::exception& ex) {
            emit append_log(QString("Error stopping tracking: ") + QString::fromStdString(ex.what()));
        }

        emit update_widget_status(m_start_tracking_btn, true);
        emit update_widget_status(m_stop_tracking_btn, false);
    }).detach();
}

void DroneTrackingWindow::frames_received(std::vector<std::tuple<int, std::string, QImage, rs2::depth_frame>> frames) {
    spdlog::debug("Received frames for processing");
    if (m_start_camera_btn->isEnabled() || frames.size() == 0) return;

    std::for_each(frames.begin(), frames.end(), [this](const auto& frameInfo) {
        int cameraId = std::get<0>(frameInfo);
        auto serial_str = std::get<1>(frameInfo);
        const QString& serial = QString::fromStdString(serial_str);

        if (m_camera_serials.find(serial_str) == m_camera_serials.end()) {
            m_camera_serials.insert(serial_str);
        }

        const QImage& img = std::get<2>(frameInfo);
        const auto& depth_frame_ptr = std::get<3>(frameInfo);

        if (cameraId < 100 || cameraId >= 200) {
            if (!m_camera_widgets.contains(cameraId)) {
                append_log(QString("Received %1 frame from camera %2 (Serial: %3)")
                               .arg(cameraId < 100 ? "color" : "infrared")
                               .arg(cameraId)
                               .arg(serial));

                CameraWidget* widget = new CameraWidget(this);
                widget->setMinimumSize(320, 240);

                int count = m_camera_widgets.size();
                int row = count / 2;
                int col = count % 2;
                m_camera_container->addWidget(widget, row, col);

                m_camera_widgets.insert(std::get<0>(frameInfo), widget);
            } else {
                m_camera_widgets[cameraId]->update_frame(img);
            }
        } else if (depth_frame_ptr) {
            // Handle depth frame
        }
    });
}

void DroneTrackingWindow::error_occurred(const QString& err) { m_log_te->append(err); }