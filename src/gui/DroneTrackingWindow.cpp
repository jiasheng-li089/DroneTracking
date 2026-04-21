#include "DroneTrackingWindow.h"

#include <QBoxLayout>
#include <QPushButton>

#include "../camera/RealSenseManager.h"
#include "../network/WebSocketSignaling.h"

DroneTrackingWindow::DroneTrackingWindow(std::string config_file, QWidget* parent)
    : QMainWindow(parent), m_config_file(std::move(config_file)), m_rs_manager(std::make_unique<RealSenseManager>()) {
    setup_ui();

    connect(this, &DroneTrackingWindow::update_widget_status, this, &DroneTrackingWindow::on_widget_status_update);
    connect(this, &DroneTrackingWindow::append_log, m_log_te, &QTextEdit::append);
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
    }

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
            break;
        case 1:
            m_log_te->append("Stop Camera button clicked");
            m_rs_manager->stop_cameras();
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

void DroneTrackingWindow::start_tracking() {
    m_log_te->append("Starting tracking...");

    on_widget_status_update(m_start_tracking_btn, false);
    on_widget_status_update(m_stop_tracking_btn, false);

    std::thread([this]() {
        m_signaling = std::make_unique<WebSocketSignaling>("ws://localhost:8188", "janus-protocol");

        try {
            auto answer = m_signaling->exchange_offer("dummy offer");

            emit append_log("Received answer: " + QString::fromStdString(answer));

            update_widget_status(m_stop_tracking_btn, true);
            update_widget_status(m_start_tracking_btn, false);
        } catch (const std::exception& e) {
            emit append_log("Error during offer exchange: " + QString::fromStdString(e.what()));
            update_widget_status(m_start_tracking_btn, true);
            update_widget_status(m_stop_tracking_btn, false);
        }
    }).detach();
}

void DroneTrackingWindow::stop_tracking() {
    m_log_te->append("Stopping tracking...");

    on_widget_status_update(m_start_tracking_btn, false);
    on_widget_status_update(m_stop_tracking_btn, false);

    std::thread([this]() {
        if (m_signaling) {
            m_signaling->end();
            m_signaling.reset();
        }

        emit append_log("Tracking stopped");

        update_widget_status(m_start_tracking_btn, true);
        update_widget_status(m_stop_tracking_btn, false);
    }).detach();
}

void DroneTrackingWindow::frames_received(std::vector<std::tuple<int, std::string, QImage, rs2::depth_frame>> frames) {
    for (const auto& frame_data : frames) {
        int camera_id = std::get<0>(frame_data);
        std::string serial = std::get<1>(frame_data);
        QImage img = std::get<2>(frame_data);
        rs2::depth_frame depth_frame = std::get<3>(frame_data);
    }
}

void DroneTrackingWindow::error_occurred(const QString& err) { m_log_te->append(err); }