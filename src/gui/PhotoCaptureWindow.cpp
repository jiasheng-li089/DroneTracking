#include "PhotoCaptureWindow.h"

#include <QDateTime>
#include <QWidget>

#include "../camera/RealSenseManager.h"
#include "CameraWidget.h"

PhotoCaptureWindow::PhotoCaptureWindow(std::string target_dir, QWidget* parent)
    : QMainWindow(parent),
      m_rs_manager(std::make_unique<RealSenseManager>()),
      m_capturer(std::make_unique<media::PhotoCaptureTask>(target_dir)) {
    setup_ui();

    connect(m_rs_manager.get(), &RealSenseManager::frames_received, this, &PhotoCaptureWindow::on_frame_received);
    connect(m_rs_manager.get(), &RealSenseManager::error_occurred, this, &PhotoCaptureWindow::on_camera_error);

    connect(m_capturer.get(), &media::PhotoCaptureTask::finalize_complete, this,
            &PhotoCaptureWindow::on_finalize_complete);
    connect(m_capturer.get(), &media::PhotoCaptureTask::update_capture_status, this,
            &PhotoCaptureWindow::on_update_capture_status);
}

PhotoCaptureWindow::~PhotoCaptureWindow() = default;

void PhotoCaptureWindow::setup_ui() {
    this->setWindowTitle("Drone Tracking Control");
    this->resize(800, 600);

    const auto centralWidget = new QWidget(this);
    const auto layout = new QVBoxLayout(centralWidget);

    const auto button_layout = new QHBoxLayout();
    m_start_btn = new QPushButton("Start", centralWidget);
    m_stop_btn = new QPushButton("Stop", centralWidget);
    m_capture_btn = new QPushButton("Capture", centralWidget);
    m_finalize_btn = new QPushButton("Finalize", centralWidget);

    button_layout->addWidget(m_start_btn);
    button_layout->addWidget(m_stop_btn);
    button_layout->addWidget(m_capture_btn);
    button_layout->addWidget(m_finalize_btn);

    m_log_te = new QTextEdit(centralWidget);
    m_log_te->setMaximumHeight(150);
    m_log_te->setReadOnly(true);

    const auto camera_widget = new QWidget(centralWidget);
    m_camera_container = new QGridLayout(camera_widget);

    layout->addLayout(button_layout);
    layout->addWidget(camera_widget);
    layout->addWidget(m_log_te);

    this->setCentralWidget(centralWidget);

    connect(reinterpret_cast<QPushButton*>(m_capture_btn), &QPushButton::clicked, this,
            &PhotoCaptureWindow::on_capture_photos);
    connect(reinterpret_cast<QPushButton*>(m_start_btn), &QPushButton::clicked, this, &PhotoCaptureWindow::on_start);
    connect(reinterpret_cast<QPushButton*>(m_stop_btn), &QPushButton::clicked, this, &PhotoCaptureWindow::on_stop);

    connect(reinterpret_cast<QPushButton*>(m_capture_btn), &QPushButton::clicked, this,
            &PhotoCaptureWindow::on_capture_photos);
    connect(reinterpret_cast<QPushButton*>(m_finalize_btn), &QPushButton::clicked, this,
            &PhotoCaptureWindow::on_finalize);

    m_start_btn->setEnabled(true);
    m_stop_btn->setEnabled(false);
    m_capture_btn->setEnabled(false);
}

void PhotoCaptureWindow::on_start() {
    append_log("Start showing video from cameras");

    m_rs_manager->start_cameras();
    m_start_btn->setEnabled(false);
    m_stop_btn->setEnabled(true);
    m_capture_btn->setEnabled(true);
}

void PhotoCaptureWindow::on_stop() {
    append_log("Stop showing video from cameras");

    m_rs_manager->stop_cameras();

    append_log("Stop successfully");

    QLayoutItem* child;
    while ((child = m_camera_container->takeAt(0)) != nullptr) {
        if (child->widget()) {
            delete child->widget();
        }
        delete child;
    }

    m_camera_widgets.clear();

    m_start_btn->setEnabled(true);
    m_stop_btn->setEnabled(false);
    m_capture_btn->setEnabled(false);
}

void PhotoCaptureWindow::on_capture_photos() {
    if (m_camera_widgets.isEmpty()) {
        append_log("No cameras available for capture.");
        return;
    }

    m_capturer->capture_frames(m_camera_serials);
}

void PhotoCaptureWindow::on_finalize() {
    m_finalize_btn->setEnabled(false);
    m_capturer->finalize();
}

void PhotoCaptureWindow::append_log(const QString& message) { m_log_te->append(message); }

void PhotoCaptureWindow::on_finalize_complete(bool success, const QString& message) {
    append_log(message);
    m_finalize_btn->setEnabled(true);
}

void PhotoCaptureWindow::on_update_capture_status(bool capturing) {
    if (capturing) {
        append_log("Photo capture started...");
        m_capture_btn->setEnabled(false);
    } else if (!m_capture_btn->isEnabled()) {
        append_log("Photo capture complete.");
        m_capture_btn->setEnabled(true);
    }
}

void PhotoCaptureWindow::on_frame_received(std::vector<std::tuple<int, std::string, QImage, rs2::depth_frame>> frames) {
    if (m_start_btn->isEnabled() || frames.size() == 0) return;

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
                append_log(QString("Received %1 frame from camera %2 (Serial: %3), video size: %4x%5")
                               .arg(cameraId < 100 ? "color" : "infrared")
                               .arg(cameraId)
                               .arg(serial)
                               .arg(img.width())
                               .arg(img.height()));

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

        if (m_capturer && cameraId >= 200 && cameraId < 300) {
            m_capturer->on_frame(serial_str, img);
        }
    });
}

void PhotoCaptureWindow::on_camera_error(const QString& err) { append_log(err); }
