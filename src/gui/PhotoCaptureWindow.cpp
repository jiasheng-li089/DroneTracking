#include "PhotoCaptureWindow.h"

#include <QDateTime>
#include <QWidget>

#include "../camera/RealSenseManager.h"
#include "CameraWidget.h"

PhotoCaptureWindow::PhotoCaptureWindow(QWidget* parent)
    : QMainWindow(parent), m_rsManager(std::make_unique<RealSenseManager>()) {
    setupUi();

    connect(m_rsManager.get(), &RealSenseManager::framesReceived, this, &PhotoCaptureWindow::onFrameReceived);
    connect(m_rsManager.get(), &RealSenseManager::errorOccurred, this, &PhotoCaptureWindow::onCameraError);
}

PhotoCaptureWindow::~PhotoCaptureWindow() = default;

void PhotoCaptureWindow::setupUi() {
    this->setWindowTitle("Drone Tracking Control");
    this->resize(800, 600);

    const auto centralWidget = new QWidget(this);
    const auto layout = new QVBoxLayout(centralWidget);

    const auto button_layout = new QHBoxLayout();
    m_start_btn = new QPushButton("Start", centralWidget);
    m_stop_btn = new QPushButton("Stop", centralWidget);
    m_capture_btn = new QPushButton("Capture", centralWidget);

    button_layout->addWidget(m_start_btn);
    button_layout->addWidget(m_stop_btn);
    button_layout->addWidget(m_capture_btn);

    m_logTextEdit = new QTextEdit(centralWidget);
    m_logTextEdit->setMaximumHeight(150);
    m_logTextEdit->setReadOnly(true);

    const auto camera_widget = new QWidget(centralWidget);
    m_camera_container = new QGridLayout(camera_widget);

    layout->addLayout(button_layout);
    layout->addWidget(camera_widget);
    layout->addWidget(m_logTextEdit);

    this->setCentralWidget(centralWidget);

    connect(reinterpret_cast<QPushButton*>(m_capture_btn), &QPushButton::clicked, this,
            &PhotoCaptureWindow::onCapturePhotos);
    connect(reinterpret_cast<QPushButton*>(m_start_btn), &QPushButton::clicked, this, &PhotoCaptureWindow::onStart);
    connect(reinterpret_cast<QPushButton*>(m_stop_btn), &QPushButton::clicked, this, &PhotoCaptureWindow::onStop);

    m_start_btn->setEnabled(true);
    m_stop_btn->setEnabled(false);
    m_capture_btn->setEnabled(false);
}

void PhotoCaptureWindow::onStart() {
    appendLog("Start showing video from cameras");
    // TODO
    m_rsManager->startCameras();
    m_start_btn->setEnabled(false);
    m_stop_btn->setEnabled(true);
    m_capture_btn->setEnabled(true);
}

void PhotoCaptureWindow::onStop() {
    appendLog("Stop showing video from cameras");
    m_rsManager->stopCameras();

    appendLog("Stop successfully");

    QLayoutItem* child;
    while ((child = m_camera_container->takeAt(0)) != nullptr) {
        if (child->widget()) {
            delete child->widget();
        }
        delete child;
    }

    m_cameraWidgets.clear();

    m_start_btn->setEnabled(true);
    m_stop_btn->setEnabled(false);
    m_capture_btn->setEnabled(false);
}

void PhotoCaptureWindow::onCapturePhotos() {
    if (m_cameraWidgets.isEmpty()) {
        appendLog("No cameras available for capture.");
        return;
    }

    QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    QString targetDir = "captures/" + timestamp;

    appendLog("Capturing photos to " + targetDir);

    m_currentCaptureTask = std::make_shared<media::PhotoCaptureTask>(m_cameraWidgets.size(), targetDir.toStdString());

    connect(m_currentCaptureTask.get(), &media::PhotoCaptureTask::captureComplete, this,
            &PhotoCaptureWindow::onCaptureComplete, Qt::QueuedConnection);

    m_rsManager->captureInfraPhotos();
}

void PhotoCaptureWindow::appendLog(const QString& message) { m_logTextEdit->append(message); }

void PhotoCaptureWindow::onFrameReceived(std::vector<std::tuple<int, std::string, QImage, rs2::depth_frame>> frames) {
    if (m_start_btn->isEnabled() || frames.size() == 0) return;

    std::for_each(frames.begin(), frames.end(), [this](const auto& frameInfo) {
        int cameraId = std::get<0>(frameInfo);
        const QString& serial = QString::fromStdString(std::get<1>(frameInfo));
        const QImage& img = std::get<2>(frameInfo);
        const auto& depth_frame_ptr = std::get<3>(frameInfo);

        if (cameraId < 100 || cameraId >= 200) {
            if (!m_cameraWidgets.contains(cameraId)) {
                appendLog(QString("Received %1 frame from camera %2 (Serial: %3)")
                              .arg(cameraId < 100 ? "color" : "infrared")
                              .arg(cameraId)
                              .arg(serial));

                CameraWidget* widget = new CameraWidget(this);
                widget->setMinimumSize(320, 240);

                int count = m_cameraWidgets.size();
                int row = count / 2;
                int col = count % 2;
                m_camera_container->addWidget(widget, row, col);

                m_cameraWidgets.insert(std::get<0>(frameInfo), widget);
            } else {
                m_cameraWidgets[cameraId]->updateFrame(img);
            }
        } else if (depth_frame_ptr) {
            // Handle depth frame
        }

        // if (m_currentCaptureTask) {
        //     m_currentCaptureTask->capture_frames(serial.toStdString(), img,
        //                                          img);  // Using the same image for both IR channels for simplicity
        // }
    });
}

void PhotoCaptureWindow::onCaptureComplete(bool success, const QString& message) {
    if (success) {
        appendLog(message);
    } else {
        appendLog("Error: " + message);
    }
    m_currentCaptureTask.reset();
}

void PhotoCaptureWindow::onCameraError(const QString& err) { appendLog(err); }
