#include "PhotoCaptureWindow.h"

#include "camera/CameraManager.h"
#include "config.h"
#include <opencv2/core/mat.hpp>

#ifdef USE_REALSENSE_CAMERA
#include "../camera/RealSenseManager.h"
#else
#endif

#include <QDateTime>
#include <QWidget>

#include "../camera/RealSenseManager.h"
#include "CameraWidget.h"

PhotoCaptureWindow::PhotoCaptureWindow(std::string target_dir, QWidget *parent)
    : QMainWindow(parent),
#ifdef USE_REALSENSE_CAMERA
      m_camera_manager(std::make_unique<RealSenseManager>()),
#else
      m_camera_manager(std::make_unique<CameraManager::CameraManager>()),
#endif
      m_capturer(std::make_unique<media::PhotoCaptureTask>(target_dir)) {
  setup_ui();

  connect(m_camera_manager.get(), &CameraManager::CameraManager::frame_received,
          this, &PhotoCaptureWindow::on_frame_received);
  connect(m_camera_manager.get(), &CameraManager::CameraManager::error_occurred,
          this, &PhotoCaptureWindow::on_camera_error);

  connect(m_capturer.get(), &media::PhotoCaptureTask::finalize_complete, this,
          &PhotoCaptureWindow::on_finalize_complete);
  connect(m_capturer.get(), &media::PhotoCaptureTask::update_capture_status,
          this, &PhotoCaptureWindow::on_update_capture_status);
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

  connect(reinterpret_cast<QPushButton *>(m_capture_btn), &QPushButton::clicked,
          this, &PhotoCaptureWindow::on_capture_photos);
  connect(reinterpret_cast<QPushButton *>(m_start_btn), &QPushButton::clicked,
          this, &PhotoCaptureWindow::on_start);
  connect(reinterpret_cast<QPushButton *>(m_stop_btn), &QPushButton::clicked,
          this, &PhotoCaptureWindow::on_stop);

  connect(reinterpret_cast<QPushButton *>(m_capture_btn), &QPushButton::clicked,
          this, &PhotoCaptureWindow::on_capture_photos);
  connect(reinterpret_cast<QPushButton *>(m_finalize_btn),
          &QPushButton::clicked, this, &PhotoCaptureWindow::on_finalize);

  m_start_btn->setEnabled(true);
  m_stop_btn->setEnabled(false);
  m_capture_btn->setEnabled(false);
}

void PhotoCaptureWindow::on_start() {
  append_log("Start showing video from cameras");

  m_camera_manager->start_cameras();
  m_start_btn->setEnabled(false);
  m_stop_btn->setEnabled(true);
  m_capture_btn->setEnabled(true);
}

void PhotoCaptureWindow::on_stop() {
  append_log("Stop showing video from cameras");

  m_camera_manager->stop_cameras();

  append_log("Stop successfully");

  QLayoutItem *child;
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

void PhotoCaptureWindow::append_log(const QString &message) {
  m_log_te->append(message);
}

void PhotoCaptureWindow::on_finalize_complete(bool success,
                                              const QString &message) {
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

void PhotoCaptureWindow::on_frame_received(const int camera_id,
                                           const std::string &camera_serial,
                                           const ulong frame_id,
                                           const cv::Mat frame) {
  if (m_start_btn->isEnabled())
    return;

  const QString serial = QString::fromStdString(camera_serial);
  QImage img((const uchar *)frame.data, frame.cols, frame.rows, frame.step,
             QImage::Format_RGB888);
  img = img.copy();

  if (!m_camera_widgets.contains(camera_id)) {
    append_log(QString("Received frame from camera %1 (Serial: %2)")
                   .arg(camera_id)
                   .arg(camera_serial));

    CameraWidget *widget = new CameraWidget(this);
    widget->setMinimumSize(320, 240);

    int count = m_camera_widgets.size();
    int row = count / 2;
    int col = count % 2;
    m_camera_container->addWidget(widget, row, col);
    widget->update_frame(img);

    m_camera_widgets.insert(camera_id, widget);
  } else {
    m_camera_widgets[camera_id]->update_frame(img);
  }
}

void PhotoCaptureWindow::on_camera_error(const QString &err) {
  append_log(err);
}
