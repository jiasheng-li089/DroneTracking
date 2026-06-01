#include "AutoPhotoCaptureWindow.h"
#include <functional>
#include <memory>
#include <opencv2/core.hpp>
#include <qboxlayout.h>
#include <qfont.h>
#include <qimage.h>
#include <qlabel.h>
#include <qmainwindow.h>
#include <qobject.h>
#include <qpushbutton.h>
#include <qtimer.h>
#include <qwidget.h>

#include "../media/AutoPhotoCapturer.h"
#include "camera/CameraManager.h"
#include "config.h"
#include "gui/CameraWidget.h"
#include "gui/PhotoCaptureWindow.h"

#ifdef USE_REALSENSE_CAMERA
#include "../camera/RealSenseManager.h"
#else
#include "../camera/GenericCamerManager.h"
#endif

AutoPhotoCaptureWindow::AutoPhotoCaptureWindow(
    AutoPhotoCaptureWindowConfig &config, QWidget *parent)
    : QMainWindow(parent) {
#ifdef USE_REALSENSE_CAMERA
  m_camera_manager = std::make_unique<RealSenseManager>();
#else
  m_camera_manager = std::make_unique<CameraManager::GenericCameraManager>();
#endif
  // TODO
  m_photo_capturer = std::make_unique<media::AutoPhotoCapturer>(
      config.camera_serial, config.chessboard_size, config.square_size_mm, true,
      config.target_dir, config.prefix, this);
  setup_ui();

  m_camera_manager->set_frame_callback(
      std::bind(&media::AutoPhotoCapturer::handle_frame, m_photo_capturer.get(),
                std::placeholders::_1, std::placeholders::_2,
                std::placeholders ::_3, std::placeholders ::_4));
  connect(m_photo_capturer.get(), &media::AutoPhotoCapturer::received_frame,
          this, &AutoPhotoCaptureWindow::on_frame_received);

  connect(m_start_btn, &QPushButton::clicked, this,
          &AutoPhotoCaptureWindow::on_start);
  connect(m_stop_btn, &QPushButton::clicked, this,
          &AutoPhotoCaptureWindow::on_stop);

  m_timer = new QTimer(this);
  connect(m_timer, &QTimer::timeout, [this]() {
    if (flag == 0) {
      m_photo_capturer->update_capture_status(true);
    } else {
      update_status_text("Waiting", "black", 30);
      flag = 0;
      m_timer->start(4000);
    }
  });
}

AutoPhotoCaptureWindow::~AutoPhotoCaptureWindow() = default;

void AutoPhotoCaptureWindow::setup_ui() {
  setWindowTitle("Photo Capturer");
  resize(1600, 1200);

  auto root_widget = new QWidget(this);
  auto layout = new QVBoxLayout(root_widget);

  auto container = new QWidget(root_widget);
  auto button_layout = new QHBoxLayout(container);
  container->setMaximumHeight(300);
  m_start_btn = new QPushButton("Start Capturing", container);
  m_stop_btn = new QPushButton("Stop Capturing", container);
  m_status_label = new QLabel("Waiting", container);
  update_status_text("Waiting", "black", 30);

  button_layout->addWidget(m_start_btn);
  button_layout->addWidget(m_stop_btn);
  button_layout->addWidget(m_status_label);

  layout->addWidget(container);

  m_preview_widget = new CameraWidget(this);
  layout->addWidget(m_preview_widget);

  m_start_btn->setEnabled(true);
  m_stop_btn->setEnabled(false);

  this->setCentralWidget(root_widget);
}

void AutoPhotoCaptureWindow::on_frame_received(int camera_id,
                                               std::string serial,
                                               ulong frame_id, cv::Mat frame,
                                               bool captured) {
  cv::flip(frame, m_flip_frame, 1);
  QImage img((const uchar *)m_flip_frame.data, frame.cols, frame.rows, frame.step,
             QImage::Format_RGB888);
  m_preview_widget->update_frame(img.copy());
  if (captured) {
    update_status_text("Captured", "red", 30);
    flag = 1;
    m_timer->start(1000);
  }
}

void AutoPhotoCaptureWindow::on_start() {
  m_camera_manager->start_cameras();
  m_start_btn->setEnabled(false);
  m_photo_capturer->start_capture();
  m_stop_btn->setEnabled(true);
  flag = 0;
  m_timer->start(5000);
}

void AutoPhotoCaptureWindow::on_stop() {
  m_timer->stop();
  m_stop_btn->setEnabled(false);
  m_photo_capturer->stop_capture();
  m_start_btn->setEnabled(true);

  m_camera_manager->stop_cameras();
}

void AutoPhotoCaptureWindow::update_status_text(std::string txt, std::string color, int size) {
  m_status_label->setText(QString::fromStdString(txt));
  m_status_label->setStyleSheet(QString("QLabel { color: %1; font-size: %2pt }").arg(QString::fromStdString(color)).arg(size));
}