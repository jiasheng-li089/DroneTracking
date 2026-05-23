#include "DroneTrackingWindow.h"

#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <qhashfunctions.h>
#include <qimage.h>
#include <qobject.h>
#include <spdlog/spdlog.h>

#include "camera/CameraManager.h"
#include "config.h"

#include <QBoxLayout>
#include <QPushButton>

#ifdef USE_REALSENSE_CAMERA
#include "../camera/RealSenseManager.h"
#else
#include "../camera/GenericCamerManager.h"
#endif
#include "../common/common.h"
#include "../network/WebRtcManager.h"
#include "../network/WebSocketSignaling.h"
#include "../tracking/TrackerConfig.h"
#include "../tracking/VisionTracker.h"
#include "CameraWidget.h"

DroneTrackingWindow::DroneTrackingWindow(std::string config_file,
                                         QWidget *parent)
    : QMainWindow(parent), m_config_file(std::move(config_file)),
#ifdef USE_REALSENSE_CAMERA
      m_camera_manager(std::make_unique<RealSenseManager>()) {
#else
      m_camera_manager(
          std::make_unique<CameraManager::GenericCameraManager>()) {
#endif
  m_webrtc_manager = std::make_unique<WebRtcManager>(
      std::make_unique<WebSocketSignaling>(WEBSOCKET_URL, "janus-protocol"));
  m_vision_tracker = std::make_unique<tracking::VisionTracker>(
      std::make_shared<tracking::TrackerConfig>(m_config_file));

  setup_ui();

  connect(this, &DroneTrackingWindow::update_widget_status, this,
          &DroneTrackingWindow::on_widget_status_update);
  connect(this, &DroneTrackingWindow::append_log, m_log_te, &QTextEdit::append);
  connect(m_webrtc_manager.get(), &WebRtcManager::on_connection_state, this,
          &DroneTrackingWindow::on_webrtc_connection_state);

  m_ping_timer = new QTimer(this);
  connect(m_ping_timer, &QTimer::timeout, this, [this]() {
    if (m_webrtc_manager) {
      m_webrtc_manager->sendMessage("Ping", "");
    }
  });

  connect(m_camera_manager.get(), &CameraManager::CameraManager::frame_received,
          this, &DroneTrackingWindow::frame_received);
  connect(m_camera_manager.get(), &CameraManager::CameraManager::error_occurred,
          this, &DroneTrackingWindow::error_occurred);

  connect(m_vision_tracker.get(), &tracking::VisionTracker::error_occurred,
          this, &DroneTrackingWindow::error_occurred);
  connect(m_vision_tracker.get(), &tracking::VisionTracker::frame_received,
          this, &DroneTrackingWindow::frame_received);
  connect(m_vision_tracker.get(), &tracking::VisionTracker::publish_message,
          m_webrtc_manager.get(), &WebRtcManager::publish_pose);
  connect(m_vision_tracker.get(),
          &tracking::VisionTracker::update_camera_status, this,
          &DroneTrackingWindow::on_update_camera_status);
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

  std::vector<QWidget *> buttons = {m_start_camera_btn, m_stop_camera_btn,
                                    m_start_tracking_btn, m_stop_tracking_btn};
  auto button_id = 0;
  for (auto *btn : buttons) {
    btn->setProperty("button_id", button_id++);

    button_layout->addWidget(btn);
    connect(reinterpret_cast<QPushButton *>(btn), &QPushButton::clicked, this,
            [this, btn]() { on_button_clicked(btn); });

    btn->setEnabled(button_id % 2 == 1);
  }

#ifdef DEBUG_CHANNEL
  auto debug_channel_btn = new QPushButton("Send Debug Message", root_widget);
  button_layout->addWidget(debug_channel_btn);
  connect(debug_channel_btn, &QPushButton::clicked, this, [this]() {
    tracking::ObjectPose test_pose =
        tracking::ObjectPose::from_t_and_r(cv::Vec3d(), cv::Vec3d());
    m_webrtc_manager->sendMessage("Pose", test_pose.to_json());
  });
#endif

  layout->addLayout(button_layout);

  // camera widget container
  auto camera_widget = new QWidget(root_widget);
  m_camera_container = new QGridLayout(camera_widget);
  layout->addWidget(camera_widget);

  // status label container
  auto label_widget = new QWidget(root_widget);
  m_labels_container = new QGridLayout(label_widget);
  label_widget->setFixedHeight(100);
  layout->addWidget(label_widget);

  // log text edit
  m_log_te = new QTextEdit(root_widget);
  m_log_te->setMaximumHeight(150);
  m_log_te->setReadOnly(true);
  layout->addWidget(m_log_te);

  this->setCentralWidget(root_widget);
}

void DroneTrackingWindow::on_update_camera_status(std::string serial,
                                                  std::string status) {
  QString q_serial = QString::fromStdString(serial);
  QString q_status =
      QString("%1: %2").arg(q_serial).arg(QString::fromStdString(status));

  if (!m_camera_labels.contains(serial)) {
    QLabel *label = new QLabel(this);
    label->setText(q_status);
    int count = m_camera_labels.size();
    int row = count / 2;
    int col = count % 2;
    m_labels_container->addWidget(label, row, col);
    m_camera_labels[serial] = label;
  } else {
    m_camera_labels[serial]->setText(q_status);
  }
}

void DroneTrackingWindow::on_widget_status_update(QWidget *sender,
                                                  bool enable) {
  sender->setEnabled(enable);
}

void DroneTrackingWindow::on_button_clicked(QWidget *sender) {
  switch (sender->property("button_id").toInt()) {
  case 0:
    m_log_te->append("Start Camera button clicked");
    m_camera_manager->start_cameras();
    on_widget_status_update(m_start_camera_btn, false);
    on_widget_status_update(m_stop_camera_btn, true);
    break;
  case 1:
    m_log_te->append("Stop Camera button clicked");
    m_camera_manager->stop_cameras();
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

    m_camera_manager->set_frame_callback(std::bind(
        &tracking::VisionTracker::process_frames, m_vision_tracker.get(),
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4));
    m_ping_timer->start(3000);
  } else {
    m_log_te->append("WebRTC connection lost");
    on_widget_status_update(m_start_tracking_btn, true);
    on_widget_status_update(m_stop_tracking_btn, false);
    m_camera_manager->set_frame_callback(nullptr);
    m_ping_timer->stop();
  }
}

void DroneTrackingWindow::start_tracking() {
  m_log_te->append("Starting tracking...");

  on_widget_status_update(m_start_tracking_btn, false);
  on_widget_status_update(m_stop_tracking_btn, false);

  std::thread([this]() {
    try {
      m_webrtc_manager->connect();
    } catch (const std::exception &ex) {
      emit append_log(QString("Error starting tracking: ") +
                      QString::fromStdString(ex.what()));
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
    } catch (const std::exception &ex) {
      emit append_log(QString("Error stopping tracking: ") +
                      QString::fromStdString(ex.what()));
    }

    emit update_widget_status(m_start_tracking_btn, true);
    emit update_widget_status(m_stop_tracking_btn, false);
  }).detach();
}

void DroneTrackingWindow::frame_received(const int camera_id,
                                         const std::string &camera_serial,
                                         const ulong frame_id,
                                         const cv::Mat &frame) {
  if (m_start_camera_btn->isEnabled())
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

void DroneTrackingWindow::error_occurred(const QString &err) {
  m_log_te->append(err);
}
