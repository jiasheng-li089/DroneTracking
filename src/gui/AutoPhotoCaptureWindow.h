#pragma once

#include "camera/CameraManager.h"
#include "gui/CameraWidget.h"
#include "media/AutoPhotoCapturer.h"
#include <QObject>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <qlabel.h>
#include <qmainwindow.h>
#include <qobject.h>
#include <qpushbutton.h>
#include <qtimer.h>
#include <qtmetamacros.h>
#include <qwidget.h>
#include <string>


struct AutoPhotoCaptureWindowConfig {
  std::string camera_serial;
  cv::Size chessboard_size;
  float square_size_mm;
  std::string target_dir;
  std::string prefix;
};

class AutoPhotoCaptureWindow : public QMainWindow {
  Q_OBJECT

public:
  explicit AutoPhotoCaptureWindow(AutoPhotoCaptureWindowConfig &config,
                                  QWidget *parent = nullptr);
  ~AutoPhotoCaptureWindow() override;

private slots:

  void on_start();
  void on_stop();

  void on_frame_received(int, std::string, ulong, cv::Mat, bool);

private:
  void setup_ui();
  void update_status_text(std::string, std::string, int);

  QPushButton *m_start_btn;
  QPushButton *m_stop_btn;
  QLabel *m_status_label;
  CameraWidget *m_preview_widget{};
  QTimer *m_timer;

  int flag = 0;

  std::string m_config_file;

  cv::Mat m_flip_frame;

  std::unique_ptr<CameraManager::CameraManager> m_camera_manager;
  std::unique_ptr<media::AutoPhotoCapturer> m_photo_capturer;

};