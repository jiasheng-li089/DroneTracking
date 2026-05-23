#pragma once

#include <QGridLayout>
#include <QMainWindow>
#include <QMap>
#include <QPushButton>
#include <QTextEdit>
#include <memory>
#include <opencv2/core/mat.hpp>

#include "../camera/CameraManager.h"
#include "../media/PhotoCaptureTask.h"

class CameraWidget;
class RealSenseManager;

class PhotoCaptureWindow : public QMainWindow {
  Q_OBJECT

public:
  explicit PhotoCaptureWindow(std::string target_dir,
                              QWidget *parent = nullptr);
  ~PhotoCaptureWindow() override;

private slots:
  void on_start();
  void on_stop();

  void on_capture_photos();
  void on_finalize();
  void append_log(const QString &message);

  void on_finalize_complete(bool success, const QString &message);
  void on_update_capture_status(bool capturing);

  void on_frame_received(const int, const std::string &, const ulong,
                         const cv::Mat);
  void on_camera_error(const QString &err);

private:
  void setup_ui();

  std::unique_ptr<CameraManager::CameraManager> m_camera_manager;
  std::unique_ptr<media::PhotoCaptureTask> m_capturer;

  QTextEdit *m_log_te{};
  QWidget *m_start_btn{};
  QWidget *m_stop_btn{};
  QWidget *m_capture_btn{};
  QWidget *m_finalize_btn{};

  QGridLayout *m_camera_container{};

  QMap<int, CameraWidget *> m_camera_widgets;

  std::set<std::string> m_camera_serials;
};
