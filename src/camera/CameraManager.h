#pragma once

#include <QObject>
#include <map>
#include <opencv2/core/mat.hpp>
#include <qtypes.h>
#include <string>
#include <thread>

namespace CameraManager {

class CameraManager : public QObject {
  Q_OBJECT
public:
  explicit CameraManager(QObject *parent = nullptr);
  ~CameraManager() override = default;

  virtual void start_cameras();
  virtual void stop_cameras();

  void set_frame_callback(
      std::function<void(const int, const std::string &, ulong, cv::Mat)>
          callback);

signals:
  void frame_received(int, const std::string &, ulong, const cv::Mat);
  void error_occurred(QString err);

protected:
  std::map<std::string, std::unique_ptr<std::thread>> m_threads;
  std::atomic<bool> m_running;
  std::function<void(const int, const std::string &, ulong, cv::Mat)>
      m_callback;
};
} // namespace CameraManager