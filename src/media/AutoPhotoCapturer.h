#pragma once

#include <atomic>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <qobject.h>
#include <qtmetamacros.h>
#include <string>
#include <thread>

namespace media {

class AutoPhotoCapturer : public QObject {
  Q_OBJECT

public:
  explicit AutoPhotoCapturer(std::string serial, cv::Size chessboard_size,
                             float square_size_mm, bool should_reset,
                             std::string target_dir, std::string prefix,
                             QObject *parent = nullptr);
  ~AutoPhotoCapturer() override;

  void start_capture();

  void stop_capture();

  void update_capture_status(bool);

  void handle_frame(int, std::string, ulong, cv::Mat);

signals:
  void received_frame(int, std::string, ulong, cv::Mat, bool);

private:
  const std::string m_camera_serial;
  const std::string m_target_dir;
  const std::string m_file_prerix;

  std::atomic<bool> m_reset_after_captured;
  std::atomic<bool> m_should_capture;
  bool m_running;
  cv::Mat m_processed_frame;
  cv::Mat m_gray_frame;
  cv::Size m_chessboard_size;
  float m_square_size_mm;

  std::vector<std::string> m_file_names;

  std::unique_ptr<std::thread> m_last_save_thread;

  void capture_thread();

  void save_photo(ulong, cv::Mat);

  void save_configuration();
};
} // namespace media