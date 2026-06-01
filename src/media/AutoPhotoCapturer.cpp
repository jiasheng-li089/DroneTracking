#include "AutoPhotoCapturer.h"
#include <iostream>
#include <memory>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <qobject.h>
#include <qtmetamacros.h>
#include <string>
#include <thread>
#include <vector>

namespace media {

AutoPhotoCapturer::~AutoPhotoCapturer() {}

AutoPhotoCapturer::AutoPhotoCapturer(std::string camera_serial,
                                     cv::Size chessboard_size,
                                     float square_size_mm, bool should_reset,
                                     std::string target_dir, std::string prefix,
                                     QObject *parent)
    : m_reset_after_captured(should_reset), m_should_capture(false),
      m_target_dir(target_dir), m_file_prerix(prefix),
      m_camera_serial(camera_serial), QObject(parent),
      m_chessboard_size(chessboard_size), m_square_size_mm(square_size_mm) {
  m_running = false;
}

void AutoPhotoCapturer::start_capture() {
  if (m_running) {
    return;
  }
  m_file_names.clear();
  m_running = true;
  m_should_capture = false;
}

void AutoPhotoCapturer::stop_capture() {
  if (!m_running) {
    return;
  }

  if (m_last_save_thread && m_last_save_thread->joinable()) {
    m_last_save_thread->join();
  }

  // save the result
  save_configuration();
}

void AutoPhotoCapturer::update_capture_status(bool enable) {
  m_should_capture = enable;
}

void AutoPhotoCapturer::handle_frame(int camera_id, std::string serial,
                                     ulong frame_id, cv::Mat frame) {
  if (serial != m_camera_serial) {
    return;
  }

  std::vector<cv::Point2f> points;
  bool save_current_frame = false;
  // show the detected chessboard in the picture
  cv::cvtColor(frame, m_processed_frame, cv::COLOR_RGB2BGR);
  cv::cvtColor(m_processed_frame, m_gray_frame, cv::COLOR_BGR2GRAY);
  if (cv::findChessboardCorners(m_gray_frame, m_chessboard_size, points,
                                cv::CALIB_CB_ADAPTIVE_THRESH |
                                    cv::CALIB_CB_NORMALIZE_IMAGE |
                                    cv::CALIB_CB_FAST_CHECK)) {
    if (m_should_capture) {
      save_current_frame = true;
      if (m_reset_after_captured) {
        m_should_capture = false;
      }
    }

    cv::cornerSubPix(
        m_gray_frame, points, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30,
                         0.1));

    cv::drawChessboardCorners(m_processed_frame, m_chessboard_size, points,
                              true);
    emit received_frame(camera_id, serial, frame_id, m_processed_frame.clone(),
                        save_current_frame);
  } else {
    emit received_frame(camera_id, serial, frame_id, m_processed_frame.clone(), false);
  }

  // TODO decide if to save this frame
  if (save_current_frame) {
    m_last_save_thread =
        std::make_unique<std::thread>(&AutoPhotoCapturer::save_photo, this,
                                      frame_id, frame.clone());
    m_last_save_thread->detach();
  }
}

void AutoPhotoCapturer::save_photo(ulong frame_id, cv::Mat frame) {
  cv::Mat normal_color_frame;
  cv::cvtColor(frame, normal_color_frame, cv::COLOR_RGB2BGR);
  std::string filename = m_target_dir;
  if (!filename.empty() && filename.back() != '/') {
    filename += "/";
  }
  filename += m_file_prerix + "_" + m_camera_serial + "_" +
              std::to_string(frame_id) + ".png";
  cv::imwrite(filename, normal_color_frame);
  m_file_names.push_back(filename);
}

void AutoPhotoCapturer::save_configuration() {
  std::string filename = m_target_dir;
  if (!filename.empty() && filename.back() != '/') {
    filename += "/";
  }
  filename += m_file_prerix + "_config.yaml";

  cv::FileStorage fs(filename, cv::FileStorage::WRITE);
  if (!fs.isOpened()) {
    std::cerr << "Cannot open output file: " << filename << std::endl;
    return;
  }

  fs << "cameras" << "{";
  {
    fs << ("camera_" + m_camera_serial) << "{";
    {
      fs << "pattern_cols" << m_chessboard_size.width;
      fs << "pattern_rows" << m_chessboard_size.height;
      fs << "square_size_mm" << m_square_size_mm;
      fs << "photos" << "[";
      for (const auto &file_name : m_file_names) {
        fs << file_name;
      }
      fs << "]";
    }
    fs << "}";
  }
  fs << "}";
}

} // namespace media