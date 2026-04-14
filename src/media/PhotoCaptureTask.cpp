//
// Created by jason on 14/04/26.
//

#include "PhotoCaptureTask.h"

#include <thread>

namespace media {
PhotoCaptureTask::PhotoCaptureTask(int camera_count, std::string target_dir,
                                   std::string prefix)
    : camera_count(camera_count), target_dir(target_dir),
      file_name_prefix(prefix), is_capturing(true) {}

PhotoCaptureTask::~PhotoCaptureTask() = default;

bool PhotoCaptureTask::capture_frame(std::string serial, QImage image) {
  if (const auto search = camera_ids.find(serial); search != camera_ids.end()) {
    // found
    return is_capturing;
  }
  return true;
}

void PhotoCaptureTask::save_video_frame(std::string serial, QImage image) {}
} // namespace media
