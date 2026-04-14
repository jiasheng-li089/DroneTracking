//
// Created by jason on 14/04/26.
//

#ifndef DRONETRACKING_PHOTOCAPTURETASK_H
#define DRONETRACKING_PHOTOCAPTURETASK_H
#include <QImage>
#include <set>
#include <string>

namespace media {

class PhotoCaptureTask {
public:
  PhotoCaptureTask(int camera_count, std::string target_dir,
                   std::string prefix);

  ~PhotoCaptureTask();

  bool capture_frame(std::string serial, QImage image);

private:
  int camera_count = 0;
  std::string target_dir;
  std::string file_name_prefix;
  std::set<std::string> camera_ids;

  bool is_capturing;

  void save_video_frame(std::string serial, QImage image);
};

} // namespace media

#endif // DRONETRACKING_PHOTOCAPTURETASK_H
