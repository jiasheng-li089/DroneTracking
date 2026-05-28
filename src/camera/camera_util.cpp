#include "camera_util.h"

#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>
#include <string>
#include <sys/ioctl.h>

#include <fcntl.h>
#include <linux/videodev2.h>
#include <unistd.h>
#include <vector>

std::vector<int> detect_rgb_cameras() {
  std::vector<int> rgb_cameras;

  for (int i = 0; i < 30; i++) {
    std::string camera_path = "/dev/video" + std::to_string(i);
    int fd = open(camera_path.c_str(), O_RDWR);
    if (fd == -1) {
      continue;
    }
    v4l2_capability cap;
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0) {
      std::string name = (const char *)cap.card;
      std::string bus = (const char *)cap.bus_info;
      spdlog::info("Found camera {}: {} on bus: {}", i, name, bus);

      if (cap.device_caps & V4L2_CAP_VIDEO_CAPTURE) {
        rgb_cameras.push_back(i);
      }
    }

    close(fd);
  }

  return std::move(rgb_cameras);
}

CameraMeta get_camera_meta(int camera_id) {
  std::string camera_path = "/dev/video" + std::to_string(camera_id);
  int fd = open(camera_path.c_str(), O_RDONLY);
  if (-1 == fd) {
    return CameraMeta{-1, camera_id, "", {}};
  }

  struct v4l2_frmsizeenum frmsize;
  frmsize.index = 0;

  frmsize.pixel_format = V4L2_PIX_FMT_YUYV;

  std::vector<CameraResolution> supported_resolutions;
  while (ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frmsize) == 0) {
    if (frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
      struct v4l2_frmivalenum frmival = {};
      frmival.index = 0;
      frmival.pixel_format = frmsize.pixel_format;
      frmival.width = frmsize.discrete.width;
      frmival.height = frmsize.discrete.height;

      CameraResolution tmp {
        static_cast<int>(frmsize.discrete.width), 
        static_cast<int>(frmsize.discrete.height),
        {}
      };
      while (ioctl(fd, V4L2_FRMIVAL_TYPE_DISCRETE) == 0) {
        if (frmival.type == V4L2_FRMIVAL_TYPE_DISCRETE) {
          double fps = static_cast<double>(frmival.discrete.denominator) /
                       static_cast<double>(frmival.discrete.numerator);
          tmp.fps.push_back(fps);
        }
        frmival.index ++;
      }

      supported_resolutions.push_back(tmp);
    }
    frmsize.index ++;
  }
  close(fd);

  // read serial from camera

  return CameraMeta{-1, camera_id, "", {}};
}