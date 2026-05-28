#include "camera_util.h"

#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>

#include <fstream>
#include <string>
#include <vector>

#include <fcntl.h>
#include <unistd.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>

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

std::string get_camera_serial(int camera_id) {
  std::string camera_class_path = "/sys/class/video4linux/video" + std::to_string(camera_id) + "/device/../serial";
  std::ifstream serial_file(camera_class_path);
  if (serial_file.is_open()) {
    std::string serial;
    std::getline(serial_file, serial);
    serial_file.close();
    return serial;
  }
  return "";
}

CameraMeta get_camera_meta(int camera_id) {
  std::string camera_path = "/dev/video" + std::to_string(camera_id);
  int fd = open(camera_path.c_str(), O_RDONLY);
  if (-1 == fd) {
    return CameraMeta{0, camera_id, "", {}};
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
      while (ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &frmival) == 0) {
        if (frmival.type == V4L2_FRMIVAL_TYPE_DISCRETE) {
          double fps = static_cast<double>(frmival.discrete.denominator) /
                       static_cast<double>(frmival.discrete.numerator);
          tmp.fps.push_back(fps);
        }
        frmival.index ++;
      }
      std::sort(tmp.fps.begin(), tmp.fps.end(), std::greater<double>());

      supported_resolutions.push_back(tmp);
    }
    frmsize.index ++;
  }
  close(fd);
  std::sort(supported_resolutions.begin(), supported_resolutions.end(),
            [](const CameraResolution &a, const CameraResolution &b) {
              if (a.width != b.width) {
                return a.width > b.width;
              }
              return a.height > b.height;
            });

  // read serial from camera

  return CameraMeta{1, camera_id, get_camera_serial(camera_id), supported_resolutions};
}

std::vector<CameraMeta> get_camera_metas(std::vector<int> camera_ids) {
  std::vector<CameraMeta> all_meta;
  for (int id : camera_ids) {
    all_meta.push_back(get_camera_meta(id));
  }
  return std::move(all_meta);
}

std::vector<CameraMeta> get_all_camera_metas() {
  auto camera_ids = detect_rgb_cameras();
  return get_camera_metas(camera_ids);
}