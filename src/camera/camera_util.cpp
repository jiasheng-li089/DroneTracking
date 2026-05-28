#include "camera_util.h"

#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>
#include <sys/ioctl.h>

#if defined(__linux__)
#include <fcntl.h>
#include <linux/videodev2.h>
#include <unistd.h>
#else
#endif

std::vector<int> detect_rgb_cameras() {
  std::vector<int> rgb_cameras;

#if defined(__linux__)
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
#else
  // for macOS and Windows, fallback: brute force open and check for 3 channels
  for (int i = 0; i < 10; i++) {
    cv::VideoCapture cap(i);
    if (cap.isOpened()) {
      cv::Mat frame;
      cap >> frame;
      if (!frame.empty() && frame.channels() == 3) {
        rgb_cameras.push_back(i);
      }
    }
  }
#endif

  return rgb_cameras;
}
