#include "camera/camera_util.h"
#include "common/common.h"
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>
#include <vector>

int main(int argc, char *argv[]) {
  spdlog::info("Hello World: {}", current_timestamp_ms());
  auto rgb_cameras = detect_rgb_cameras();
  spdlog::info("Found {} RGB cameras", rgb_cameras.size());
  for (auto &camera : rgb_cameras) {
    spdlog::info("Camera: {}", camera);
  }
  return 0;
}