#include "camera/camera_util.h"
#include "common/common.h"
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>
#include <vector>

int main(int argc, char *argv[]) {
  spdlog::info("Hello World: {}", current_timestamp_ms());

  std::vector<CameraMeta> camera_metas = get_all_camera_metas();
  for (const auto &meta : camera_metas) {
    if (meta.supported_resolutions.size() == 0) {
      continue;
    }
    spdlog::info("Camera {}: serial: {}, supported resolutions:", meta.id, meta.serial);
    for (const auto &res : meta.supported_resolutions) {
      std::string fps_str = "";
      for (const auto &fps : res.fps) {
        fps_str.append(std::to_string(static_cast<int>(fps)) + ", ");
      }
      spdlog::info("  {}x{}, supported fps: {}", res.width, res.height, fps_str);
    }
  }
  return 0;
}