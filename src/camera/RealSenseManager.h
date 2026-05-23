#pragma once
#include <QImage>
#include <QObject>
#include <string>

#include <librealsense2/rs.hpp>

#include <spdlog/spdlog.h>

#include "CameraManager.h"

class RealSenseManager : public CameraManager::CameraManager {
public:
  explicit RealSenseManager(QObject *parent = nullptr);
  ~RealSenseManager() override;

  void start_cameras() override;
  void stop_cameras() override;

private:
  void camera_worker_thread(int cameraId, std::string serial);
};