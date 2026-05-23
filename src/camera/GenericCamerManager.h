#pragma once
#include "CameraManager.h"
#include <QObject>

namespace CameraManager {

class GenericCameraManager : public CameraManager {
public:
  explicit GenericCameraManager(QObject *parent = nullptr);
  ~GenericCameraManager() override = default;

  void start_cameras() override;
  void stop_cameras() override;
};

} // namespace CameraManager