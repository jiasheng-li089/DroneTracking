#include "GenericCamerManager.h"

#include <QObject>
#include <spdlog/spdlog.h>

namespace CameraManager {

GenericCameraManager::GenericCameraManager(QObject *parent)
    : CameraManager(parent) {}

void GenericCameraManager::start_cameras() {
  spdlog::info("Starting GenericCameraManager...");
}

void GenericCameraManager::stop_cameras() {
  spdlog::info("Stopping GenericCameraManager...");
}
} // namespace CameraManager