#include "CameraManager.h"
#include <QDebug>

namespace CameraManager {

CameraManager::CameraManager(QObject *parent)
    : QObject(parent), m_running(false) {}

void CameraManager::set_frame_callback(
    std::function<void(const int, const std::string &, ulong, cv::Mat)>
        callback) {
  m_callback = callback;
}

void CameraManager::start_cameras() {}
void CameraManager::stop_cameras() {}

} // namespace CameraManager