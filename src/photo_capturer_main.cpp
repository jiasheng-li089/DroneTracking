#include "camera/camera_util.h"
#include "gui/AutoPhotoCaptureWindow.h"
#include "logger.h"

#include <QApplication>
#include <QDir>
#include <QSurfaceFormat> // Required for configuring OpenGL Widget
#include <algorithm>
#include <cctype>
#include <iostream>
#include <opencv2/core/types.hpp>
#include <qdir.h>
#include <string>

int main(int argc, char *argv[]) {
  Logger::init("logs/photo_capturing.log");
  // QSurfaceFormat must be set BEFORE creating the QApplication instance.
  QSurfaceFormat format;
  format.setVersion(3, 3);
  format.setProfile(QSurfaceFormat::CoreProfile);
  QSurfaceFormat::setDefaultFormat(format);

  QApplication a(argc, argv);

  QDir current = QDir::current();
  if (!current.exists("photos")) {
    current.mkdir("photos");
  }
  current.cd("photos");

  auto camera_metas = get_all_camera_metas();
  std::string camera_serial = "";

  for (const auto &meta : camera_metas) {
    auto serial = meta.serial;
    std::transform(serial.begin(), serial.end(), serial.begin(), ::tolower);
    auto non_zero_index = serial.find_first_not_of('0');
    if (non_zero_index != std::string::npos) {
      serial = serial.substr(non_zero_index);
    }

    if (meta.supported_resolutions.size() <= 0 ||
        meta.supported_resolutions[0].fps.size() <= 0) {
      continue;
    }

    if (!camera_serial.empty()) {
        std::cerr << "Too many cameras are installed into the device" <<std::endl;
        return -1;
    }
    camera_serial = serial;
  }

  if (camera_serial.empty()) {
    std::cerr << "Failed to find a camera" << std::endl;
    return -1;
  } else {
    std::cout << "Start photo capturing with camera #" << camera_serial << std::endl;
  }

  AutoPhotoCaptureWindowConfig config{camera_serial, cv::Size(10, 7), 25.0f,
                                      current.absolutePath().toStdString(),
                                      camera_serial};

  AutoPhotoCaptureWindow w(config, nullptr);
  w.show();

  return a.exec();
}
