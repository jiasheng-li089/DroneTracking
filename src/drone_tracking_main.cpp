#include <stdlib.h>

#include <QApplication>
#include <QDir>

#include "gui/DroneTrackingWindow.h"
#include "logger.h"
#include <spdlog/spdlog.h>

int main(int argc, char *argv[]) {
    Logger::init("logs/drone_tracking.log");

    QApplication a(argc, argv);

    QDir configDir = QDir::current();
    configDir.cd("captured_photos");
    auto config_file_path = configDir.absoluteFilePath("camera_calibration.yaml");
    spdlog::info("Config file path: {}", config_file_path.toStdString());
    DroneTrackingWindow window (config_file_path.toStdString());

    window.show();

    return a.exec();
}