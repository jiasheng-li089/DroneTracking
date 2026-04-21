#include <stdlib.h>

#include <QApplication>
#include <QDir>

#include "gui/DroneTrackingWindow.h"
#include "logger.h"

int main(int argc, char *argv[]) {
    Logger::init("logs/drone_tracking.log");

    QApplication a(argc, argv);

    DroneTrackingWindow window (QDir::currentPath().toStdString() + "/captured_photos/20260415_125626.yaml");

    window.show();

    return a.exec();
}