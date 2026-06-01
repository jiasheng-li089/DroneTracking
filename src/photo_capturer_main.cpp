#include "gui/AutoPhotoCaptureWindow.h"
#include "logger.h"

#include <QApplication>
#include <QSurfaceFormat> // Required for configuring OpenGL Widget
#include <QDir>
#include <opencv2/core/types.hpp>
#include <qdir.h>


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

    auto camera_serial = "903223052137";
    AutoPhotoCaptureWindowConfig config {
        camera_serial,
        cv::Size(10, 7),
        25.0f,
        current.absolutePath().toStdString(),
        camera_serial
    };

    AutoPhotoCaptureWindow w(config, nullptr);
    w.show();

    return a.exec();
}
