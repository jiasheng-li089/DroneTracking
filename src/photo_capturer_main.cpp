#include "gui/PhotoCaptureWindow.h"
#include "logger.h"

#include <QApplication>
#include <QSurfaceFormat> // Required for configuring OpenGL Widget
#include <QDir>


int main(int argc, char *argv[]) {
    Logger::init("logs/photo_capturing.log");
    // QSurfaceFormat must be set BEFORE creating the QApplication instance.
    QSurfaceFormat format;
    format.setVersion(3, 3);
    format.setProfile(QSurfaceFormat::CoreProfile);
    QSurfaceFormat::setDefaultFormat(format);

    QApplication a(argc, argv);

    PhotoCaptureWindow w(QDir::currentPath().toStdString() + "/captured_photos");
    w.show();

    return a.exec();
}
