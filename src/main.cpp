#include "gui/MainWindow.h"
#include <QApplication>
#include <QSurfaceFormat> // Required for configuring OpenGL Widget

int main(int argc, char *argv[]) {
    // QSurfaceFormat must be set BEFORE creating the QApplication instance.
    QSurfaceFormat format;
    format.setVersion(3, 3);
    format.setProfile(QSurfaceFormat::CoreProfile);
    QSurfaceFormat::setDefaultFormat(format);

    QApplication a(argc, argv);

    MainWindow w;
    w.show();

    return a.exec();
}
