# include <stdlib.h>

#include <QApplication>
#include <QDir>

# include "gui/DroneTrackingWindow.h"

int main(int argc, char *argv[]) {
    
    QApplication a(argc, argv);

    DroneTrackingWindow window (QDir::currentPath().toStdString() + "/captured_photos/20260415_125626.yaml");

    window.show();

    return a.exec();
}