//
// Created by jason on 14/04/26.
//

#ifndef DRONETRACKING_REALSENSECAMERA_H
#define DRONETRACKING_REALSENSECAMERA_H

#include <QObject>
#include <string>
#include <librealsense2/rs.hpp>

namespace realsense {

class RealSenseCamera : QObject {

    Q_OBJECT

public:
    void start();

    void stop();
private:
    std::string m_serial_number;

    rs2::pipeline p;
};

} // realsense

#endif //DRONETRACKING_REALSENSECAMERA_H
