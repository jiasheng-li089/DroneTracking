#pragma once

#include <QObject>

#include <librealsense2/rs.hpp>

class rs2::frameset;

namespace tracking {
class VisionTracker : public QObject {
    Q_OBJECT
   public:
    VisionTracker(QObject *parent = nullptr);

    ~VisionTracker();

    void process_frames(const int camera_id, const std::string & serial, const rs2::frameset& frames);
};
}  // namespace tracking