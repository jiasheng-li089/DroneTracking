# include "VisionTracker.h"

#include <spdlog/spdlog.h>

namespace tracking
{
    
VisionTracker::VisionTracker(QObject *parent) : QObject(parent) {}

VisionTracker::~VisionTracker() = default;

void VisionTracker::process_frames(const int camera_id, const std::string & serial, const rs2::frameset& frames) {
    // detect the possible position of the target (aurcode) related to the camera.


    // get the real distance of the target (aurcode) from the camera


    // based on the real distance and the related position, calculate the relative position of the target (aurcode) to the camera


    // average the relative positions of the target (aurcode) to the cameras to get a stable position estimation of the target (aurcode)
}
} // namespace tracking
