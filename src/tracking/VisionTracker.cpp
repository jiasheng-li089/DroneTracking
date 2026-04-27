# include "VisionTracker.h"

#include <spdlog/spdlog.h>

namespace tracking
{
    
VisionTracker::VisionTracker(QObject *parent) : QObject(parent) {}

VisionTracker::~VisionTracker() = default;

void VisionTracker::process_frames(const int camera_id, const std::string & serial, const rs2::frameset& frames) {
    spdlog::debug("Processing frameset with {} frames", frames.size());
}
} // namespace tracking
