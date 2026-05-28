#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <vector>

// Helper struct to hold our results
struct CameraResolution {
    int width;
    int height;
};

std::vector<CameraResolution> getSupportedResolutions(const std::string& device_path) {
    std::vector<CameraResolution> resolutions;
    
    int fd = open(device_path.c_str(), O_RDONLY);
    if (fd == -1) {
        std::cerr << "Failed to open " << device_path << std::endl;
        return resolutions;
    }

    struct v4l2_frmsizeenum frmsize;
    frmsize.index = 0;
    
    // We are asking for resolutions that support Motion-JPEG (MJPG)
    // You could change this to V4L2_PIX_FMT_YUYV for raw uncompressed video
    // frmsize.pixel_format = V4L2_PIX_FMT_MJPEG; 
    frmsize.pixel_format = V4L2_PIX_FMT_YUYV; 

    // Loop through the indices until the kernel says "no more formats"
    while (ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frmsize) == 0) {
        if (frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
            // Discrete means standard fixed sizes (e.g., 1920x1080, 1280x720)
            resolutions.push_back({
                static_cast<int>(frmsize.discrete.width), 
                static_cast<int>(frmsize.discrete.height)
            });
        }
        frmsize.index++;
    }

    close(fd);
    return resolutions;
}

int main() {
    auto resList = getSupportedResolutions("/dev/video6");
    
    std::cout << "Supported MJPEG Resolutions:\n";
    for (const auto& res : resList) {
        std::cout << res.width << " x " << res.height << "\n";
    }
    
    return 0;
}
