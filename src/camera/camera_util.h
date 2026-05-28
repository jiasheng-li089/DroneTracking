#pragma once

#include <vector>
#include <string>

struct CameraResolution {
    int width;
    int height;
    std::vector<double> fps;
};

struct CameraMeta {
    int result;
    int id;
    std::string serial;
    std::vector<CameraResolution> supported_resolutions;
};

std::vector<int> detect_rgb_cameras();


CameraMeta get_camera_meta(int camera_id);


std::vector<CameraMeta> get_camera_metas(std::vector<int> camera_ids);

std::vector<CameraMeta> get_all_camera_metas();