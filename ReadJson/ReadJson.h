#ifndef READ_JSON_H
#define READ_JSON_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

struct ConfigInfo{
    float nms_thresh;
    float conf_thresh;
    std::string engine_dir;
    std::string run_mode;
    std::string source_dir;
    bool show_mode;
    cv::Point aim_point;
};

bool ReadJsonConfig(std::string config_dir, ConfigInfo *config_info);

#endif
