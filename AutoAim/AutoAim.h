#ifndef _AUTO_AIM_H
#define _AUTO_AIM_H

#include <opencv2/opencv.hpp>

#define CONFIG_DIR "/home/y/github/CVRM2022-sust/data/CVRM2022-sust.json"

struct Aim{
    bool is_find_arm;
    cv::Rect aim_rect;
}Aim;

void AutoAimHandle();

#endif