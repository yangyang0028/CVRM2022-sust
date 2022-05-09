#ifndef _AUTO_AIM_CV_H
#define _AUTO_AIM_CV_H

#include <opencv2/opencv.hpp>

struct Aim{
    uint32_t count;
    bool is_find_arm;
    cv::Rect aim_rect;
}Aim;

void AutoAimHandle();

#endif