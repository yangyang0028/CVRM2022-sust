#include "CVRM2021-sust.h"
#include <iostream>
#include <algorithm>

static cv::Rect g_last_target;

static int g_lost_cnt = 0;

static int g_gray_th = 10;

static int g_color_th = 30;

class LedStick{
    public:
        LedStick():matched(false){}
        LedStick(const cv::RotatedRect& r_) {
            r.angle=r_.angle;
            r.center=r_.center;
            r.size=r_.size;
            matched=false;
            match_index = 0;
            match_factor = 0;
        }
        cv::RotatedRect r; 
        bool matched; 
        size_t match_index;
        float match_factor;
};

class Armor{
    public:
        Armor(){};
        Armor(const LedStick& l1,const LedStick& l2);
        void DrawRect(cv::Mat& img,cv::Point2f roi) const;
        void DrawSpot(cv::Mat& Img, cv::Point2f Roi) const;
        int GetAverageIntensity(const cv::Mat& img);
        void MaxMatch(std::vector<LedStick>& led, size_t i, size_t j);
        bool IsSuitableSize(void) const;
        LedStick led[2];
        float error_angle;
        cv::Point2i center;
        cv::Rect2i r;
        int average_intensity;
};

Armor::Armor(const LedStick& l1, const LedStick& l2) {
    led[0]= l1;
    led[1]= l2;
    error_angle = fabs(l1.r.angle - l2.r.angle);

    r.width = abs(l1.r.center.x - l2.r.center.x);

    r.height = (l1.r.size.height + l2.r.size.height)/2.0;

    center.x = (l1.r.center.x + l2.r.center.x)/2.0;
    center.y = (l1.r.center.y + l2.r.center.y)/2.0;

    r.x = center.x - r.width/3;
    r.y = center.y - r.height/3;
    r.width*= 2.0/3;
    r.height*= 2.0/3;
}

bool Armor::IsSuitableSize(void) const {
    if( led[0].r.size.height*0.7f < led[1].r.size.height && led[0].r.size.height*1.3f > led[1].r.size.height){

        float armor_width = fabs(led[0].r.center.x - led[1].r.center.x);
        if(armor_width > led[0].r.size.width
           && armor_width > led[1].r.size.width
           && armor_width > (led[0].r.size.width+led[1].r.size.width)*3) {

            float h_max = (led[0].r.size.height + led[1].r.size.height)/2.0f;
            if(fabs(led[0].r.center.y - led[1].r.center.y) < 0.8f* h_max ) {
                if(h_max*4.0f > r.width && h_max < 1.2f* r.width){
                    return true;
                }
            }
        }
    }
    return false;
}

int Armor::GetAverageIntensity(const cv::Mat& img) {
    if(r.width < 1 || r.height < 1 || r.x < 1 || r.y < 1 || r.width + r.x > img.cols || r.height + r.y > img.rows) return 255;
    cv::Mat roi = img(cv::Range(r.y, r.y + r.height), cv::Range(r.x, r.x + r.width));
    average_intensity = mean(roi).val[0];
    return average_intensity;
}

void Armor::MaxMatch(std::vector<LedStick>& led, size_t i, size_t j){// 灯条匹配算法
    cv::RotatedRect r, l;
    if( led[0].r.center.x > led[1].r.center.x){
        r =  led[0].r;
        l =  led[1].r;
    }else{
        r =  led[1].r;
        l =  led[0].r;
    }

    float angle = l.angle - r.angle;
    if(angle < 1e-3f) angle = 0.0f;
    float f = error_angle + 0.5 * angle;
    if(!led.at(i).matched && !led.at(j).matched){

        led.at(i).matched = true;
        led.at(i).match_index = j;
        led.at(j).matched = true;
        led.at(j).match_index = i;
        led.at(i).match_factor = f;
        led.at(j).match_factor = f;
    }
    if(led.at(i).matched && !led.at(j).matched){
        if(f < led.at(i).match_factor){
            led.at(led.at(i).match_index).matched = false;
            led.at(i).match_factor = f;
            led.at(i).match_index = j;
            led.at(j).matched = true;
            led.at(j).match_factor = f;
            led.at(j).match_index = i;

        }
    }
    if(led.at(j).matched && !led.at(i).matched){
        if(f < led.at(j).match_factor ){
            led.at(led.at(j).match_index).matched = false;
            led.at(j).match_factor = f;
            led.at(j).match_index = i;
            led.at(i).matched = true;
            led.at(i).match_factor = f;
            led.at(i).match_index = j;
        }
    }
    if(led.at(j).matched && led.at(i).matched &&
            led.at(i).match_factor > f &&
            led.at(j).match_factor > f){
        led.at(led.at(j).match_index).matched = false;
        led.at(led.at(i).match_index).matched = false;
        led.at(i).matched = true;
        led.at(i).match_factor = f;
        led.at(i).match_index = j;
        led.at(j).matched = true;
        led.at(j).match_factor = f;
        led.at(j).match_index = i;
    }
}

void Armor::DrawRect( cv::Mat& img, cv::Point2f roi) const{
    cv::rectangle(img, r+cv::Point_<int>(roi), cv::Scalar(255,255,255), 1);
}

void Armor::DrawSpot(cv::Mat& img, cv::Point2f roi) const{
    cv::circle(img, center + cv::Point_<int>(roi), int(r.height/4), cv::Scalar(0,0,255), -1);
}

cv::Rect GetRoi(const cv::Mat &img) {
    cv::Size img_size = img.size();
    cv::Rect rect_tmp = g_last_target;
    cv::Rect rect_roi;

    if(rect_tmp.x==0||rect_tmp.y==0||
       rect_tmp.width==0||rect_tmp.height==0) {
        g_last_target = cv::Rect(0,0,img_size.width,img_size.height);
        rect_roi = cv::Rect(0,0,img_size.width,img_size.height);
    }else{
        float scale=2;
        if(g_lost_cnt<30) scale=3;
        else if(g_lost_cnt<=60) scale=4;
        else if(g_lost_cnt<=120) scale=5;

        int w=int(rect_tmp.width * scale);
        int h=int(rect_tmp.height * scale);
        int x=std::min(std::max(int(rect_tmp.x-(w-rect_tmp.width)*0.5f),0),img_size.width);
        int y=std::min(std::max(int(rect_tmp.y-(h-rect_tmp.height)*0.5f),0),img_size.height);
        w=std::min(int(rect_tmp.width*scale),img_size.width-x);
        h=std::min(int(rect_tmp.height*scale),img_size.height-y);
        rect_roi=cv::Rect(x,y,w,h);
    }
    return rect_roi;
}

void DetectArmor(cv::Mat& img, cv::Rect roi_rect) {
    cv::Mat roi_image = img(roi_rect);

    cv::Point2f offset_roi_point(roi_rect.x, roi_rect.y);

    std::vector<LedStick> led;

    cv::Mat binary_brightness_img, binary_color_img, gray;
    cv::cvtColor(roi_image,gray,cv::COLOR_BGR2GRAY);
    std::vector<cv::Mat> bgr;
    cv::split(roi_image, bgr);
    cv::Mat result_img;

    if(g_config_info.run_mode == "armor_red") cv::subtract(bgr[2], bgr[1], result_img);
    else cv::subtract(bgr[0], bgr[1], result_img);

    cv::threshold(gray, binary_brightness_img, g_gray_th, 255, cv::THRESH_BINARY);
    cv::threshold(result_img, binary_color_img, g_color_th, 255, cv::THRESH_BINARY);

    std::vector<std::vector<cv::Point>> contours_light;
    std::vector<std::vector<cv::Point>> contours_brightness;

    cv::findContours(binary_color_img, contours_light, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    cv::findContours(binary_brightness_img, contours_brightness, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    imshow("Binary color img", binary_color_img);
    imshow("Binary brightness img", binary_brightness_img);
    for(size_t i = 0; i < contours_brightness.size(); i++){
        double area = cv::contourArea(contours_brightness[i]);
        if (area < 5 || 1e5 < area) continue;

        for(size_t j = 0; j < contours_light.size(); j++) {
            
            if(cv::pointPolygonTest(contours_light[j], contours_brightness[i][0], false) >= 0.0 ) {

                double length = cv::arcLength(contours_brightness[i], true);

                if (length > 5 && length <4000){

                    cv::RotatedRect r_rect = cv::fitEllipse(contours_brightness[i]);
                    
                    cv::Point2f rect_point[4];

                    r_rect.points(rect_point);

                    for (int k = 0; k < 4 ; k++){
                        line(img, rect_point[k]+offset_roi_point, rect_point[(k+1)%4] + offset_roi_point, cv::Scalar(255,0,0),1);
                    }

                    if(r_rect.angle>90.0f) r_rect.angle =  r_rect.angle - 180.0f;

                    if (fabs(r_rect.angle) <= 30){
                        LedStick r(r_rect);
                        led.push_back(r);
                    }
                }
                break;
            }
        }
    }

    for(size_t i = 0; i < led.size() ; i++) {
        for(size_t j = i + 1; j < led.size() ; j++) {
            Armor arm_tmp(led.at(i), led.at(j));
            if (arm_tmp.error_angle < 8.0f) {
                if(arm_tmp.IsSuitableSize()){
                    if(arm_tmp.GetAverageIntensity(gray)< 50 ){
                        arm_tmp.MaxMatch(led, i, j);
                    }
                }
            }
        }
    }

    std::vector<Armor> final_armor_list;
    for(size_t i = 0; i < led.size() ; i++){
        if(led.at(i).matched){
            led.at(led.at(i).match_index).matched = false;
            Armor arm_tmp( led.at(i), led.at(led.at(i).match_index));
            final_armor_list.push_back(arm_tmp);
        }
    }

    float dist=1e8;
    bool found_flag = false;
    Armor target;
    cv::Point2f roi_center(roi_rect.width/2, roi_rect.height/2);
    float dx,dy;
    for (size_t i = 0; i < final_armor_list.size() ; i++ ){
        dx = fabs(final_armor_list.at(i).center.x - roi_center.x);
        dy = fabs(final_armor_list.at(i).center.y - roi_center.y);
        if( dx + dy < dist){
            target = final_armor_list.at(i);
            dist = dx + dy;
        }
        final_armor_list.at(i).DrawRect(img, offset_roi_point);
        found_flag = true;
    }
    cv::rectangle(img, roi_rect, cv::Scalar(255, 0, 255), 1);

    if(found_flag) {
        target.DrawSpot(img, offset_roi_point);
        g_last_target = target.r + cv::Point_<int>(offset_roi_point);
        g_aim.aim_rect = g_last_target;
        g_aim.is_find_arm = true;
        g_lost_cnt = 0;
    }else{
        g_last_target = cv::Rect(0, 0, 0, 0);
        g_aim.aim_rect = cv::Rect(0, 0, 0, 0);
        g_aim.is_find_arm = false;
        g_lost_cnt ++;
    }

    if (g_config_info.show_mode) {
        cv::imshow("video", img);
    }
}

void AutoAimHandle(){
    cv::VideoCapture capture(g_config_info.source_dir);
    cv::VideoWriter output_video;
    if (!capture.isOpened()) {
        std::cerr << "Error opening video stream or file" << std::endl;
        return ;
    }

    cv::Size capture_size = cv::Size(capture.get(cv::CAP_PROP_FRAME_WIDTH),
                                    capture.get(cv::CAP_PROP_FRAME_HEIGHT));

    int myFourCC = cv::VideoWriter::fourcc('X', 'V', 'I', 'D');
    output_video.open(g_config_info.saved_video_dir, myFourCC, 30.0, capture_size,
                        true);

    if (!output_video.isOpened()) {
        std::cout << g_config_info.saved_video_dir << " fail to open!" << std::endl;
        return ;
    }

    if (g_config_info.show_mode) {
        cv::namedWindow("video", cv::WINDOW_AUTOSIZE);
    }

    while(true) {
        cv::Mat frame;
        capture >> frame;
        if (frame.empty()) {
        continue;
        }
        cv::Rect roi = GetRoi(frame);
        DetectArmor(frame, roi);
        output_video.write(frame);
        if (g_config_info.show_mode) {
            cv::waitKey(1);
        }
    }
}
