#ifndef AUTO_AIM_H
#define AUTO_AIM_H

#define CONFIG_DIR "/home/y/github/CVRM2022-sust/data/auto_aim.json"

/*
#include <iostream>
#include <chrono>
#include <unistd.h>
#include <pwd.h>
#include <json.h>
#include <CppLinuxSerial/SerialPort.hpp>
#include "cuda_utils.h"
#include "logging.h"
#include "common.hpp"
#include "utils.h"
#include "calibrator.h"

#define DEVICE 0
#define BATCH_SIZE 1
#define THREAD 1

using namespace mn::CppLinuxSerial;

// stuff we know about the network and the input/output blobs
static const int INPUT_H = Yolo::INPUT_H;
static const int INPUT_W = Yolo::INPUT_W;
static const int OUTPUT_SIZE = Yolo::MAX_OUTPUT_BBOX_COUNT * sizeof(Yolo::Detection) / sizeof(float) + 1;
static const passwd *USR_PASSWRD = getpwuid(getuid());
static char *USR_HOMEDIR = USR_PASSWRD->pw_dir;
static const char *INPUT_BLOB_NAME = "data";
static const char *OUTPUT_BLOB_NAME = "prob";

static float NMS_THRESH;
static float CONF_THRESH;
static std::string engine_dir;
static std::string run_mode;
static std::string source_dir;
static bool show_mode;
static cv::Point aim_point;
static cv::Rect aim_box;
static int AIM_WIDTH;
static int AIM_HEIGHT;
static int MIN_AIM_SIZE;

static Logger gLogger;

const cv::Scalar white = cv::Scalar(0xFF, 0xFF, 0xFF);
const cv::Scalar red = cv::Scalar(0x47, 0x63, 0xFF);
const cv::Scalar blue = cv::Scalar(0xEB, 0xCE, 0x87);
const cv::Scalar green = cv::Scalar(0x22, 0x8B, 0x22);
const cv::Scalar pink = cv::Scalar(0xDD, 0xA0, 0xDD);
const std::string class_name[] = {"armor_blue",
                                  "armor_red",
                                  "ignore",
                                  "base",
                                  "watcher",
                                  "car"};
const std::string portName = "/dev/ttyTHS1";

SerialPort serialPort(portName, BaudRate::B_57600, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);


void doInference(IExecutionContext &context, cudaStream_t &stream, void **buffers, float *input, float *output,
                 int batchSize) {
    // DMA input batch data to device, infer on the batch asynchronously, and DMA output back to host
    CUDA_CHECK(cudaMemcpyAsync(buffers[0], input, batchSize * 3 * INPUT_H * INPUT_W * sizeof(float),
                               cudaMemcpyHostToDevice, stream));
    context.enqueue(batchSize, buffers, stream, nullptr);
    CUDA_CHECK(cudaMemcpyAsync(output, buffers[1], batchSize * OUTPUT_SIZE * sizeof(float), cudaMemcpyDeviceToHost,
                               stream));
    cudaStreamSynchronize(stream);
}

cv::Point rect_center_point(const cv::Rect &rect) {
    cv::Point center_point;
    center_point.x = rect.x + cvRound(rect.width / 2.0);
    center_point.y = rect.y + cvRound(rect.height / 2.0);
    return center_point;
}

cv::Point image_center_point(const cv::Mat &image) {
    cv::Point center_point;
    center_point.x = cvRound(image.size().width / 2.0);
    center_point.y = cvRound(image.size().height / 2.0);
    return center_point;
}

cv::Rect expand_rect_from_point(const cv::Point &point, int width, int height) {
    cv::Rect rect;
    rect.x = point.x - width / 2 - 1;
    rect.y = point.y - height / 2 - 1;
    rect.width = width + 1;
    rect.height = height + 1;
    return rect;
}

int box_size(const cv::Rect &rect) {
    return rect.height * rect.width;
}

bool isPointInRect(const cv::Point &point, const cv::Rect &rect) {
    cv::Point A = rect.tl();
    cv::Point B = rect.br();
    if ((point.x > A.x) && (point.x < B.x) && (point.y > A.y) && (point.y < B.y)) {
        return true;
    } else {
        return false;
    }
}

std::string getStringFromRectCenter(const cv::Rect &rect) {
    std::string str =
            "[" + std::to_string(rect_center_point(rect).x) + "," + std::to_string(rect_center_point(rect).y) + "]";
    return str;
}

std::string getStringFromRectSize(const cv::Rect &rect) {
    std::string str = "(" + std::to_string(box_size(rect)) + ")";
    return str;
}

std::string getStringFromPoint(const cv::Point &point) {
    std::string str = "[" + std::to_string(point.x) + "," + std::to_string(point.y) + "]";
    return str;
}

std::string readFileIntoString(const std::string &path) {
    std::ifstream input_file(path);
    if (!input_file.is_open()) {
        return NULL;
    }
    return std::string((std::istreambuf_iterator<char>(input_file)), std::istreambuf_iterator<char>());
}

bool readConfig() {
    Json::Reader reader;
    Json::Value rootValue;
    std::string config_dir = std::string(USR_HOMEDIR) + "/.auto_aim/auto_aim.json";
    if (!reader.parse(readFileIntoString(config_dir), rootValue)) {
        return false;
    }
    if (!rootValue.isObject()) {
        return false;
    }
    if (rootValue.isMember("nms_thresh") && rootValue["nms_thresh"].isDouble()) {
        NMS_THRESH = rootValue["nms_thresh"].asFloat();
    } else {
        return false;
    }
    if (rootValue.isMember("conf_thresh") && rootValue["conf_thresh"].isDouble()) {
        CONF_THRESH = rootValue["conf_thresh"].asFloat();
    } else {
        return false;
    }
    if (rootValue.isMember("engine") && rootValue["engine"].isString()) {
        engine_dir = rootValue["engine"].asString();
        if (engine_dir == "n") {
            engine_dir = std::string(USR_HOMEDIR) + "/.auto_aim/yolov5n.engine";
        }
        if (engine_dir == "s") {
            engine_dir = std::string(USR_HOMEDIR) + "/.auto_aim/yolov5s.engine";
        }
    } else {
        return false;
    }
    if (rootValue.isMember("source") && rootValue["source"].isString()) {
        source_dir = rootValue["source"].asString();
    } else {
        return false;
    }
    if (rootValue.isMember("run_mode") && rootValue["run_mode"].isString()) {
        run_mode = rootValue["run_mode"].asString();
    } else {
        return false;
    }
    if (rootValue.isMember("show_mode") && rootValue["show_mode"].isBool()) {
        show_mode = rootValue["show_mode"].asBool();
    } else {
        return false;
    }
    if (rootValue.isMember("aim") && rootValue["aim"].isObject()) {
        if (rootValue["aim"].isMember("center_point") && rootValue["aim"]["center_point"].isArray()) {
            if (rootValue["aim"]["center_point"].size() == 2) {
                if (rootValue["aim"]["center_point"][0].isInt() && rootValue["aim"]["center_point"][1].isInt()) {
                    aim_point = cv::Point(rootValue["aim"]["center_point"][0].asInt(),
                                          rootValue["aim"]["center_point"][1].asInt());
                } else {
                    return false;
                }
            } else {
                return false;
            }
        } else {
            return false;
        }
        if (rootValue["aim"].isMember("width") && rootValue["aim"]["width"].isInt()) {
            AIM_WIDTH = rootValue["aim"]["width"].asInt();
        } else {
            return false;
        }
        if (rootValue["aim"].isMember("height") && rootValue["aim"]["height"].isInt()) {
            AIM_HEIGHT = rootValue["aim"]["height"].asInt();
        } else {
            return false;
        }
        if (rootValue["aim"].isMember("min_size") && rootValue["aim"]["min_size"].isInt()) {
            MIN_AIM_SIZE = rootValue["aim"]["min_size"].asInt();
        } else {
            return false;
        }
    } else {
        return false;
    }
    return true;
}
*/
#endif //AUTO_AIM_H
