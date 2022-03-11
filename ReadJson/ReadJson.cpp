#include "ReadJson.h"
#include <json.h>
#include <fstream>

static std::string readFileIntoString(const std::string &path) {
    std::ifstream input_file(path);
    if (!input_file.is_open()) {
        return NULL;
    }
    return std::string((std::istreambuf_iterator<char>(input_file)), std::istreambuf_iterator<char>());
}

bool ReadJsonConfig(std::string config_dir, ConfigInfo *config_info) {
    Json::Reader reader;
    Json::Value rootValue;
    if (!reader.parse(readFileIntoString(config_dir), rootValue)) {
        std::cerr<<"read"<<config_dir<<"error!"<<std::endl;
        return false;
    }
    if (!rootValue.isObject()) {
        return false;
    }
    if (rootValue.isMember("nms_thresh") && rootValue["nms_thresh"].isDouble()) {
        config_info->nms_thresh = rootValue["nms_thresh"].asFloat();
    } else {
        return false;
    }
    if (rootValue.isMember("conf_thresh") && rootValue["conf_thresh"].isDouble()) {
        config_info->conf_thresh = rootValue["conf_thresh"].asFloat();
    } else {
        return false;
    }
    if (rootValue.isMember("engine_dir") && rootValue["engine_dir"].isString()) {
        config_info->engine_dir = rootValue["engine_dir"].asString();
    } else {
        return false;
    }
    if (rootValue.isMember("source") && rootValue["source"].isString()) {
        config_info->source_dir = rootValue["source"].asString();
    } else {
        return false;
    }
    if (rootValue.isMember("run_mode") && rootValue["run_mode"].isString()) {
        config_info->run_mode = rootValue["run_mode"].asString();
    } else {
        return false;
    }
    if (rootValue.isMember("show_mode") && rootValue["show_mode"].isBool()) {
        config_info->show_mode = rootValue["show_mode"].asBool();
    } else {
        return false;
    }
    if (rootValue.isMember("aim") && rootValue["aim"].isObject()) {
        if (rootValue["aim"].isMember("center_point") && rootValue["aim"]["center_point"].isArray() && rootValue["aim"]["center_point"].size() == 2 
            && rootValue["aim"]["center_point"][0].isInt() && rootValue["aim"]["center_point"][1].isInt()) {
                config_info->aim_point = cv::Point(rootValue["aim"]["center_point"][0].asInt(),
                                        rootValue["aim"]["center_point"][1].asInt());
        } else {
            return false;
        }
    } else {
        return false;
    }
    return false;
}
