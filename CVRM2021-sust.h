#ifndef _CVRM2021_SUST_H
#define _CVRM2021_SUST_H
#include "SerialPort/SerialPort.h"
#include "ReadJson/ReadJson.h"
#include "AutoAimCV.h"

#define CONFIG_DIR "/home/y/github/CVRM2022-sust/data/CVRM2022-sust.json"

struct ConfigInfo g_config_info = {0};
struct Aim g_aim = {0};

#endif