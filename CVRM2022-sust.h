#ifndef _CVRM2022_SUST_H
#define _CVRM2022_SUST_H
#include "ReadJson/ReadJson.h"
#include "SerialPort/SerialPort.h"
#include "AutoAim/AutoAim.h"
#include <pthread.h>

#define CONFIG_DIR "/home/y/github/CVRM2022-sust/data/CVRM2022-sust.json"

pthread_mutex_t g_serial_tx_mutex = PTHREAD_MUTEX_INITIALIZER;

pthread_mutex_t g_aim_mutex = PTHREAD_MUTEX_INITIALIZER;

struct ConfigInfo g_config_info = {0};
struct Aim g_aim = {0};
struct SerialPortRx g_serial_rx = {
    .start_flag = 0x55,
    .yaw = 0.0,
    .pitch = 0.0,
    .roll = 0.0, 
    .end_flag = 0x54,
};
struct SerialPortTx g_serial_tx = {
    .start_flag = 0x55,
    .x_offset = 0,
    .y_offset = 0,
    .shooting = 0,
    .end_flag = 0x54,
};

#endif
