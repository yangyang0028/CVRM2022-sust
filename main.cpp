#include "AutoAim/AutoAim.h"
#include "SerialPort/SerialPort.h"
#include "ReadJson/ReadJson.h"
#include "Predict/Predict.h"
#include "main.h"

#include <thread>

static int g_srial_port = 0;

int main() {

  ReadJsonConfig(CONFIG_DIR, &g_config_info);
  char tmp_string[30] = {0};
  snprintf(tmp_string, sizeof(tmp_string), "/%ld.avi", time(0));
  g_config_info.saved_video_dir = g_config_info.saved_video_dir + tmp_string;
  std::cout << "nms_thresh  " << g_config_info.nms_thresh << std::endl;
  std::cout << "conf_thresh  " << g_config_info.conf_thresh << std::endl;
  std::cout << "engine_dir  " << g_config_info.engine_dir << std::endl;
  std::cout << "camera_config_dir  " << g_config_info.camera_config_dir << std::endl;
  std::cout << "saved_video_dir  " << g_config_info.saved_video_dir << std::endl;
  std::cout << "run_mode  " << g_config_info.run_mode << std::endl;
  std::cout << "source  " << g_config_info.source_dir << std::endl;
  std::cout << "serial_port  " << g_config_info.serial_port << std::endl;
  std::cout << "show_mode  " << g_config_info.show_mode << std::endl;
  std::cout << "center_point  " << g_config_info.aim_point << std::endl;

  g_srial_port = OpenPort((char*)g_config_info.serial_port.data());
  SetPort(g_srial_port,115200,8,'N',1);

  std::thread AutoAim(AutoAimHandle);
  std::thread SerialPortRx(SerialPortRxHandle, g_srial_port);
  std::thread SerialPortTx(SerialPortTxHandle, g_srial_port);
  std::thread Predict(PredictHandle);
  SerialPortRx.detach();
  SerialPortTx.detach();
  AutoAim.detach();
  Predict.join();
  return 0;
}
