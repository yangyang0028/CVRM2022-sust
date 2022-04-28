#include "AutoAim.h"
#include <ctime>
#include <iostream>

#include "CVRM2022-sust.h"
#include "ReadJson/ReadJson.h"
#include "YoloLayer/common.hpp"
#include "YoloLayer/cuda_utils.h"
#include "YoloLayer/logging.h"
#include "YoloLayer/yololayer.h"
#include "SerialPort/SerialPort.h"

#define DEVICE 0
#define BATCH_SIZE 1
#define MIN_AIM_SIZE 1000

const cv::Scalar white = cv::Scalar(0xFF, 0xFF, 0xFF);
const cv::Scalar red = cv::Scalar(0x47, 0x63, 0xFF);
const cv::Scalar blue = cv::Scalar(0xEB, 0xCE, 0x87);
const cv::Scalar green = cv::Scalar(0x22, 0x8B, 0x22);
const cv::Scalar pink = cv::Scalar(0xDD, 0xA0, 0xDD);
const std::string class_name[] = {"armor_blue", "armor_red", "ignore",
                                  "base",       "watcher",   "car"};

static const char *INPUT_BLOB_NAME = "data";
static const char *OUTPUT_BLOB_NAME = "prob";

static const int INPUT_H = Yolo::INPUT_H;
static const int INPUT_W = Yolo::INPUT_W;
static const int OUTPUT_SIZE =
    Yolo::MAX_OUTPUT_BBOX_COUNT * sizeof(Yolo::Detection) / sizeof(float) + 1;

static Logger gLogger;

static inline cv::Mat preprocess_img(cv::Mat &img, int input_w, int input_h) {
  int w, h, x, y;
  float r_w = input_w / (img.cols * 1.0);
  float r_h = input_h / (img.rows * 1.0);
  if (r_h > r_w) {
    w = input_w;
    h = r_w * img.rows;
    x = 0;
    y = (input_h - h) / 2;
  } else {
    w = r_h * img.cols;
    h = input_h;
    x = (input_w - w) / 2;
    y = 0;
  }
  cv::Mat re(h, w, CV_8UC3);
  cv::resize(img, re, re.size(), 0, 0, cv::INTER_LINEAR);
  cv::Mat out(input_h, input_w, CV_8UC3, cv::Scalar(128, 128, 128));
  re.copyTo(out(cv::Rect(x, y, re.cols, re.rows)));
  return out;
}
static void doInference(IExecutionContext &context, cudaStream_t &stream,
                        void **buffers, float *input, float *output,
                        int batchSize) {
  // DMA input batch data to device, infer on the batch asynchronously, and DMA
  // output back to host
  CUDA_CHECK(cudaMemcpyAsync(buffers[0], input,
                             batchSize * 3 * INPUT_H * INPUT_W * sizeof(float),
                             cudaMemcpyHostToDevice, stream));
  context.enqueue(batchSize, buffers, stream, nullptr);
  CUDA_CHECK(cudaMemcpyAsync(output, buffers[1],
                             batchSize * OUTPUT_SIZE * sizeof(float),
                             cudaMemcpyDeviceToHost, stream));
  cudaStreamSynchronize(stream);
}

static int box_size(const cv::Rect &rect) { return rect.height * rect.width; }

static cv::Point rect_center_point(const cv::Rect &rect) {
  cv::Point center_point;
  center_point.x = rect.x + cvRound(rect.width / 2.0);
  center_point.y = rect.y + cvRound(rect.height / 2.0);
  return center_point;
}

std::string getStringFromPoint(const cv::Point &point) {
  std::string str =
      "[" + std::to_string(point.x) + "," + std::to_string(point.y) + "]";
  return str;
}

void AutoAimHandle() {

  std::ifstream file(g_config_info.engine_dir, std::ios::binary);
  if (!file.good()) {
    std::cerr << " read " << g_config_info.engine_dir << " error! " << std::endl;
    assert(0);
  }

  char *trtModelStream{nullptr};
  size_t size = 0;
  file.seekg(0, std::ifstream::end);
  size = file.tellg();
  file.seekg(0, std::ifstream::beg);
  trtModelStream = new char[size];
  assert(trtModelStream);
  file.read(trtModelStream, size);
  file.close();

  static float data[BATCH_SIZE * 3 * INPUT_H * INPUT_W];
  static float prob[BATCH_SIZE * OUTPUT_SIZE];
  IRuntime *runtime = createInferRuntime(
      gLogger);  //创建运行时环境 IRuntime对象，传入 gLogger 用于打印信息
  assert(runtime != nullptr);
  ICudaEngine *engine = runtime->deserializeCudaEngine(
      trtModelStream,
      size);  //将序列化得到的结果进行反序列化，以执行后续的inference
  delete[] trtModelStream;
  assert(engine != nullptr);
  IExecutionContext *context =
      engine->createExecutionContext();  // inference推断过程
  assert(context != nullptr);

  assert(engine->getNbBindings() == 2);  // 获取总索引个数
  void *buffers[2];
  const int inputIndex =
      engine->getBindingIndex(INPUT_BLOB_NAME);  //获取输入节点索引
  const int outputIndex =
      engine->getBindingIndex(OUTPUT_BLOB_NAME);  //获取输出节点索引
  assert(inputIndex == 0);
  assert(outputIndex == 1);

  cudaSetDevice(DEVICE);
  CUDA_CHECK(cudaMalloc(&buffers[inputIndex],
                        BATCH_SIZE * 3 * INPUT_H * INPUT_W * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&buffers[outputIndex],
                        BATCH_SIZE * OUTPUT_SIZE * sizeof(float)));
  cudaStream_t stream;
  CUDA_CHECK(cudaStreamCreate(&stream));

  cv::VideoCapture capture(g_config_info.source_dir);
  cv::VideoWriter output_video;
  if (!capture.isOpened()) {
    std::cerr << "Error opening video stream or file" << std::endl;
    return ;
  }

  cv::Size capture_size = cv::Size(capture.get(cv::CAP_PROP_FRAME_WIDTH),
                                   capture.get(cv::CAP_PROP_FRAME_HEIGHT));

  int myFourCC = cv::VideoWriter::fourcc('X', 'V', 'I', 'D');
  output_video.open(g_config_info.saved_video_dir, myFourCC, 20.0, capture_size,
                    true);
  if (!output_video.isOpened()) {
    std::cout << g_config_info.saved_video_dir << " fail to open!" << std::endl;
    return ;
  }

  if (g_config_info.show_mode) {
    cv::namedWindow("video", cv::WINDOW_AUTOSIZE);
  }

  while (true) {
    auto start = std::chrono::system_clock::now();
    cv::Mat frame;
    capture >> frame;
    if (frame.empty()) {
      continue;
    }
    cv::Mat img = frame;
    cv::Mat pr_img =
        preprocess_img(img, INPUT_W, INPUT_H);  // letterbox BGR to RGB

    int count = 0;
    for (int row = 0; row < INPUT_H; ++row) {
      uchar *uc_pixel = pr_img.data + row * pr_img.step;
      for (int col = 0; col < INPUT_W; ++col) {
        data[count] = (float)uc_pixel[2] / 255.0;
        data[count + INPUT_H * INPUT_W] = (float)uc_pixel[1] / 255.0;
        data[count + 2 * INPUT_H * INPUT_W] = (float)uc_pixel[0] / 255.0;
        uc_pixel += 3;
        ++count;
      }
    }

    doInference(*context, stream, buffers, data, prob, BATCH_SIZE);

    std::vector<std::vector<Yolo::Detection>> batch_res(1);

    auto &res = batch_res[0];
    nms(res, prob, g_config_info.conf_thresh, g_config_info.nms_thresh);

    int aim_area = 0;
    cv::Point aim_point = {0};
    cv::Rect aim_rect;
    g_aim.is_find_arm = false;
    
    for (size_t i = 0; i < res.size(); i++) {
      cv::Rect r = get_rect(frame, res[i].bbox);
      if ((res[i].class_id == 0 || res[i].class_id == 1) &&
          box_size(r) >= MIN_AIM_SIZE) {
        cv::rectangle(frame, r, green, 2);
        if (g_config_info.run_mode == class_name[(int)res[i].class_id]) {
          if (aim_area < r.area()) {
            aim_area = r.area();
            aim_point = rect_center_point(r);
            aim_rect = r;
          }
        }
      } else {
        cv::rectangle(frame, r, pink, 2);
      }
      std::string label = class_name[(int)res[i].class_id];
      switch ((int)(res[i].class_id)) {
        case 0: {
          cv::putText(frame, label, cv::Point(r.x, r.y - 1),
                      cv::FONT_HERSHEY_PLAIN, 1.2, blue, 2);
          break;
        }
        case 1: {
          cv::putText(frame, label, cv::Point(r.x, r.y - 1),
                      cv::FONT_HERSHEY_PLAIN, 1.2, red, 2);
          break;
        }
        default: {
          cv::putText(frame, label, cv::Point(r.x, r.y - 1),
                      cv::FONT_HERSHEY_PLAIN, 1.2, pink, 2);
          break;
        }
      }
    }
    if (aim_area) {
      cv::circle(frame, aim_point, 4, white, 2, cv::LINE_AA);
      pthread_mutex_lock(&g_aim_mutex);
      g_aim.aim_rect = aim_rect;
      pthread_mutex_unlock(&g_aim_mutex);
      g_aim.is_find_arm = true;
      g_aim.count ++;
    }
    auto end = std::chrono::system_clock::now();
    int fps = (int) (1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
    std::string video_fps = "FPS: " + std::to_string(fps);
    std::string aim_point_info = "Aim Point: " + getStringFromPoint(aim_point);
    cv::putText(frame, video_fps, cv::Point(10, 40), cv::FONT_HERSHEY_PLAIN, 3,
                cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
    cv::putText(frame, aim_point_info, cv::Point(10, 80),
                cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(255, 255, 255), 2,
                cv::LINE_AA);  
    output_video.write(frame);
    if (g_config_info.show_mode) {
      cv::imshow("video", frame);
      cv::waitKey(1);
    }
  }
}
