#include <vector>
#include <string>
#include <fstream>

#include <benchmark/benchmark.h>
#include <opencv2/highgui/highgui.hpp>
#include <bbox/bbox.h>

#define USE_RANDOM_IMAGE

static void OpenCV_DNN_readnet(benchmark::State& state) {
  // Perform setup here
  vector<string> classes;
  // get labels of all classes
  string classesFile = "/home/sipah00/hawk_ws/src/tracking/coco.names";
  ifstream ifs(classesFile.c_str());
  string line;
  while(getline(ifs, line)) classes.push_back(line);
  cout << "Total Number of classes: " << classes.size() << endl;

  int inpW = 416;
  int inpH = 416;
  float confThreshold = 0.5;
  float nmsThreshold = 0.4;
  bool swapRB = true;
  auto mean = cv::Scalar(0,0,0);
  
  string kWinName = "bbox_test";

  Bbox box(model, config, "yolo");
  box.setParams(inpW, inpH, classes, confThreshold, nmsThreshold, mean, swapRB, kWinName);


  #ifdef USE_RANDOM_IMAGE
  cv::Mat img(cv::Size(416, 416), CV_32FC3);
  cv::randu(img, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255))
  #else
  string file = "";
  cv::Mat img = cv::imread(file);
  #endif

  vector<int> classIds;
  vector<float> confidences;
  vector<cv::Rect> boxes;

  for (auto _ : state) {
    // This code gets timed
    tie (classIds, confidences, boxes) = box.predict(img, false);
  }
}
// Register the function as a benchmark
BENCHMARK(OpenCV_DNN_readnet);
// Run the benchmark
BENCHMARK_MAIN();
