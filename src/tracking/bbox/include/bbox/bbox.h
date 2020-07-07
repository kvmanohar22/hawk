#ifndef BBOX_H
#define BBOX_H

#include <fstream>
#include <sstream>
#include <iostream>
#include <tuple>


// Required for dnn modules.
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
using namespace cv::dnn;

class Bbox {
    private:
        float confThreshold;
        float nmsThreshold;
        vector<string> classes;
        float scale;
        Scalar mean;
        bool swapRB;
        int inpWidth;
        int inpHeight;
        Net net;
        Mat frame, blob;
        string kWinName;
        std::vector<int> outLayers;
        std::string outLayerType;
        std::vector<String> outNames;
    public:
        Bbox(string modelPath, string configPath, string framework,
                        int backend=0, int target=0) {
                net = readNet(modelPath, configPath, framework);
                net.setPreferableBackend(backend);
                net.setPreferableTarget(target);
                outLayers = net.getUnconnectedOutLayers();
                outLayerType = net.getLayer(outLayers[0])->type;
                outNames = net.getUnconnectedOutLayersNames();
            }

        void setParams(int _inpW, int _inpH, const vector<string>& _classes,
                            float _confThreshold, float _nmsThreshold, Scalar _mean, bool _swapRB, string _kWinName="BBOX Output") {
                inpWidth = _inpW;
                inpHeight = _inpH;
                classes = _classes;
                confThreshold = _confThreshold;
                nmsThreshold = _nmsThreshold;
                mean = _mean;
                swapRB = _swapRB;
                kWinName = _kWinName;
        }
        std::tuple<std::vector<int>, std::vector<float>, std::vector<Rect>> predict(Mat frame, bool draw);

        void postprocess(Mat& frame, const vector<Mat>& out, Net& net, std::vector<int>& classIds,
                            std::vector<float>& confidences, std::vector<Rect>& boxes, vector<int>& indices, bool draw);

        void drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat& frame);
};

#endif // BBOX_H


