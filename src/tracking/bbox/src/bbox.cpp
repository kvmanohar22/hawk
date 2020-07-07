#include <bbox/bbox.h>

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


tuple<std::vector<int>, std::vector<float>, std::vector<Rect>> Bbox::predict(Mat frame, bool draw=false) {
    Mat blob;
    // frame = imread(file_path, CV_LOAD_IMAGE_COLOR);
    Size inpSize(this->inpWidth > 0 ? this->inpWidth : frame.cols,
                     this->inpHeight > 0 ? this->inpHeight : frame.rows);
    blobFromImage(frame, blob, scale, inpSize, this->mean, this->swapRB, false);
    this->net.setInput(blob);

    if (this->net.getLayer(0)->outputNameToIndex("im_info") != -1) { // Faster-RCNN or R-FCN
        resize(frame, frame, inpSize);
        Mat imInfo = (Mat_<float>(1, 3) << inpSize.height, inpSize.width, 1.6f);
        this->net.setInput(imInfo, "im_info");
    }

    std::vector<Mat> outs;
    this->net.forward(outs, this->outNames);

    std::vector<int> classIds_;
    std::vector<float> confidences_;
    std::vector<Rect> boxes_;
    std::vector<int> indices_;

    this->postprocess(frame, outs, net, classIds_, confidences_, boxes_, indices_, draw);

    // Put efficiency information.
    std::vector<double> layersTimes;
    double freq = getTickFrequency() / 1000;
    double t = net.getPerfProfile(layersTimes) / freq;
    std::string label = format("Inference time: %.2f ms", t);
    std::cout << label << std::endl;

    std::vector<int> classIds;
    std::vector<float> confidences;
    std::vector<Rect> boxes;

    for(size_t i = 0; i < (int) indices_.size(); i++) {
        classIds.push_back(classIds_[i]);
        confidences.push_back(confidences_[i]);
        boxes.push_back(boxes_[i]);
    }


    if(draw) {
        imshow(this->kWinName, frame);
    }

    return make_tuple(classIds, confidences, boxes);

}

void Bbox::postprocess(Mat& frame, const std::vector<Mat>& outs, Net& net,
                    std::vector<int>& classIds, std::vector<float>& confidences,
                    std::vector<Rect>& boxes, std::vector<int>& indices, bool draw) {

    if (this->outLayerType == "DetectionOutput") {
        // Network produces output blob with a shape 1x1xNx7 where N is a number of
        // detections and an every detection is a vector of values
        // [batchId, classId, confidence, left, top, right, bottom]
        CV_Assert(outs.size() > 0);
        for (size_t k = 0; k < outs.size(); k++) {
            float* data = (float*)outs[k].data;
            for (size_t i = 0; i < outs[k].total(); i += 7) {
                float confidence = data[i + 2];
                if (confidence > this->confThreshold) {
                    int left   = (int)data[i + 3];
                    int top    = (int)data[i + 4];
                    int right  = (int)data[i + 5];
                    int bottom = (int)data[i + 6];
                    int width  = right - left + 1;
                    int height = bottom - top + 1;
                    if (width * height <= 1) {
                        left   = (int)(data[i + 3] * frame.cols);
                        top    = (int)(data[i + 4] * frame.rows);
                        right  = (int)(data[i + 5] * frame.cols);
                        bottom = (int)(data[i + 6] * frame.rows);
                        width  = right - left + 1;
                        height = bottom - top + 1;
                    }
                    classIds.push_back((int)(data[i + 1]) - 1);  // Skip 0th background class id.
                    boxes.push_back(Rect(left, top, width, height));
                    confidences.push_back(confidence);
                }
            }
        }
    } else if (this->outLayerType == "Region") {
        for (size_t i = 0; i < outs.size(); ++i) {
            // Network produces output blob with a shape NxC where N is a number of
            // detected objects and C is a number of classes + 4 where the first 4
            // numbers are [center_x, center_y, width, height]
            float* data = (float*)outs[i].data;
            for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols) {
                Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
                Point classIdPoint;
                double confidence;
                minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
                if (confidence > this->confThreshold) {
                    int centerX = (int)(data[0] * frame.cols);
                    int centerY = (int)(data[1] * frame.rows);
                    int width = (int)(data[2] * frame.cols);
                    int height = (int)(data[3] * frame.rows);
                    int left = centerX - width / 2;
                    int top = centerY - height / 2;

                    classIds.push_back(classIdPoint.x);
                    confidences.push_back((float)confidence);
                    boxes.push_back(Rect(left, top, width, height));
                }
            }
        }
    } else {
        CV_Error(Error::StsNotImplemented, "Unknown output layer type: " + this->outLayerType);
    }

    NMSBoxes(boxes, confidences, this->confThreshold, this->nmsThreshold, indices);
    for (size_t i = 0; i < indices.size(); ++i) {
        int idx = indices[i];
        Rect box = boxes[idx];
        if(draw) {
            drawPred(classIds[idx], confidences[idx], box.x, box.y,
                 box.x + box.width, box.y + box.height, frame);
        }
    }
}

void Bbox::drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat& frame) {
    rectangle(frame, Point(left, top), Point(right, bottom), Scalar(0, 255, 0));

    std::string label = format("%.2f", conf);
    if (!this->classes.empty()) {
        CV_Assert(classId < (int)this->classes.size());
        label = this->classes[classId] + ": " + label;
    }

    int baseLine;
    Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

    top = max(top, labelSize.height);
    rectangle(frame, Point(left, top - labelSize.height),
              Point(left + labelSize.width, top + baseLine), Scalar::all(255), FILLED);
    putText(frame, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.5, Scalar());
}










