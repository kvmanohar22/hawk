#ifndef ORBMATCHER_H
#define ORBMATCHER_H

#include<vector>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

namespace vk {

class ORBmatcher
{    
public:

  static const int TH_LOW;
  static const int TH_HIGH;
  static const int HISTO_LENGTH;

  ORBmatcher(float nnratio=0.6, bool checkOri=true);

  // Computes the Hamming distance between two ORB descriptors
  static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

protected:
  float mfNNratio;
  bool mbCheckOrientation;

};

}

#endif
