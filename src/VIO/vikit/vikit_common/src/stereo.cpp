#include <vikit/stereo.h>

namespace vk {

StereoInitialization::StereoInitialization(
  AbstractCamera* cam0, AbstractCamera* cam1,
  Sophus::SE3& T_c1_c0)
  : cam0_(cam0),
    cam1_(cam1),
    T_c1_c0_(T_c1_c0)
{}

void StereoInitialization::setImages(
  const cv::Mat& imgl, const cv::Mat& imgr)
{
  imgl_ = imgl.clone();
  imgr_ = imgr.clone();
}

void StereoInitialization::extractORB(int flag, const cv::Mat& img)
{
  if(flag==0)
    (*mpORBextractorLeft)(img, cv::Mat(), mvKeys, mDescriptors);
  else
    (*mpORBextractorRight)(img, cv::Mat(), mvKeysRight, mDescriptorsRight);
}

void StereoInitialization::computeStereoMatches()
{
    mvuRight = vector<float>(N,-1.0f);
    mvDepth = vector<float>(N,-1.0f);

    const int thOrbDist = (ORBmatcher::TH_HIGH+ORBmatcher::TH_LOW)/2;

    const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

    //Assign keypoints to row table
    vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());

    for(int i=0; i<nRows; i++)
        vRowIndices[i].reserve(200);

    const int Nr = mvKeysRight.size();

    for(int iR=0; iR<Nr; iR++)
    {
        const cv::KeyPoint &kp = mvKeysRight[iR];
        const float &kpY = kp.pt.y;
        const float r = 2.0f*mvScaleFactors[mvKeysRight[iR].octave];
        const int maxr = ceil(kpY+r);
        const int minr = floor(kpY-r);

        for(int yi=minr;yi<=maxr;yi++)
            vRowIndices[yi].push_back(iR);
    }

    // Set limits for search
    const float minZ = mb;
    const float minD = 0;
    const float maxD = mbf/minZ;

    // For each left keypoint search a match in the right image
    vector<pair<int, int> > vDistIdx;
    vDistIdx.reserve(N);

    for(int iL=0; iL<N; iL++)
    {
        const cv::KeyPoint &kpL = mvKeys[iL];
        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;

        const vector<size_t> &vCandidates = vRowIndices[vL];

        if(vCandidates.empty())
            continue;

        const float minU = uL-maxD;
        const float maxU = uL-minD;

        if(maxU<0)
            continue;

        int bestDist = ORBmatcher::TH_HIGH;
        size_t bestIdxR = 0;

        const cv::Mat &dL = mDescriptors.row(iL);

        // Compare descriptor to right keypoints
        for(size_t iC=0; iC<vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = mvKeysRight[iR];

            if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                continue;

            const float &uR = kpR.pt.x;

            if(uR>=minU && uR<=maxU)
            {
                const cv::Mat &dR = mDescriptorsRight.row(iR);
                const int dist = ORBmatcher::DescriptorDistance(dL,dR);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }

        // Subpixel match by correlation
        if(bestDist<thOrbDist)
        {
            // coordinates in image pyramid at keypoint scale
            const float uR0 = mvKeysRight[bestIdxR].pt.x;
            const float scaleFactor = mvInvScaleFactors[kpL.octave];
            const float scaleduL = round(kpL.pt.x*scaleFactor);
            const float scaledvL = round(kpL.pt.y*scaleFactor);
            const float scaleduR0 = round(uR0*scaleFactor);

            // sliding window search
            const int w = 5;
            cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
            IL.convertTo(IL,CV_32F);
            IL = IL - IL.at<float>(w,w) *cv::Mat::ones(IL.rows,IL.cols,CV_32F);

            int bestDist = INT_MAX;
            int bestincR = 0;
            const int L = 5;
            vector<float> vDists;
            vDists.resize(2*L+1);

            const float iniu = scaleduR0+L-w;
            const float endu = scaleduR0+L+w+1;
            if(iniu<0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                continue;

            for(int incR=-L; incR<=+L; incR++)
            {
                cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
                IR.convertTo(IR,CV_32F);
                IR = IR - IR.at<float>(w,w) *cv::Mat::ones(IR.rows,IR.cols,CV_32F);

                float dist = cv::norm(IL,IR,cv::NORM_L1);
                if(dist<bestDist)
                {
                    bestDist =  dist;
                    bestincR = incR;
                }

                vDists[L+incR] = dist;
            }

            if(bestincR==-L || bestincR==L)
                continue;

            // Sub-pixel match (Parabola fitting)
            const float dist1 = vDists[L+bestincR-1];
            const float dist2 = vDists[L+bestincR];
            const float dist3 = vDists[L+bestincR+1];

            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

            if(deltaR<-1 || deltaR>1)
                continue;

            // Re-scaled coordinate
            float bestuR = mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);

            float disparity = (uL-bestuR);

            if(disparity>=minD && disparity<maxD)
            {
                if(disparity<=0)
                {
                    disparity=0.01;
                    bestuR = uL-0.01;
                }
                mvDepth[iL]=mbf/disparity;
                mvuRight[iL] = bestuR;
                vDistIdx.push_back(pair<int,int>(bestDist,iL));
            }
        }
    }

    sort(vDistIdx.begin(),vDistIdx.end());
    const float median = vDistIdx[vDistIdx.size()/2].first;
    const float thDist = 1.5f*1.4f*median;

    for(int i=vDistIdx.size()-1;i>=0;i--)
    {
        if(vDistIdx[i].first<thDist)
            break;
        else
        {
            mvuRight[vDistIdx[i].second]=-1;
            mvDepth[vDistIdx[i].second]=-1;
        }
    }
}

void StereoInitialization::rectifyImage(
    const cv::Mat& K, const cv::Mat& D,
    const cv::Mat& R, const cv::Mat& P, 
    const cv::Mat& raw, cv::Mat& rectified)
{
  cv::Mat undist_map1, undist_map2;
  cv::initUndistortRectifyMap(K, D, R, P, raw.size(), CV_16SC2, undist_map1, undist_map2);
  cv::remap(raw, rectified, undist_map1, undist_map2, cv::INTER_LINEAR);
}

Eigen::Vector3d StereoInitialization::UnprojectStereo(const int &i)
{
  const float z = mvDepth[i];
  const float u = mvKeys[i].pt.x;
  const float v = mvKeys[i].pt.y;
  const float x = (u-cx)*z*invfx;
  const float y = (v-cy)*z*invfy;
  Eigen::Vector3d point(x, y, z);
  return point;
}

void StereoInitialization::triangulate()
{
  size_t count = 0;
  if(N>500)
  {
    for(int i=0; i<N;i++)
    {
      float z = mvDepth[i];
      if(z>0)
      {
        Eigen::Vector3d xyz = UnprojectStereo(i);
        ++count;
      }
    }
  }
  std::cout << "Initialized map with " << count << " 3D points" << std::endl;
}

void StereoInitialization::initialize()
{
  // initialization
  mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
  mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();

  // Rectification parameters
  cv::Mat R1, R2, P1, P2, Q;
  const cv::Mat K1 = dynamic_cast<vk::PinholeCamera*>(cam0_)->cvK();
  const cv::Mat D1 = dynamic_cast<vk::PinholeCamera*>(cam0_)->cvD();
  const cv::Mat K2 = dynamic_cast<vk::PinholeCamera*>(cam1_)->cvK();
  const cv::Mat D2 = dynamic_cast<vk::PinholeCamera*>(cam1_)->cvD();
  const Eigen::Matrix<double, 3, 3> R = T_c1_c0_.rotation_matrix();
  const Eigen::Vector3d t = T_c1_c0_.translation();

  cv::Mat R_c1_c0(3,3,CV_64F);
  for(size_t i=0;i<3;++i)
    for(size_t j=0;j<3;++j)
      R_c1_c0.at<double>(i,j) = R.coeff(i,j);

  cv::Mat t_c1_c0(3,1,CV_64F);
  for(size_t i=0;i<3;++i)
    t_c1_c0.at<double>(i) = t(i);

  cv::stereoRectify(K1, D1, K2, D2, imgl_.size(), R_c1_c0, t_c1_c0, R1, R2, P1, P2, Q);

  // First rectify the images
  cv::Mat imgl_rect, imgr_rect;
  rectifyImage(K1, D1, R1, P1, imgl_, imgl_rect);
  rectifyImage(K2, D2, R2, P2, imgr_, imgr_rect);

  // set intrinsic parameters
  fx = P1.at<double>(0, 0);
  fy = P1.at<double>(1, 1);
  cx = P1.at<double>(0, 2);
  cy = P1.at<double>(1, 2);
  invfx = 1.0/fx;
  invfy = 1.0/fy;
  mbf = P2.at<double>(0, 3);
  mb  = mbf / fx;

  int nFeatures = 500;
  float fScaleFactor = 1.2;
  int nLevels = 8;
  int fIniThFAST = 20;
  int fMinThFAST = 7;
  mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
  mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

  boost::thread thread_l(&StereoInitialization::extractORB, this, 0, imgl_);
  boost::thread thread_r(&StereoInitialization::extractORB, this, 0, imgr_);
  thread_l.join();
  thread_r.join();

  N = mvKeys.size();
  std::cout << "Extracted " << N << " features in the left image" << std::endl;

  cv::Mat imgc;
  cv::cvtColor(imgl_, imgc, cv::COLOR_GRAY2BGR);
  for(size_t i=0; i<N; ++i) {
    const float u = mvKeys[i].pt.x;
    const float v = mvKeys[i].pt.y;
    cv::rectangle(imgc, cv::Point2f(u-1, v-1), cv::Point2f(u+1, v+1), cv::Scalar(0,255,0), cv::FILLED);
  }
  cv::imshow("left", imgc);
  cv::waitKey(0);

  computeStereoMatches();

  triangulate();
}

} // namespace vk
