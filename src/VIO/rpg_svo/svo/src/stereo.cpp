#include <svo/stereo.h>
#include <svo/feature_detection.h>
#include <svo/config.h>
#include <svo/feature.h>

namespace svo {

StereoInitialization::StereoInitialization(
  vk::AbstractCamera* cam0, vk::AbstractCamera* cam1,
  Sophus::SE3& T_c1_c0,
  bool verbose) :
    cam0_(cam0),
    cam1_(cam1),
    T_c1_c0_(T_c1_c0),
    verbose_(verbose)
{
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
}

StereoInitialization::~StereoInitialization()
{
}

void StereoInitialization::detectFeatures(
  const ImgPyr& img_pyr,
  vector<cv::KeyPoint>& kps,
  vector<Vector3d>& f_vec,
  cv::Mat& descriptors)
{
  // extract FAST corners
  Features new_features;
  feature_detection::FastDetector detector(
      ref_frame_->img().cols, ref_frame_->img().rows, Config::gridSize(), Config::nPyrLevels());
  detector.detect(ref_frame_.get(), img_pyr, Config::triangMinCornerScore(), new_features);

  kps.clear(); kps.reserve(new_features.size());
  f_vec.clear(); f_vec.reserve(new_features.size());
  std::for_each(new_features.begin(), new_features.end(), [&](Feature* ftr){
    kps.push_back(cv::KeyPoint(cv::Point2f(ftr->px[0], ftr->px[1]), 1.0));
    f_vec.push_back(ftr->f);
    delete ftr;
  });

  // extract descriptors
  const cv::Mat img = img_pyr[0];
  cv::Ptr<cv::BRISK> descriptor = cv::BRISK::create();
  descriptor->compute(img, kps, descriptors);
}

void StereoInitialization::match(cv::Mat& descriptors_l, cv::Mat& descriptors_r,
  std::vector<cv::DMatch>& good_matches)
{
  std::vector<std::vector<cv::DMatch> > knn_matches;
  cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
  matcher->knnMatch(descriptors_l, descriptors_r, knn_matches, 2);

  // filter the matches
  const float ratio_thresh = 0.7f;
  for(size_t i = 0; i < knn_matches.size(); i++)
  {
    if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
    {
      good_matches.push_back(knn_matches[i][0]);
    }
  }
}

bool StereoInitialization::initialize()
{
  // 1. Extract features. TODO: use threading
  cv::Mat descriptors_l, descriptors_r;
  vector<Vector3d> f_ref, f_cur;
  vector<cv::KeyPoint> kps_ref, kps_cur;
  detectFeatures(ref_frame_->img_pyr_,       kps_ref, f_ref, descriptors_l);
  detectFeatures(ref_frame_->img_pyr_right_, kps_cur, f_cur, descriptors_r);

  if(verbose_) {
    std::cout << "Number of features detected (ref) = " << kps_ref.size() << "\n";
    std::cout << "Number of features detected (cur) = " << kps_cur.size() << "\n";
    // ref image
    cv::Mat fts_img_l;
    cv::Mat imgl = ref_frame_->img();
    cv::cvtColor(imgl, fts_img_l, cv::COLOR_GRAY2BGR);
    for(size_t i=0; i<kps_ref.size(); ++i) {
      const auto px = kps_ref[i].pt;
      cv::rectangle(fts_img_l, cv::Point2f(px.x-2, px.y-2), cv::Point2f(px.x+2, px.y+2), cv::Scalar(0,255,0), cv::FILLED);
    }
    // cur image
    cv::Mat fts_img_r;
    cv::Mat imgr = ref_frame_->imgRight();
    cv::cvtColor(imgr, fts_img_r, cv::COLOR_GRAY2BGR);
    for(size_t i=0; i<kps_cur.size(); ++i) {
      const auto px = kps_cur[i].pt;
      cv::rectangle(fts_img_r, cv::Point2f(px.x-2, px.y-2), cv::Point2f(px.x+2, px.y+2), cv::Scalar(0,255,0), cv::FILLED);
    }
    cv::imshow("features left", fts_img_l);
    cv::imshow("features right", fts_img_r);
    cv::waitKey(0);
  }

  // 2. Match & filter
  std::vector<cv::DMatch> good_matches;
  match(descriptors_l, descriptors_r, good_matches);

  if(verbose_) {
    std::cout << "Number of matches detected = " << good_matches.size() << "\n";
    cv::Mat img_matches;
    cv::Mat imgl = ref_frame_->img();
    cv::Mat imgr = ref_frame_->imgRight();
    cv::drawMatches(imgl, kps_ref, imgr, kps_cur, good_matches, img_matches, cv::Scalar::all(-1),
      cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    cv::imshow("matches", img_matches);
    cv::waitKey(0);
  }

  f_ref_.reserve(good_matches.size());
  f_cur_.reserve(good_matches.size());
  px_ref_.reserve(good_matches.size());
  px_cur_.reserve(good_matches.size());
  for(size_t i=0; i<good_matches.size(); ++i)
  {
    int l_idx = good_matches[i].queryIdx;
    int r_idx = good_matches[i].trainIdx;

    f_ref_.push_back(f_ref[l_idx]);
    f_cur_.push_back(f_cur[r_idx]);
    px_ref_.push_back(kps_ref[l_idx].pt);
    px_cur_.push_back(kps_cur[r_idx].pt);
  }

  vector<int> outliers, inliers;
  vector<Vector3d> xyz_in_cur;
  vk::computeInliers(f_cur_, f_ref_,
                     T_c1_c0_.rotation_matrix(), T_c1_c0_.translation(),
                     ref_frame_->cam_->errorMultiplier2(), Config::poseOptimThresh(),
                     xyz_in_cur, inliers, outliers);

  int count=0;
  for(vector<int>::iterator it=inliers.begin(); it!=inliers.end(); ++it)
  {
    Vector2d px_cur(px_cur_[*it].x, px_cur_[*it].y);
    Vector2d px_ref(px_ref_[*it].x, px_ref_[*it].y);
    if(ref_frame_->cam_->isInFrame(px_cur.cast<int>(), 10) && ref_frame_->cam_->isInFrame(px_ref.cast<int>(), 10) && xyz_in_cur[*it].z() > 0)
    {
      // Vector3d pos = T_world_cur * (xyz_in_cur_[*it]*scale);
      // Point* new_point = new Point(pos);

      // Feature* ftr_cur(new Feature(frame_cur.get(), new_point, px_cur, f_cur_[*it], 0));
      // frame_cur->addFeature(ftr_cur);
      // new_point->addFrameRef(ftr_cur);

      // Feature* ftr_ref(new Feature(frame_ref_.get(), new_point, px_ref, f_ref_[*it], 0));
      // frame_ref_->addFeature(ftr_ref);
      // new_point->addFrameRef(ftr_ref);
      ++count;
    }
  }
  if(verbose_) {
    std::cout << "Triangulated initial map with " << count << " points\n";
  }


  return true;
}

} // namespace svo
