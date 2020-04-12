#include <svo/stereo.h>
#include <svo/feature_detection.h>
#include <svo/config.h>
#include <svo/feature.h>
#include <svo/point.h>
#include <vikit/homography.h>

namespace svo {

StereoInitialization::StereoInitialization(
  vk::AbstractCamera* caml, vk::AbstractCamera* camr,
  Sophus::SE3& T_cl_cr,
  bool verbose) :
    cam_l_(caml),
    cam_r_(camr),
    T_cl_cr_(T_cl_cr),
    verbose_(verbose)
{}

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
  const float ratio_thresh = 0.87f;
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
  // 1. Extract features.
  cv::Mat descriptors_l, descriptors_r;
  vector<Vector3d> f_l, f_r;
  vector<cv::KeyPoint> kps_l, kps_r;

  boost::thread thread_l(&StereoInitialization::detectFeatures,
                         this,
                         ref_frame_->img_pyr_,
                         std::ref(kps_l),
                         std::ref(f_l),
                         std::ref(descriptors_l));
  boost::thread thread_r(&StereoInitialization::detectFeatures,
                         this,
                         ref_frame_->img_pyr_right_,
                         std::ref(kps_r),
                         std::ref(f_r),
                         std::ref(descriptors_r));
  thread_l.join();
  thread_r.join();

  // bearing vectors in cur image should be wrt second camera
  for(size_t i=0; i<f_r.size(); ++i) {
    Vector2d px(kps_r[i].pt.x, kps_r[i].pt.y);
    f_r[i] = ref_frame_->camR_->cam2world(px);
  }

  if(verbose_) {
    std::cout << "Number of features detected (l) = " << kps_l.size() << "\n";
    std::cout << "Number of features detected (r) = " << kps_r.size() << "\n";
    // ref image
    cv::Mat fts_img_l;
    cv::Mat imgl = ref_frame_->img();
    cv::cvtColor(imgl, fts_img_l, cv::COLOR_GRAY2BGR);
    for(size_t i=0; i<kps_l.size(); ++i) {
      const auto px = kps_l[i].pt;
      cv::rectangle(fts_img_l, cv::Point2f(px.x-2, px.y-2), cv::Point2f(px.x+2, px.y+2), cv::Scalar(0,255,0), cv::FILLED);
    }
    // cur image
    cv::Mat fts_img_r;
    cv::Mat imgr = ref_frame_->imgRight();
    cv::cvtColor(imgr, fts_img_r, cv::COLOR_GRAY2BGR);
    for(size_t i=0; i<kps_r.size(); ++i) {
      const auto px = kps_r[i].pt;
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
    cv::drawMatches(imgl, kps_l, imgr, kps_r, good_matches, img_matches, cv::Scalar::all(-1),
      cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    cv::imshow("matches", img_matches);
    cv::waitKey(0);
  }

  f_l_.reserve(good_matches.size());
  f_r_.reserve(good_matches.size());
  px_l_.reserve(good_matches.size());
  px_r_.reserve(good_matches.size());
  for(size_t i=0; i<good_matches.size(); ++i)
  {
    // TODO: Is the association right?
    int l_idx = good_matches[i].queryIdx;
    int r_idx = good_matches[i].trainIdx;

    f_l_.push_back(f_l[l_idx]);
    f_r_.push_back(f_r[r_idx]);
    px_l_.push_back(kps_l[l_idx].pt);
    px_r_.push_back(kps_r[r_idx].pt);
  }

  if(verbose_) {
    std::cout << "Number of matches detected = " << good_matches.size() << "\n";
    cv::Mat img_matches;
    cv::Mat imgl = ref_frame_->img();
    cv::Mat imgr = ref_frame_->imgRight();

    cv::Mat newimg(imgl.rows, imgl.cols*2, imgl.type());
    imgl.copyTo(newimg.rowRange(0, imgl.rows).colRange(0, imgl.cols));
    imgr.copyTo(newimg.rowRange(0, imgl.rows).colRange(imgl.cols, 2*imgl.cols));
    cv::cvtColor(newimg, newimg, cv::COLOR_GRAY2BGR);
    for(size_t i=0; i<px_l_.size(); ++i) {
      cv::Point2f newpt = px_r_[i]+cv::Point2f(imgl.cols, 0);
      cv::line(newimg, px_l_[i], newpt, cv::Scalar(0, 255, 0), 1);
    }
    cv::imshow("matches", newimg);
    cv::waitKey(0);
  }
/*
  vector<Vector2d > uv_ref(f_l_.size());
  vector<Vector2d > uv_cur(f_r_.size());
  for(size_t i=0, i_max=f_l_.size(); i<i_max; ++i)
  {
    uv_ref[i] = vk::project2d(f_l_[i]);
    uv_cur[i] = vk::project2d(f_r_[i]);
  }
  double focal_length = ref_frame_->cam_->errorMultiplier2();
  double reprojection_threshold = Config::poseOptimThresh();
  vk::Homography Homography(uv_ref, uv_cur, focal_length, reprojection_threshold);
  Homography.computeSE3fromMatches();
  vector<int> outliers, inliers;
  vector<Vector3d> xyz_in_r;
  vk::computeInliers(f_r_, f_l_,
                     Homography.T_c2_from_c1.rotation_matrix(), Homography.T_c2_from_c1.translation(),
                     reprojection_threshold, focal_length,
                     xyz_in_r, inliers, outliers);
  const SE3 T_cur_from_ref = Homography.T_c2_from_c1;
  const SE3 T_ref_from_cur = T_cur_from_ref.inverse();

  std::cout << "H t = " << T_cur_from_ref.translation().transpose() << std::endl;
  std::cout << "C t = " << T_cl_cr_.inverse().translation().transpose() << std::endl;
*/


  vector<int> outliers, inliers;
  vector<Vector3d> xyz_in_r; // right image
  double tot_error = vk::computeInliers(
                     f_r_,  // right
                     f_l_,  // left
                     T_cl_cr_.rotation_matrix(),
                     T_cl_cr_.translation(),
                     Config::poseOptimThresh(),
                     ref_frame_->cam_->errorMultiplier2(),
                     xyz_in_r,
                     inliers,
                     outliers);
  std::cout << "total error = " << tot_error << "#inliers = " << inliers.size() << std::endl;

  int count=0;
  double el=0;
  double er=0;
  for(vector<int>::iterator it=inliers.begin(); it!=inliers.end(); ++it)
  {
    Vector2d px_r(px_r_[*it].x, px_r_[*it].y);
    Vector2d px_l(px_l_[*it].x, px_l_[*it].y);
    if(ref_frame_->camR_->isInFrame(px_r.cast<int>(), 10) && ref_frame_->cam_->isInFrame(px_l.cast<int>(), 10) && xyz_in_r[*it].z() > 0)
    {
      Vector3d pos_l = T_cl_cr_.inverse() * xyz_in_r[*it];
      Point* new_point = new Point(pos_l);

      Feature* ftr_left(new Feature(ref_frame_.get(), new_point, px_l, f_l_[*it], 0));
      ref_frame_->addFeature(ftr_left);
      new_point->addFrameRef(ftr_left);
      ++count;
/*
      const Vector2d uv_l = ref_frame_->cam_->world2cam(pos_l);
      const Vector2d uv_r = ref_frame_->cam_->world2cam(xyz_in_r[*it]);
      el += (uv_l-px_l).norm();
      er += (uv_r-px_r).norm();
*/
      er += vk::reprojError(f_r_[*it], xyz_in_r[*it], ref_frame_->cam_->errorMultiplier2());
      el += vk::reprojError(f_l_[*it], T_cl_cr_.inverse() * xyz_in_r[*it], ref_frame_->cam_->errorMultiplier2());
      // el += ref_frame_->cam_->errorMultiplier2() * (vk::project2d(f_l_[*it])-vk::project2d(T_cl_cr_ * xyz_in_r[*it])).norm();
      // er += ref_frame_->cam_->errorMultiplier2() * (vk::project2d(f_r_[*it])-vk::project2d(xyz_in_r[*it])).norm();
    }
  }
  if(verbose_) {
    std::cout << "Triangulated initial map with " << count << " points\n"
              << "total el = " << el <<"\n"
              << "avg el = " << el/count <<"\n"
              << "total er = " << er <<"\n"
              << "avg er = " << er/count <<"\n"
              << "total = " << el+er << "\n"
              << "avg total = " << (el+er)/count 
              << "count = " << count << std::endl;
  }

  return true;
}

} // namespace svo
