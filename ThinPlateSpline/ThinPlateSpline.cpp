#include "ThinPlateSpline.hpp"
#include <Eigen/QR>


ThinPlateSpline::ThinPlateSpline(const PointList &src, const PointList &dst)
    : mSrcPoints(src), mDstPoints(dst) {}

void ThinPlateSpline::solve() {

  if (mSrcPoints.size() != mDstPoints.size())
    return;

  const int num(int(mSrcPoints.size()));
  const int rows(num + 3 + 1);

  // Create L Matrix
  double lambda = 10000;
  mL = Eigen::MatrixXd::Zero(rows, rows);

  for (int i(0); i < num; ++i)
  {
      mL(i, i) = lambda;
  }

  for (int i(0); i < num; ++i) {

    int j(i + 1);

    for (; j < num; ++j)
      mL(i, j) = mL(j, i) = radialBasis(
          (mSrcPoints[std::size_t(i)] - mSrcPoints[std::size_t(j)]).norm());

    mL(j, i) = mL(i, j) = 1.0;
    ++j;

    for (int posElm(0); j < rows; ++posElm, ++j)
      mL(j, i) = mL(i, j) = mSrcPoints[std::size_t(i)][posElm];
  }

  // Create Y Matrix
  Eigen::MatrixXd Y = Eigen::MatrixXd::Zero(rows, 3);

  for (int i(0); i < num; ++i)
    Y.row(i) = mDstPoints[std::size_t(i)];

  // Solve L W^T = Y as W^T = L^-1 Y
  mW = mL.colPivHouseholderQr().solve(Y);
  std::cout << mL << std::endl;
  std::cout << mW << std::endl;
}

Eigen::Vector3d ThinPlateSpline::interpolate(const Eigen::Vector3d &p) const {

  Eigen::Vector3d res = Eigen::Vector3d::Zero();
  int i(0);

  for (; i < mW.rows() - (3 + 1); ++i) {
    double rb = radialBasis((mSrcPoints[std::size_t(i)] - p).norm());
    res += mW.row(i) * rb;
  }

  res += mW.row(i);
  i++;

  for (int j(0); j < 3; ++j, ++i)
    res += mW.row(i) * p[j];

  return res;
}

cv::Mat flow2RGB(float* flow_x, float* flow_y, int h, int w)
{
    cv::Mat mat_flow_x(h, w, CV_32FC1, flow_x);
    cv::Mat mat_flow_y(h, w, CV_32FC1, flow_y);
    // visualization
    cv::Mat magnitude, angle, magn_norm;
    cartToPolar(mat_flow_x, mat_flow_y, magnitude, angle, true);
    cv::normalize(magnitude, magn_norm, 0.0f, 1.0f, cv::NORM_MINMAX);
    angle *= ((1.f / 360.f) * (180.f / 255.f));
    //build hsv image
    cv::Mat _hsv[3], hsv, hsv8, bgr;
    _hsv[0] = angle;
    _hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
    _hsv[2] = magn_norm;
    cv::merge(_hsv, 3, hsv);
    hsv.convertTo(hsv8, CV_8U, 255.0);
    cv::cvtColor(hsv8, bgr, cv::COLOR_HSV2BGR);
    
    return bgr;
}

cv::Mat ThinPlateSpline::getWarpFlow(int height, int width)
{
    float* flow_x = new float[height * width];
    float* flow_y = new float[height * width];
    for (int i = 0; i < height; ++i)
    {
        for (int j = 0; j < width; ++j)
        {
            Eigen::Vector3d p_in(j, i, 0);
            Eigen::Vector3d p_warp = interpolate(p_in);
            flow_x[i * width + j] = p_warp(0) - p_in(0);
            flow_y[i * width + j] = p_warp(1) - p_in(1);
            //printf("(%d, %d) -> (%.2f, %.2f), shift=(%.2f, %.2f)\n", j, i, p_warp(0), p_warp(1), flow_x[i * width + j], flow_y[i * width + j]);
        }
    }
    cv::Mat bgr = flow2RGB(flow_x, flow_y, height, width);

    delete[] flow_x;
    delete[] flow_y;
    return bgr;
}
