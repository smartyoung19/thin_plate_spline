#pragma once
#include <opencv.hpp>
#include <Eigen/Core>
#include <Eigen/StdVector>

typedef std::vector<Eigen::Vector3d,
    Eigen::aligned_allocator<Eigen::Vector3d>>
    PointList;

class ThinPlateSpline {
public:
  
  ThinPlateSpline() {}
  ThinPlateSpline(const PointList &src, const PointList &dst);
  ~ThinPlateSpline() {}

  /* Solve */
  void solve();

  /* Interpolate */
  Eigen::Vector3d interpolate(const Eigen::Vector3d &p) const;

  cv::Mat getWarpFlow(int height, int width);

  /* Source Points */
  const PointList &srcPoints() const { return mSrcPoints; }

  /* Set Source Points */
  void setSrcPoints(const PointList &points) { mSrcPoints = points; }

  /* Destination Points */
  const PointList &dstPoints() const { return mDstPoints; }

  /* Set Destination Points */
  void setDstPoints(const PointList &points) { mDstPoints = points; }

protected:
  /* Radial Basis Function */
  static inline double radialBasis(double r) {
    return r == 0.0 ? r : r * r * log(r);
  }

  /* Data */
  PointList mSrcPoints;
  PointList mDstPoints;
  Eigen::MatrixXd mW;
  Eigen::MatrixXd mL;
};
