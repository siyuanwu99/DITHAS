#ifndef DITHAS_SE3_SOLVER_H
#define DITHAS_SE3_SOLVER_H
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <sophus/se3.hpp>
#include <vector>

namespace dithas {
namespace solver {

typedef Eigen::Vector3d Vec;
typedef Eigen::Matrix3d Rot;
typedef Sophus::SE3d    SE3d;

struct Residual {
  Residual(const Vec &data, const Vec &gt) : data_(data), gt_(gt) {}

  // compute residual
  template <typename T>
  bool operator()(const T *const tf, T *residual) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 3>> R(tf);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t(tf + 9);
    Eigen::Map<Eigen::Matrix<T, 3, 1>>       residual_map(residual);

    residual_map = R * data_.cast<T>() + t - gt_.cast<T>();
    return true;
  }

  static ceres::CostFunction *Create(const Vec &data, const Vec &gt) {
    return (new ceres::AutoDiffCostFunction<Residual, 3, 12>(new Residual(data, gt)));
  }

  const Vec data_;
  const Vec gt_;
};

struct PointToPointSquareError {
  const Vec p_src_;  // input points in local frame
  const Vec p_glb_;  // ground truth points in global frame
  PointToPointSquareError(const Vec &p_src, const Vec &p_glb) : p_src_(p_src), p_glb_(p_glb) {}

  template <typename T>
  bool operator()(const T *const x, T *residual) const {
    Eigen::Matrix<T, 3, 1> src;
    src << p_src_.cast<T>();

    Sophus::SE3<T> se3_tf = Eigen::Map<const Sophus::SE3<T>>(x);

    Eigen::Matrix<T, 3, 1> glb = se3_tf.unit_quaternion() * src + se3_tf.translation();

    residual[0] = glb[0] - p_glb_.cast<T>()[0];
    residual[1] = glb[1] - p_glb_.cast<T>()[1];
    residual[2] = glb[2] - p_glb_.cast<T>()[2];
    return true;
  }

  static ceres::CostFunction *Create(const Vec &p_src, const Vec &p_glb) {
    return (new ceres::AutoDiffCostFunction<PointToPointSquareError, 3, 7>(
        new PointToPointSquareError(p_src, p_glb)));
  }
};

class SE3Solver {
 public:
  SE3Solver() {
    options_.max_num_iterations           = 100;
    options_.linear_solver_type           = ceres::DENSE_QR;
    options_.minimizer_progress_to_stdout = false;
    options_.minimizer_type               = ceres::TRUST_REGION;
  }

  ~SE3Solver();

  bool solve(const std::vector<Vec> &data, const std::vector<Vec> &gt, double eps = 1e-6);
  SE3d getSophusSE3d() const { return tf_; }
  Vec  getTranslation() const { return tf_.translation(); }
  Rot  getRotation() const { return tf_.rotationMatrix(); }

 protected:
  Vec  vec_;
  Rot  rot_;
  SE3d tf_;  // tf from self to global in lie group

  /* optimization problem */
  ceres::Problem        *problem_;
  ceres::Solver::Options options_;
};

}  // namespace solver
}  // namespace dithas

#endif  // DITHAS_SE3_SOLVER_H
