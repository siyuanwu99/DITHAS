#include "dithas/se3_solver.h"
namespace dithas {
namespace solver {

/**
 * @brief
 *
 * @param data
 * @param gt ground truth position in world frame
 * @param eps [TODO:parameter]
 * @return true: success, false: failed
 */
bool SE3Solver::solve(const std::vector<Vec> &data, const std::vector<Vec> &gt, double eps) {
  bool success = false;
  if (data.size() != gt.size()) {
    std::cout << "[SE3Solver] data size not equal to gt size!" << std::endl;
  }

  Sophus::SE3d soph(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());

  problem_ = new ceres::Problem;

  for (int i = 0; i < data.size(); ++i) {
    ceres::CostFunction *cost_function = PointToPointSquareError::Create(data[i], gt[i]);
    problem_->AddResidualBlock(cost_function, nullptr, soph.data());
  }

  if (problem_->NumResidualBlocks() > 3) {
    ceres::Solver::Summary summary;
    ceres::Solve(options_, problem_, &summary);
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "Result: \n" << soph.matrix() << std::endl;

    tf_ = soph;

    // TODO: check the validity of the result

    success = true;
  }

  return success;
}
}  // namespace solver

}  // namespace dithas
