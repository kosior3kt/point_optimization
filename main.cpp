#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <iostream>
#include <vector>

struct DistanceCostFunction {
  DistanceCostFunction(double target_length) : target_length(target_length) {}

  template <typename T>
  bool operator()(const T *const p1, const T *const p2, T *residual) const {
    T delta[2] = {p2[0] - p1[0], p2[1] - p1[1]};
    T length_squared = delta[0] * delta[0] + delta[1] * delta[1];
    residual[0] = length_squared - target_length * target_length;

    return true;
  }

  // Define Jacobians
  template <typename T>
  bool operator()(const T *const p1, const T *const p2, T *residual,
                  T *jacobian_p1, T *jacobian_p2) const {
    T delta[2] = {p2[0] - p1[0], p2[1] - p1[1]};
    T length_squared = delta[0] * delta[0] + delta[1] * delta[1];
    residual[0] = length_squared - target_length * target_length;

    if (jacobian_p1 != nullptr) {
      jacobian_p1[0] = -2 * delta[0];
      jacobian_p1[1] = -2 * delta[1];
    }

    if (jacobian_p2 != nullptr) {
      jacobian_p2[0] = 2 * delta[0];
      jacobian_p2[1] = 2 * delta[1];
    }

    return true;
  }

  double target_length;
};

struct LoopClosureCostFunction {
  template <typename T>
  bool operator()(const T *const p1, const T *const p2, T *residual) const {
    residual[0] = p1[0] - p2[0];
    residual[1] = p1[1] - p2[1];
    return true;
  }

  // Define Jacobian
  template <typename T>
  bool operator()(const T *const p1, const T *const p2, T *residual,
                  T *jacobian_p1, T *jacobian_p2) const {
    if (jacobian_p1 != nullptr) {
      jacobian_p1[0] = T(1);
      jacobian_p1[1] = T(0);
    }

    if (jacobian_p2 != nullptr) {
      jacobian_p2[0] = T(-1);
      jacobian_p2[1] = T(0);
    }

    return true;
  }
};

int main() {
  size_t N = 5;
  std::vector<Eigen::Vector2d> initial_positions(N);

  // Initialize points in a circular arrangement
  for (size_t i = 0; i < N; ++i) {
    double theta = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(N);
    initial_positions[i] << std::cos(theta), std::sin(theta);
  }

  std::vector<double> target_lengths = {
      2.0, 2.5, 1.0, 3.0}; // Replace with your target lengths

  // Make the last point the same as the first point to close the loop
  initial_positions.back() = initial_positions.front();

  std::cout << "Initial Positions:" << std::endl;
  for (const auto &position : initial_positions) {
    std::cout << position.transpose() << std::endl;
  }

  std::cout << "\nTarget Lengths:" << std::endl;
  for (const auto &length : target_lengths) {
    std::cout << length << std::endl;
  }

  ceres::Problem problem;

  // Add residual blocks for distances between consecutive points
  for (size_t i = 0; i < initial_positions.size() - 1; ++i) {
    ceres::CostFunction *cost_function =
        new ceres::AutoDiffCostFunction<DistanceCostFunction, 1, 2, 2>(
            new DistanceCostFunction(target_lengths[i]));

    problem.AddResidualBlock(cost_function, nullptr,
                             initial_positions[i].data(),
                             initial_positions[i + 1].data());
  }

  // Add a residual block for the loop closure constraint
  ceres::CostFunction *loop_closure_cost_function =
      new ceres::AutoDiffCostFunction<LoopClosureCostFunction, 2, 2, 2>(
          new LoopClosureCostFunction());

  problem.AddResidualBlock(loop_closure_cost_function, nullptr,
                           initial_positions.front().data(),
                           initial_positions.back().data());

  // Configure solver options
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  // Solve the problem
  ceres::Solve(options, &problem, &summary);

  std::cout << "\nOptimized Positions:" << std::endl;
  for (const auto &position : initial_positions) {
    std::cout << position.transpose() << std::endl;
  }

  std::cout << "\nDistances After Optimization:" << std::endl;
  for (size_t i = 0; i < initial_positions.size() - 1; ++i) {
    double distance = (initial_positions[i + 1] - initial_positions[i]).norm();
    std::cout << distance << std::endl;
  }

  std::cout << summary.BriefReport() << std::endl;

  return 0;
}
