#ifndef __MATH_STRUCTURES__
#define __MATH_STRUCTURES__

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <ceres/ceres.h>
#include <functional>

enum class CALCULATION_STATUS
{
   CONVERGENCE = 69, // just for fun, you can delete it later
   DIVERGENCE,
   // this is failure - it can be due to not enough points or lines or w.e.
   FAILURE_SIZE_MISMATCH,
   FAILURE,
};

struct wrappedResult
{
      std::vector<Eigen::Vector2d> vec_{};
      CALCULATION_STATUS stat_;
};

struct DistanceCostFunction

{
      DistanceCostFunction(double target_length) :
          target_length(target_length)
      {
      }

      template <typename T>
      bool operator()(const T *const p1, const T *const p2, T *residual) const
      {
         T delta[2]       = { p2[0] - p1[0], p2[1] - p1[1] };
         T length_squared = delta[0] * delta[0] + delta[1] * delta[1];
         residual[0]      = length_squared - target_length * target_length;

         return true;
      }

      // Define Jacobians
      template <typename T>
      bool operator()(
          const T *const p1, const T *const p2, T *residual, T *jacobian_p1,
          T *jacobian_p2
      ) const
      {
         T delta[2]       = { p2[0] - p1[0], p2[1] - p1[1] };
         T length_squared = delta[0] * delta[0] + delta[1] * delta[1];
         residual[0]      = length_squared - target_length * target_length;

         if(jacobian_p1 != nullptr)
         {
            jacobian_p1[0] = -2 * delta[0];
            jacobian_p1[1] = -2 * delta[1];
         }

         if(jacobian_p2 != nullptr)
         {
            jacobian_p2[0] = 2 * delta[0];
            jacobian_p2[1] = 2 * delta[1];
         }

         return true;
      }

      double target_length;
};

struct LoopClosureCostFunction
{
      template <typename T>
      bool operator()(const T *const p1, const T *const p2, T *residual) const
      {
         residual[0] = p1[0] - p2[0];
         residual[1] = p1[1] - p2[1];
         return true;
      }

      // Define Jacobian
      template <typename T>
      bool operator()(
          const T *const p1, const T *const p2, T *residual, T *jacobian_p1,
          T *jacobian_p2
      ) const
      {
         if(jacobian_p1 != nullptr)
         {
            jacobian_p1[0] = T(1);
            jacobian_p1[1] = T(0);
         }

         if(jacobian_p2 != nullptr)
         {
            jacobian_p2[0] = T(-1);
            jacobian_p2[1] = T(0);
         }

         return true;
      }
};

#endif
