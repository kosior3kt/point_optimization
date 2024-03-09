#include "lib.h"

#include "math_structures.hpp"

#include <types.h>

CALCULATION_STATUS
solver::helper::getCalculationStatus(ceres::TerminationType _terminationStatus)
{
   switch(_terminationStatus)
   {
   case ceres::CONVERGENCE:
      return CALCULATION_STATUS::CONVERGENCE;
   case ceres::NO_CONVERGENCE:
      return CALCULATION_STATUS::DIVERGENCE;
   case ceres::FAILURE:
      return CALCULATION_STATUS::FAILURE;
   default:
      return CALCULATION_STATUS::FAILURE;
   }
}

std::string solver::helper::getError(CALCULATION_STATUS _stat)
{
   switch(_stat)
   {
   case CALCULATION_STATUS::CONVERGENCE:
      return "no errors were encountered \n";
   case CALCULATION_STATUS::DIVERGENCE:
      return "the solution does not converge - there may not be solution to "
             "given points and lengths \n";
   case CALCULATION_STATUS::FAILURE:
      return "undefined failure occured ;c \n";
   case CALCULATION_STATUS::FAILURE_SIZE_MISMATCH:
      return "the size of points vec is not size of lengths + 1 (it should "
             "be)\n";
   default:
      return "enum got bigger and so you need to ask developer to implement "
             "this\n";
   }
};

wrappedResult solver::transformPoints(
    std::vector<Eigen::Vector2d> _points, std::vector<float> _lengths,
    bool _verbose
)
{
   if(_points.size() != _lengths.size() + 1)
   {
      // TODO: temp solution, will implement correct idea later. for now it
      // has to take N points and N-1 lengths
      return { _points, CALCULATION_STATUS::FAILURE_SIZE_MISMATCH };
   }

   ceres::Problem problem;
   for(size_t i = 0; i < _points.size() - 1; ++i)
   {
      ceres::CostFunction *cost_function
          = new ceres::AutoDiffCostFunction<DistanceCostFunction, 1, 2, 2>(
              new DistanceCostFunction(_lengths[i])
          );

      problem.AddResidualBlock(
          cost_function, nullptr, _points[i].data(), _points[i + 1].data()
      );
   }

   // TODO: add checking if the problem is closed loop here

   // Add a residual block for the loop closure constraint
   ceres::CostFunction *loop_closure_cost_function
       = new ceres::AutoDiffCostFunction<LoopClosureCostFunction, 2, 2, 2>(
           new LoopClosureCostFunction()
       );

   problem.AddResidualBlock(
       loop_closure_cost_function, nullptr, _points.front().data(),
       _points.back().data()
   );

   // Configure solver options
   ceres::Solver::Options options;
   ceres::Solver::Summary summary;

   // Solve the problem
   ceres::Solve(options, &problem, &summary);

   CALCULATION_STATUS status
       = solver::helper::getCalculationStatus(summary.termination_type);

   if(_verbose)
   {
      std::cout << summary.BriefReport() << std::endl;
   }

   return { _points, status };
}
