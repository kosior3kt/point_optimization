#include "../lib/src/lib/lib.h"
#include "../lib/src/lib/math_structures.hpp"

int main()
{
   size_t N = 5;
   std::vector<Eigen::Vector2d> initial_positions(N);

   // Initialize points in a circular arrangement
   for(size_t i = 0; i < N; ++i)
   {
      double theta
          = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(N);
      initial_positions[i] << std::cos(theta), std::sin(theta);
   }

   std::vector<float> target_lengths((int)(N - 1));
   target_lengths = { 2.0, 2.5, 1.0, 3.0 }; // Replace with your target lengths

   // Make the last point the same as the first point to close the loop
   initial_positions.back() = initial_positions.front();

   std::cout << "Initial Positions:" << std::endl;
   for(const auto &position : initial_positions)
   {
      std::cout << position.transpose() << std::endl;
   }

   std::cout << "\nTarget Lengths:" << std::endl;
   for(const auto &length : target_lengths)
   {
      std::cout << length << std::endl;
   }

   wrappedResult res
       = solver::transformPoints(initial_positions, target_lengths, true);

   std::cout << solver::helper::getError(res.stat_) << std::endl;

   if(res.stat_ == CALCULATION_STATUS::CONVERGENCE)
   {
      std::cout << "this shiet succeeded!!!!\n\n\n\n\n";

      std::cout << "\nOptimized Positions:" << std::endl;
      for(const auto &position : res.vec_)
      {
         std::cout << position.transpose() << std::endl;
      }

      std::cout << "\nDistances After Optimization:" << std::endl;
      for(size_t i = 0; i < res.vec_.size() - 1; ++i)
      {
         double distance = (res.vec_[i + 1] - res.vec_[i]).norm();
         std::cout << distance << std::endl;
      }
   }

   return 0;
}
