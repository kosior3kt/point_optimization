#ifndef __POINT_OPTIMIZATION__
#define __POINT_OPTIMIZATION__

#include "math_structures.hpp"

#include <iostream>
#include <types.h>

namespace solver
{

   // this sucker shuold be responsible for you know... just calculationg this
   // shiet
   wrappedResult transformPoints(
       std::vector<Eigen::Vector2d>, std::vector<float>, bool _verbose
   );

   namespace helper
   {
      std::string getError(CALCULATION_STATUS);

      CALCULATION_STATUS getCalculationStatus(ceres::TerminationType);
   }
}

#endif
