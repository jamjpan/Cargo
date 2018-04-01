// author @J.T. Hu

#pragma once

#include "../src/Solution.h"
#include "../src/RoadNet.h"

using namespace cargo;

class NearestSolution : public Solution
{
public:
  NearestSolution(RoadNet&);
};
