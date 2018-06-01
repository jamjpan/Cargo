// author @J.T. Hu

#include "libcargo/Solution.h"

namespace cargo
{
Solution::Solution(Simulator *sim)
{
    sim_ = sim;
    speed_ = sim->Speed();
}
}