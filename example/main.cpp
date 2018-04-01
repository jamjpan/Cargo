#include "../src/Cargo.h"
#include "../src/Solution.h"
#include "NearestSolution.h"

using namespace cargo;

int main(int argc, const char* argv[])
{
  Cargo c(argc, argv);
  Solution* s = new NearestSolution(c.getRoadNet());
  c.run(s);
  return 0;
}
