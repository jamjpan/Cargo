#include "libcargo.h"
#include <algorithm>

using namespace cargo;

class CargoWeb : public RSAlgorithm {
 public:
  CargoWeb();
  size_t k;
  virtual void handle_customer(const Customer &);
};

