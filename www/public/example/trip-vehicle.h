#include <unordered_map>
#include <vector>
#include <omp.h>
#include <algorithm> /* std::find, std::nth_element */
#include <chrono>
#include <ctime>
#include <exception>
#include <iostream> /* std::endl */
#include <vector>

#include "glpk/glpk.h"

#include "libcargo.h"

using namespace cargo;

const int BATCH = 30;
const int TOP_CUST = 30;  // customers per vehicle for rv-graph
//const int TRIP_MAX = 30000;  // maximum number of trips per batch
// const int TOP_CUST = 8;  // customers per vehicle for rv-graph
const int TRIP_MAX = 12000;  // maximum number of trips per batch


typedef int SharedTripId;
typedef std::vector<Customer> SharedTrip;

class CargoWeb : public RSAlgorithm {
 public:
  CargoWeb();

  int unassign_penalty = -1;

  /* My overrides */
  virtual void handle_vehicle(const Vehicle &);
  virtual void match();
  virtual void listen(bool skip_assigned = true, bool skip_delayed = true);

 private:
  Grid grid_;

  /* Vector of GTrees for parallel sp */
  std::vector<GTree::G_Tree> gtre_;


  /* Workspace variables */
  std::unordered_map<CustId, bool> is_matched;
  tick_t timeout_rv_0;
  tick_t timeout_rtv_0;


  SharedTripId stid_;
  dict<CustId, std::vector<SharedTripId>>   cted_;  // cust-trip edges
  dict<SharedTripId, SharedTrip>            trip_;  // trip lookup

  bool travel(const Vehicle &,                // Can this vehl...
              const std::vector<Customer> &,  // ...serve these custs?
              DistInt &,                      // cost of serving
              std::vector<Stop> &,            // resultant schedule
              std::vector<Wayp> &,            // resultant route
              GTree::G_Tree &);               // gtree to use for sp

  SharedTripId add_trip(const SharedTrip &);

  void reset_workspace();
};

