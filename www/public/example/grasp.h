#include <algorithm>
#include <iostream>
#include <random>
#include <unordered_set>
#include <utility>
#include <vector>
#include <memory>
#include <tuple>
#include <unordered_map>
#include <random>

#include "libcargo.h"

using namespace cargo;

const int BATCH = 30;
const int TOP_K = 10;
const int SCHED_MAX = 10;

typedef dict<MutableVehicleSptr, std::pair<vec_t<CustId>, vec_t<CustId>>>
Solution;

class CargoWeb : public RSAlgorithm {
 public:
  CargoWeb();

  /* My overrides */
  virtual void handle_vehicle(const Vehicle &);
  virtual void match();
  virtual void end();
  virtual void listen(bool skip_assigned = true, bool skip_delayed = true);

 private:
  Grid grid_;

  int max_iter;

  std::mt19937 gen;
  std::uniform_real_distribution<> d;

  int nswap_;
  int nreplace_;
  int nrearrange_;
  int nnoimprov_;

  tick_t timeout_0;

  Solution best_solution;
  DistInt  best_cost;

  dict<Customer, std::unordered_set<MutableVehicleSptr>> candidates_index;
  dict<MutableVehicleSptr, vec_t<Customer>> candidates_list;
  dict<MutableVehicleSptr, dict<Customer, DistInt>> fitness;
  dict<MutableVehicleSptr, dict<Customer, vec_t<Stop>>> schedules;
  dict<MutableVehicleSptr, dict<Customer, vec_t<Wayp>>> routes;
  vec_t<Stop> sch;
  vec_t<Wayp> rte;

  Solution initialize(Grid &, vec_t<Customer> &);
  Solution replace(const Solution &, vec_t<Customer> &);
  Solution swap(const Solution &);
  Solution rearrange(const Solution &);
  Customer roulette(const dict<Customer, DistInt> &);
  DistInt cost(const Solution &);

  void construct(const int &);

  void print_sol(const Solution &);

  void commit(const Solution &);

  void reset_workspace() {
    this->candidates_index = {};
    this->candidates_list = {};
    this->fitness = {};
    this->schedules = {};
    this->routes = {};
    this->sch = {};
    this->rte = {};
  }

};

