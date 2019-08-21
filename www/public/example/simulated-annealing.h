#include <memory>
#include <set>
#include <unordered_map>
#include <utility>
#include <random>
#include <algorithm> /* std::shuffle */
#include <chrono>
#include <cmath> /* std::exp */
#include <iostream> /* std::endl */
#include <random>
#include <unordered_map>
#include <vector>


#include "libcargo.h"

using namespace cargo;

typedef dict<VehlId, std::pair<MutableVehicle, vec_t<Customer>>> SASol;

const int f = 50;
const int tmax = 5;
const int pmax = 5000;
const int BATCH = 30;
const int SCHED_MAX = 10;

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
  int p_max;
  int t_max;
  float f_;

  int nclimbs_;
  int ntries_;
  int nanneals_;
  int ntimeouts_;
  int ncap_;
  int ntw_;
  std::mt19937 gen;
  std::uniform_real_distribution<> d;
  std::uniform_int_distribution<> n;

  /* Workspace variables */
  SASol sol;
  //dict<CustId, std::set<VehlId>> tried;
  dict<CustId, vec_t<MutableVehicleSptr>> candidates_list;
  dict<VehlId, MutableVehicleSptr> vehicle_lookup;
  tick_t timeout_0;
  std::vector<Stop> sch, sch_after_rem, sch_after_add;
  std::vector<Wayp> rte, rte_after_rem, rte_after_add;
  std::vector<std::tuple<Customer, MutableVehicle, DistInt>> best_sol;
  std::unordered_map<VehlId, std::vector<Customer>> commit_cadd;
  std::unordered_map<VehlId, std::vector<Wayp>> commit_rte;
  std::unordered_map<VehlId, std::vector<Stop>> commit_sch;

  DistInt sol_cost(const SASol& sol) {
      std::set<MutableVehicle> vehls = {};
      for (const auto& kv : sol) vehls.insert(kv.second.first);
      DistInt sum = 0;
      for (const MutableVehicle& vehl : vehls) sum += vehl.route().cost();
      return sum;
  }

  void initialize(Grid &);
  SASol perturb(const SASol &, const int &);
  void commit();

  bool hillclimb(const int& T) {
    float mark = this->d(this->gen);
    float thresh = std::exp(this->f_*(float)T)/100;
    return mark < thresh;
  }

  void anneal(const int& T_MAX, const int& P_MAX) {
    for (int t = T_MAX; t > 0; t--) {
      for (int p = P_MAX; p > 0; p--) {
        this->nanneals_++;
        this->sol = std::move(this->perturb(this->sol, t));
        if (this->timeout(this->timeout_0)) {
          print << "timed out" << std::endl;
          this->ntimeouts_++;
          return;
        }
      }
      if (this->timeout(this->timeout_0)) {
        print << "timed out" << std::endl;
        this->ntimeouts_++;
        return;
      }

      // Visualize initial assignments
      gui::reset();
      for (const auto& kv1 : this->sol) {
        for (const Customer& cust : kv1.second.second) {
          gui::chi(cust.orig());
          gui::clinev(cust.id(), kv1.first);
          // gui::newroute(kv1.second.first.route());
        }
      }
      pause();
    }
  }

  void reset_workspace();

  void print_sol(const SASol &);
};

