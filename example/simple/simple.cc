#include <iostream>
#include <string>
#include <vector>
#include "libcargo.h"
class MyAlgorithm : public cargo::RSAlgorithm {
 public:
  MyAlgorithm();
  virtual void handle_customer(const cargo::Customer &);
  virtual void end();
 private:
  int nmatches;
};

MyAlgorithm::MyAlgorithm() : cargo::RSAlgorithm("myalg") {
  this->nmatches = 0;
}

void MyAlgorithm::handle_customer(const cargo::Customer &cust)
{
s00:  if (cust.assigned()) return;
s01:  for (const cargo::Vehicle &vehl : this->vehicles())
      {
s02:    cargo::MutableVehicle matched_vehicle(vehl);
s03:    std::vector<cargo::Stop> augmented_schedule;
s04:    std::vector<cargo::Wayp> augmented_route;
s05:    cargo::sop_insert(vehl, cust, augmented_schedule, augmented_route);
s06:    if (chktw(augmented_schedule, augmented_route) == true)
        {
s07:      std::vector<cargo::CustId> to_assign {cust.id()};
s08:      std::vector<cargo::CustId> to_remove {};
s09:      if (this->assign(to_assign, to_remove,
                augmented_route, augmented_schedule, matched_vehicle) == true)
          {
s10:        this->print(cargo::MessageType::Success) << "Matched!" << std::endl;
s11:        this->nmatches++;
s12:        break;
          }
        }
      }
}

void MyAlgorithm::end() {
  this->print(cargo::MessageType::Info)
    << "Matches: " << this->nmatches << std::endl;
}

int main()
{
    cargo::Options opt;
    opt.path_to_roadnet = "../../data/roadnetwork/bj5.rnet";
    opt.path_to_gtree   = "../../data/roadnetwork/bj5.gtree";
    opt.path_to_edges   = "../../data/roadnetwork/bj5.edges";
    opt.path_to_problem = "../../data/benchmark/rs-sm-1.instance";
    opt.time_multiplier = 1;
    opt.vehicle_speed   = 10;
    opt.matching_period = 60;

    cargo::Cargo cargo(opt);  // must be initialized before algorithm
    MyAlgorithm alg;
    cargo.start(alg);
}

