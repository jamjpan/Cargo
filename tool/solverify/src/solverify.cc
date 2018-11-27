#include <fstream>
#include <iostream>
#include <map>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include "../include/gtree.h"

GTree::G_Tree gtree = GTree::get();

template <typename K, typename V> using dict = std::map<K, V>;
template <typename T> using vec_t = std::vector<T>;

using Trip = struct {
  int id;
  int origin;
  int destination;
  int load;
  int early;
  int late;
  vec_t<std::pair<int, int>> route = {};
};

dict<int, Trip> vehicles = {};
dict<int, Trip> customers = {};
dict<int, int> assignments = {};
dict<int, int> pickup_times = {};
dict<int, int> dropoff_times = {};
dict<int, int> arrival_times = {};
dict<int, int> base_costs = {};
dict<int, int> waits = {};


int print_help();
int solverify(const std::string &, const std::string &);
void print_ranged_route(const vec_t<std::pair<int, int>> &);

/* -------------------------------------------------------------------------- */
int print_help() {
  std::cout
    << "Verify sol/dat files are correct.\n"
    << "Usage: solverify solfile datfile\n"
    << std::endl;
  return 0;
}

int solverify(const std::string& solfile, const std::string& datfile) {
  std::string road, problem;
  int n_vehls, n_custs, base_cost, sol_cost, n_matches, pdel, tdel;

  std::cout << "Loading sol file" << std::endl;
  { std::ifstream f_sol(solfile);
    if (!f_sol.good()) {
      std::cout << "Could not open " << solfile << std::endl;
      return 1;
    }
    std::string _;
    f_sol
      >> problem >> road >> _ >> n_vehls >> _ >> n_custs >> _
      >> _ >> base_cost >> _ >> _ >> sol_cost
      >> _ >> n_matches >> _ >> _ >> _ >> _ >> _ >> _ >> _ >> _ >> _
      >> _ >> _ >> _ >> pdel >> _ >> _ >> _ >> _ >> tdel;
    std::cout
      << "\tFound problem=" << problem << '\n'
      << "\tFound road=" << road << '\n'
      << "\tFound vehicles=" << n_vehls << '\n'
      << "\tFound customers=" << n_custs << '\n'
      << "\tFound base cost=" << base_cost << '\n'
      << "\tFound solution cost=" << sol_cost << '\n'
      << "\tFound matches=" << n_matches << '\n'
      << "\tFound pdel=" << pdel << '\n'
      << "\tFound tdel=" << tdel
      << std::endl;
  }

  std::cout << "Loading problem/" << problem << ".instance" << std::endl;
  { std::ifstream f_prob("problem/"+problem+".instance");
    if (!f_prob.good()) {
      std::cout << "Could not open " << problem << ".instance" << std::endl;
      return 1;
    }
    std::string _;
    for (int i = 0; i < 6; ++i) getline(f_prob, _);
    Trip trip;
    while (f_prob >> trip.id >> trip.origin >> trip.destination >> trip.load
                  >> trip.early >> trip.late) {
      if (trip.load < 0) vehicles[trip.id] = trip;
      else customers[trip.id] = trip;
    }
    if (n_vehls != (int)vehicles.size()) {
      std::cout << "Vehicles size mismatch (" << n_vehls << " != " << (int)vehicles.size() << ")!" << std::endl;
      return 1;
    }
    if (n_custs != (int)customers.size()) {
      std::cout << "Customers size mismatch!" << std::endl;
      return 1;
    }
  }

  std::cout << "Loading data/" << road << ".gtree" << std::endl;
  GTree::load("data/"+road+".gtree");
  gtree = GTree::get();

  std::cout << "Verifying base cost" << std::endl;
  { int sum = 0;
    for (const auto& kv : customers) {
      const Trip& cust = kv.second;
      int cost = gtree.search(cust.origin, cust.destination);
      sum += cost;
      base_costs[cust.id] = cost;
    }
    for (const auto& kv : vehicles) {
      const Trip& vehl = kv.second;
      if (vehl.destination != -1)
        sum += gtree.search(vehl.origin, vehl.destination);
    }
    if (sum != base_cost) {
      std::cout << "Base cost (" << sum << ") mismatch (" << base_cost << ")!" << std::endl;
      return 1;
    }
  }

  std::cout << "Loading dat file" << std::endl;
  { std::ifstream f_dat(datfile);
    if (!f_dat.good()) {
      std::cout << "Could not open " << datfile << std::endl;
      return 1;
    }
    std::string line;
    while (std::getline(f_dat, line)) {
      vec_t<std::string> col;
      { std::string buf;
        std::stringstream ss(line);
        while (getline(ss, buf, ' '))
          col.push_back(buf);
      }
      if (col[1] == "V") {
        for (size_t i = 2; i < col.size() - 1; i+=2) {
          const int& vehl_id = std::stoi(col.at(i));
          const int& node_id = std::stoi(col.at(i+1));
          vehicles.at(vehl_id).route.push_back(std::make_pair(std::stoi(col[0]), node_id));
        }
      } else if (col[1] == "M") {
        for (size_t i = 3; i < col.size(); i++) {
          const int& vehl_id = std::stoi(col.at(2));
          const int& cust_id = std::stoi(col.at(i));
          assignments[cust_id] = vehl_id;
        }
      } else if (col[1] == "P") {
        const int& time = std::stoi(col.at(0));
        for (size_t i = 2; i < col.size(); i+=1) {
          const int& cust_id = std::stoi(col.at(i));
          pickup_times[cust_id] = time;
        }
      } else if (col[1] == "D") {
        const int& time = std::stoi(col.at(0));
        for (size_t i = 2; i < col.size(); i+=1) {
          const int& cust_id = std::stoi(col.at(i));
          dropoff_times[cust_id] = time;
        }
      } else if (col[1] == "A") {
        const int& time = std::stoi(col.at(0));
        for (size_t i = 2; i < col.size(); i+=1) {
          const int& vehl_id = std::stoi(col.at(i));
          arrival_times[vehl_id] = time;
        }
      }
    }
    if ((int)assignments.size() != n_matches) {
      std::cout << "Number of matches (" << assignments.size() << ") mismatch (" << n_matches << ")!" << std::endl;
      return 1;
    }
    if ((int)arrival_times.size() != n_vehls) {
      std::cout << "Vehicles not all arrived!" << std::endl;
      return 1;
    }
  }

  std::cout << "Verifying vehicle late time" << std::endl;
  for (const auto& kv : vehicles) {
    const Trip& vehl = kv.second;
    if (vehl.late != -1 && vehl.route.back().first > vehl.late) {
      std::cout << "Vehicle " << vehl.id << " arrived (" << vehl.route.back().first << ") after late time (" << vehl.late << ")!" << std::endl;
      return 1;
    }
  }

  std::cout << "Verifying customer early and late times" << std::endl;
  { for (const auto& kv : assignments) {
      const int& cust_id = kv.first;
      const int& vehl_id = kv.second;
      if (pickup_times.count(cust_id) == 0) {
        std::cout << "Cust " << cust_id << " matched to vehl " << vehl_id << " but not picked up!" << std::endl;
        return 1;
      }
      const int& pickup_time = pickup_times.at(cust_id);
      if (pickup_time < customers.at(cust_id).early) {
        std::cout << "Cust " << cust_id << " picked up before its early time (" << customers.at(cust_id).early << ")!" << std::endl;
        return 1;
      }
      if (dropoff_times.count(cust_id) == 0){ 
        std::cout << "Cust " << cust_id << " matched to vehl " << vehl_id << " but not dropped off!" << std::endl;
        return 1;
      }
      const int& dropoff_time = dropoff_times.at(cust_id);
      if (dropoff_time > customers.at(cust_id).late) {
        std::cout << "Cust " << cust_id << " dropped off after its late time (" << customers.at(cust_id).late << ")!" << std::endl;
        return 1;
      }
    }
  }

  std::cout << "Verifying pickup and trip delays" << std::endl;
  { int sum = 0;
    for (const auto& kv : pickup_times) {
      const int& cust_id = kv.first;
      const int& pickup_time = kv.second;
      //std::cout << "\tCustomer " << cust_id << " picked up at " << pickup_time << "; early=" << customers.at(cust_id).early << std::endl;
      sum += (pickup_time - customers.at(cust_id).early);
    }
    if (sum/(int)pickup_times.size() != pdel) {
      std::cout << "Avg. pdel mismatch (" << sum/(int)pickup_times.size() << " != " << pdel << ")!" << std::endl;
      return 1;
    }
    sum = 0;
    for (const auto& kv : dropoff_times)
      sum += (kv.second - pickup_times.at(kv.first) - base_costs.at(kv.first)/10);
    if (sum/(int)dropoff_times.size() != tdel) {
      std::cout << "Avg. tdel mismatch (" << sum/(int)dropoff_times.size() << " != " << tdel << ")!" << std::endl;
      return 1;
    }
  }

  std::cout << "No problems. All done!" << std::endl;

  return 0;
}

void print_ranged_route(const vec_t<std::pair<int, int>>& ranged_route) {
  for (const auto& waypoint : ranged_route)
    std::cout << "(" << waypoint.first << "|" << waypoint.second << ") ";
  std::cout << std::endl;
}

int main(int argc, char**argv) {
  vec_t<std::string> args(argv, argv + argc);
  if (argc < 3) return print_help();
  std::string solfile = args.at(1);
  std::string datfile = args.at(2);
  std::cout << "Set solfile to " << solfile << std::endl;
  std::cout << "Set datfile to " << datfile << std::endl;

  return solverify(solfile, datfile);
}

