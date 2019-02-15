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
dict<int, int> capacities = {};
dict<int, int> original_capacities = {};
dict<int, bool> bad_capacities = {};
dict<int, vec_t<std::string>> schedules = {};
dict<int, int> assignments = {};
dict<int, int> pickup_times = {};
dict<int, int> dropoff_times = {};
dict<int, int> arrival_times = {};
dict<int, int> base_costs = {};
dict<int, int> waits = {};


//int solverify(const std::string &, const std::string &, const std::string &, const std::string &);
int solverify(const std::string &, const std::string &, const std::string &);
int route_cost(const vec_t<std::pair<int, int>> &);
int print_help();
void print_ranged_route(const vec_t<std::pair<int, int>> &);
void print_schedule(const vec_t<std::string> &);

/* -------------------------------------------------------------------------- */
int print_help() {
  std::cout
    << "Verify sol/dat files are correct.\n"
    << "Usage: ./solverify *.instance *.gtree *.sol *.dat\n"
    << std::endl;
  return 0;
}

//int solverify(const std::string& problem, const std::string& road,
//              const std::string& solfile, const std::string& datfile) {
int solverify(const std::string& problem, const std::string& road,
              const std::string& datfile) {

  int n_vehls, n_custs, base_cost, sol_cost, n_matches, pdel, tdel;

  /* std::cout << "Loading sol file" << std::endl;
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
  } */

  std::cout << "Loading problem " << problem << std::endl;
  { std::ifstream f_prob(problem);
    if (!f_prob.good()) {
      std::cout << "Could not open " << problem << std::endl;
      return 1;
    }
    std::string _;
    for (int i = 0; i < 6; ++i) getline(f_prob, _);
    Trip trip;
    while (f_prob >> trip.id >> trip.origin >> trip.destination >> trip.load
                  >> trip.early >> trip.late) {
      if (trip.load < 0) {
        vehicles[trip.id] = trip;
        capacities[trip.id] = (int)trip.load;
        original_capacities[trip.id] = (int)trip.load;
        schedules[trip.id] = {};
      } else {
        customers[trip.id] = trip;
        assignments[trip.id] = -1;
      }
    }
    // if (n_vehls != (int)vehicles.size()) {
    //   std::cout << "Vehicles size mismatch (" << n_vehls << " != " << (int)vehicles.size() << ")!" << std::endl;
    //   return 1;
    // }
    // if (n_custs != (int)customers.size()) {
    //   std::cout << "Customers size mismatch!" << std::endl;
    //   return 1;
    // }
    std::cout << "  Got " << vehicles.size() << " vehicles" << std::endl;
    std::cout << "  Got " << customers.size() << " customers" << std::endl;
  }

  std::cout << "Loading " << road << std::endl;
  GTree::load(road);
  gtree = GTree::get();

  // std::cout << "Verifying base cost" << std::endl;
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
    // if (sum != base_cost) {
    //   std::cout << "Base cost (" << sum << ") mismatch (" << base_cost << ")!" << std::endl;
    //   return 1;
    // }
    std::cout << "  Got base cost " << sum << " meters" << std::endl;
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
        const int& vehl_id = std::stoi(col.at(2));
        const int& node_id = std::stoi(col.at(3));
        vehicles.at(vehl_id).route.push_back(std::make_pair(std::stoi(col[0]), node_id));
      } else if (col[1] == "M") {
        for (size_t i = 3; i < col.size(); i++) {
          const int& vehl_id = std::stoi(col.at(2));
          const int& cust_id = std::stoi(col.at(i));
          if (cust_id < 0) {
            assignments[-cust_id] = -1;
          } else {
            assignments[cust_id] = vehl_id;
          }
        }
      } else if (col[1] == "P") {
        const int& time = std::stoi(col.at(0));
        for (size_t i = 2; i < col.size(); i++) {
          const int& cust_id = std::stoi(col.at(i));
          schedules[assignments.at(cust_id)].push_back("(p" + std::to_string(cust_id) + ")");
          if (pickup_times.count(cust_id) != 0) {
            std::cout << "ERROR: Cust " << cust_id << " picked up twice (t="
                      << pickup_times.at(cust_id) << ", t=" << time << ")!" << std::endl;
            // return 1;
          }
          if (assignments.count(cust_id) == 0) {
            std::cout << "ERROR: Cust " << cust_id << " picked up (t=" << time << ") without being assigned!" << std::endl;
          }
          capacities.at(assignments.at(cust_id))++;
          if (capacities.at(assignments.at(cust_id)) > 0) {
            std::cout << "ERROR: Vehl " << assignments.at(cust_id) << " exceeds capacity!" << std::endl;
            // print_schedule(schedules.at(assignments.at(cust_id)));
            bad_capacities[assignments.at(cust_id)] = true;
            capacities.at(assignments.at(cust_id))--;
          }
          pickup_times[cust_id] = time;
        }
      } else if (col[1] == "D") {
        const int& time = std::stoi(col.at(0));
        for (size_t i = 2; i < col.size(); i+=1) {
          const int& cust_id = std::stoi(col.at(i));
          schedules[assignments.at(cust_id)].push_back("(d" + std::to_string(cust_id) + ")");
          if (dropoff_times.count(cust_id) != 0) {
            std::cout << "ERROR: Cust " << cust_id << " dropped off (t="
                      << dropoff_times.at(cust_id) << ", t=" << time << ") twice!" << std::endl;
            // return 1;
          }
          if (assignments.count(cust_id) == 0) {
            std::cout << "ERROR: Cust " << cust_id << " dropped off (t=" << time << ") without being assigned!" << std::endl;
          }
          capacities.at(assignments.at(cust_id))--;
          if (capacities.at(assignments.at(cust_id)) < original_capacities.at(assignments.at(cust_id))) {
            std::cout << "ERROR: Vehl " << assignments.at(cust_id) << " excess capacity!" << std::endl;
            // print_schedule(schedules.at(assignments.at(cust_id)));
            bad_capacities[assignments.at(cust_id)] = true;
            capacities.at(assignments.at(cust_id))++;
          }
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
    // if ((int)assignments.size() != n_matches) {
    //   std::cout << "Number of matches (" << assignments.size() << ") mismatch (" << n_matches << ")!" << std::endl;
    //   return 1;
    // }
    { int number_matches = 0;
      for (const auto& kv : assignments)
        if (kv.second != -1)
          number_matches ++;
      std::cout << "Assignments: " << number_matches << std::endl;
    }
    // for (const auto& kv : assignments) {
    //   const int& cust_id = kv.first;
    //   std::cout << "Cust " << cust_id << ": " << pickup_times.at(cust_id) << ", " << dropoff_times.at(cust_id) << std::endl;
    // }
    // if ((int)arrival_times.size() != n_vehls) {
    //   std::cout << "Vehicles not all arrived!" << std::endl;
    //   return 1;
    // }
    if (arrival_times.size() != vehicles.size()) {
      std::cout << "ERROR: Vehicles not all arrived!" << std::endl;
      // return 1;
    }
  }

  if (bad_capacities.size() != 0) {
    std::cout << "ERROR: Bad vehicle capacities:" << std::endl;
    for (const auto& kv : bad_capacities) {
      std::cout << "  Vehl " << kv.first << ": ";
      print_schedule(schedules.at(kv.first));
    }
  }

  std::cout << "Verifying vehicle late time" << std::endl;
  for (const auto& kv : vehicles) {
    const Trip& vehl = kv.second;
    if (vehl.late != -1 && vehl.route.back().first > vehl.late) {
      std::cout << "ERROR: Vehicle " << vehl.id << " arrived (" << vehl.route.back().first << ") after late time (" << vehl.late << ")!" << std::endl;
      // return 1;
    }
  }

  std::cout << "Verifying customer early and late times" << std::endl;
  { for (const auto& kv : assignments) {
      const int& cust_id = kv.first;
      const int& vehl_id = kv.second;
      if (vehl_id != -1) {
        if (pickup_times.count(cust_id) == 0) {
          std::cout << "ERROR: Cust " << cust_id << " matched to vehl " << vehl_id << " but not picked up!" << std::endl;
          // return 1;
        } else {
          const int& pickup_time = pickup_times.at(cust_id);
          if (pickup_time < customers.at(cust_id).early) {
            std::cout << "ERROR: Cust " << cust_id << " picked up (t=" << pickup_time << ") before early time (t=" << customers.at(cust_id).early << ")!" << std::endl;
            // return 1;
          }
        }
        if (dropoff_times.count(cust_id) == 0){
          std::cout << "ERROR: Cust " << cust_id << " matched to vehl " << vehl_id << " but not dropped off!" << std::endl;
          // return 1;
        } else {
          const int& dropoff_time = dropoff_times.at(cust_id);
          if (dropoff_time > customers.at(cust_id).late) {
            std::cout << "ERROR: Cust " << cust_id << " dropped off (t=" << dropoff_time << ") after late time (t=" << customers.at(cust_id).late << ")!" << std::endl;
            // return 1;
          }
        }
      }
    }
  }

  // std::cout << "Verifying pickup and trip delays" << std::endl;
  // { int sum = 0;
  //   for (const auto& kv : pickup_times) {
  //     const int& cust_id = kv.first;
  //     const int& pickup_time = kv.second;
  //     //std::cout << "\tCustomer " << cust_id << " picked up at " << pickup_time << "; early=" << customers.at(cust_id).early << std::endl;
  //     sum += (pickup_time - customers.at(cust_id).early);
  //   }
  //   if (sum/(int)pickup_times.size() != pdel) {
  //     std::cout << "Avg. pdel mismatch (" << sum/(int)pickup_times.size() << " != " << pdel << ")!" << std::endl;
  //     return 1;
  //   }
  //   sum = 0;
  //   for (const auto& kv : dropoff_times)
  //     sum += (kv.second - pickup_times.at(kv.first) - base_costs.at(kv.first)/10);
  //   if (sum/(int)dropoff_times.size() != tdel) {
  //     std::cout << "Avg. tdel mismatch (" << sum/(int)dropoff_times.size() << " != " << tdel << ")!" << std::endl;
  //     return 1;
  //   }
  // }

  { std::cout << "Computing solution cost" << std::endl;
    int penalty = 0;
    int unmatched = 0;
    int traveled = 0;
    for (const auto& kv : assignments) {
      if (kv.second == -1) {
        unmatched++;
        penalty += base_costs.at(kv.first);
      }
    }
    std::cout << "  Got penalty " << penalty << " for " << unmatched << " unmatched custs" << std::endl;
    for (const auto& kv : vehicles) {
      traveled += route_cost(kv.second.route);
    }
    std::cout << "  Got " << traveled << " traveled meters" << std::endl;
    std::cout << "  Got total sol cost " << traveled + penalty << " meters" << std::endl;
  }

  std::cout << "All done!" << std::endl;

  return 0;
}

void print_ranged_route(const vec_t<std::pair<int, int>>& ranged_route) {
  for (const auto& waypoint : ranged_route)
    std::cout << "(" << waypoint.first << "|" << waypoint.second << ") ";
  std::cout << std::endl;
}

void print_schedule(const vec_t<std::string>& schedule) {
  for (const auto& stop : schedule)
    std::cout << stop << " ";
  std::cout << std::endl;
}

int route_cost(const vec_t<std::pair<int, int>>& route) {
  int cost = 0;
  for (size_t i = 1; i < route.size(); i++) {
    cost += gtree.search(route.at(i-1).second, route.at(i).second);
  }
  return cost;
}

int main(int argc, char**argv) {
  vec_t<std::string> args(argv, argv + argc);
  if (argc < 3) return print_help();
  std::string instance = args.at(1);
  std::string road = args.at(2);
  //std::string solfile = args.at(3);
  std::string datfile = args.at(3);
  std::cout << "Set instance to " << instance << std::endl;
  std::cout << "Set gtree to " << road << std::endl;
  //std::cout << "Set solfile to " << solfile << std::endl;
  std::cout << "Set datfile to " << datfile << std::endl;

  //return solverify(instance, road, solfile, datfile);
  return solverify(instance, road, datfile);
}

