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
  vec_t<int> route = {};
};

dict<int, Trip> vehicles = {};
dict<int, Trip> customers = {};
dict<int, int> assignments = {};
dict<int, int> pickup_times = {};
dict<int, int> dropoff_times = {};
dict<int, int> arrival_times = {};
dict<int, vec_t<std::pair<int, int>>> ranged_routes = {};
dict<int, int> base_costs = {};

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
      if (trip.load < -1) vehicles[trip.id] = trip;
      else customers[trip.id] = trip;
    }
    if (n_vehls != (int)vehicles.size()) {
      std::cout << "Vehicles size mismatch!" << std::endl;
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
          vehicles.at(vehl_id).route.push_back(node_id);
        }
      } else if (col[1] == "M") {
        for (size_t i = 2; i < col.size() - 1; i+=2) {
          const int& vehl_id = std::stoi(col.at(i));
          const int& cust_id = std::stoi(col.at(i+1));
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
    if ((int)pickup_times.size()+(int)dropoff_times.size() != n_matches*2) {
      std::cout << "Not all matches picked up or dropped off!" << std::endl;
      return 1;
    }
  }

  std::cout << "Verifying route costs and vehicle time windows" << std::endl;
  { int sum = 0;
    for (const auto& kv : vehicles) {
      const Trip& vehl = kv.second;
      const vec_t<int>& route = vehl.route;
      vec_t<std::pair<int, int>> ranged_route = {};
      ranged_route.push_back(std::make_pair(0, vehl.origin));
      int local_sum = gtree.search(vehl.origin, route.front());
      ranged_route.push_back(std::make_pair(local_sum, route.front()));
      for (size_t i = 0; i < route.size()-1; ++i) {
        local_sum += gtree.search(route.at(i), route.at(i+1));
        ranged_route.push_back(std::make_pair(local_sum, route.at(i+1)));
      }
      // TODO When problems and sol file include variable vehicle speeds, edit this part below
      int arrival = std::round(local_sum/10.0);  // make it precise whenever Cargo supports it
      if (vehl.late != -1 && arrival > vehl.late) {
        std::cout << "Vehicle " << vehl.id << " fails time window (" << arrival << " > " << vehl.late << ")" << std::endl;
        print_ranged_route(ranged_route);
        return 1;
      }
      if (arrival < arrival_times.at(vehl.id)
       || arrival > arrival_times.at(vehl.id)+1) {
        std::cout << "Vehicle " << vehl.id << " arrival time (" << arrival_times.at(vehl.id) << ") mistmatches route (" << arrival << ")!" << std::endl;
        print_ranged_route(ranged_route);
        return 1;
      }
      sum += local_sum;
      ranged_routes[vehl.id] = ranged_route;
    }
    if (sum != sol_cost) {
      std::cout << "Route costs (" << sum << ") mismatch solution cost (" << sol_cost << ")!" << std::endl;
      return 1;
    }
  }

  std::cout << "Verifying assignments and times" << std::endl;
  { for (const auto& kv : assignments) {
      const int& vehl_id = kv.second;
      const Trip& cust = customers.at(kv.first);
      const vec_t<std::pair<int, int>>& ranged_route = ranged_routes.at(vehl_id);
      auto i = std::find_if(ranged_route.begin(), ranged_route.end(), [&](const std::pair<int, int>& waypoint) {
              int time_to_pickup = std::round(waypoint.first/10.0);
              return time_to_pickup <= pickup_times.at(cust.id)+1 && time_to_pickup >= pickup_times.at(cust.id)-1; });
      if (i == ranged_route.end()) {
        std::cout << "Customer " << cust.id << " (" << cust.origin << ", " << cust.destination << ") matched but never picked up (vehl " << vehl_id << ")!" << std::endl;
        print_ranged_route(ranged_route);
        return 1;
      }
      int time_to_o = (int)(i->first/10.0);  // 10 is the vehicle speed
      if (time_to_o < cust.early) {
        std::cout << "Customer " << cust.id <<  " (" << cust.origin << ", " << cust.destination << ") picked up before early (" << time_to_o << " < " << cust.early << ")" << std::endl;
        print_ranged_route(ranged_route);
        return 1;
      }
      auto j = std::find_if(i, ranged_route.end(), [&](const std::pair<int, int>& waypoint) {
              int time_to_dropoff= std::round(waypoint.first/10.0);
              return time_to_dropoff <= dropoff_times.at(cust.id)+1 && time_to_dropoff >= dropoff_times.at(cust.id)-1; });
      if (j == ranged_route.end()) {
        std::cout << "Customer " << cust.id << " (" << cust.origin << ", " << cust.destination << ") matched but never dropped off (vehl " << vehl_id << ")!" << std::endl;
        print_ranged_route(ranged_route);
        return 1;
      }
      int time_to_d = (int)(j->first/10.0);
      if (time_to_d > cust.late + 1) {
        std::cout << "Customer " << cust.id << " (" << cust.origin << ", " << cust.destination << ") dropped off after late (" << time_to_d << " < " << cust.late << ")" << std::endl;
        print_ranged_route(ranged_route);
        return 1;
      }
    }
  }

  std::cout << "Verifying pickup and trip delays" << std::endl;
  { int sum_pdel = 0;
    for (const auto& kv : pickup_times)
      sum_pdel += (kv.second - customers.at(kv.first).early);
    int our_pdel = sum_pdel/n_custs;
    if (our_pdel != pdel) {
      std::cout << "Our pdel (" << our_pdel << ") != pdel (" << pdel << ")!" << std::endl;
      return 1;
    }
    int sum_tdel = 0;
    for (const auto& kv : dropoff_times) {
      const int& cust_id = kv.first;
      int delay = (kv.second - pickup_times.at(cust_id)) - (customers.at(cust_id).early + base_costs.at(cust_id)/10.0);
      sum_tdel += delay;
    }
    int our_tdel = sum_tdel/n_custs;
    if (our_tdel != tdel) {
      std::cout << "Our tdel (" << our_tdel << ") != tdel (" << tdel << ")!" << std::endl;
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

