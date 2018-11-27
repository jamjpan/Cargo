#include <assert.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <math.h>
#include <sstream>
#include <string>
#include <vector>

#include "../include/gtree.h"
#include "../include/kdtree.h"

const int MAP_TOLERANCE = 100;  // meters
const int MIN_TRIP_LENGTH = 500; // meters
const int EARTH_RADIUS = 6371008;  // meters

// Default parameters
int opt_m = 5000;
short int opt_c = 3;
short int opt_s = 10;
short int opt_t = 1;
short int opt_d = 6;
short int opt_b = 18;
short int opt_r = 30;
short int opt_l = 1;
float opt_x = 1.0;
std::string opt_i = "data/bj5.rnet";
std::string opt_e = "data/bj5.edges";
std::string opt_f = "data/bj5.dat";
std::string opt_g = "data/bj5.gtree";

GTree::G_Tree gtree = GTree::get();
kdtree* kd;

template <typename K, typename V> using dict = std::map<K, V>;
template <typename T> using vec_t = std::vector<T>;

using Trip = struct {
  int id;
  double origin_lng;
  double origin_lat;
  int origin;
  double destination_lng;
  double destination_lat;
  int destination;
  int load;
  int early;
  int late;
};

// Containers
dict<int, std::pair<double, double>> nodes_index = {};
dict<int, dict<int, int>> edges_index = {};
dict<int, vec_t<Trip>> trips_index = {};
vec_t<Trip> customers = {};
vec_t<Trip> vehicles = {};

int print_help();
int rspgen_c(const std::string &);
int point2node(const double &, const double &);
int timestamp2early(const std::string &);
double haversine(const double &, const double &, const double &, const double &);

/* -------------------------------------------------------------------------- */
int print_help() {
  std::cout
    << "Generate problem instances for the RSP.\n"
    << "Usage: rspgen_c [OPTIONS] filename\n"
    << "Parameters:\n"
    << "\n"
    << "\tfilename\toutput the generated instance\n"
    << "Options:\n"
    << "\n"
    << "\t-m\tNumber of vehicles (default=5000)\n"
    << "\t-c\tVehicle capacity (default=3)\n"
    << "\t-s\tVehicle speed (default=10 m/s)\n"
    << "\t-t\tVehicle type (default=1(taxi))\n"
    << "\t-d\tTrip delay (default=6 min)\n"
    << "\t-b\tBegin hour (default=18)\n"
    << "\t-r\tSampling duration (default=30 min)\n"
    << "\t-l\tCustomer load (default=1)\n"
    << "\t-x\tSampling scale (default=1)\n"
    << "\t-i\tInput road network (default=data/bj5.rnet)\n"
    << "\t-f\tInput trips file (default=data/bj5.dat)\n"
    << "\t-g\tInput gtree (default=data/bj5.gtree)\n"
    << std::endl;
  return 0;
}

int rspgen_c(const std::string& out) {
  std::cout << "Loading nodes" << std::endl;
  { std::ifstream f_road(opt_i);
    if (!f_road.good()) {
      std::cout << "Could not open " << opt_i << std::endl;
      return 1;
    }
    int u, v, _;
    double u_lng, u_lat, v_lng, v_lat;
    while (f_road >> _ >> u >> v >> u_lng >> u_lat >> v_lng >> v_lat) {
      nodes_index[u] = std::make_pair(u_lng, u_lat);
      nodes_index[v] = std::make_pair(v_lng, v_lat);
    }
    std::cout << "\tGot " << nodes_index.size() << " nodes" << std::endl;
  }

  std::cout << "Loading edges" << std::endl;
  { std::ifstream f_edge(opt_e);
    if (!f_edge.good()) {
      std::cout << "Could not open " << opt_e << std::endl;
      return 1;
    }
    std::string _;
    int u, v, w;
    std::getline(f_edge, _);
    int count = 0;
    while (f_edge >> u >> v >> w) {
      edges_index[u][v] = w;
      edges_index[v][u] = w;
      edges_index[u][u] = edges_index[v][v] = 0;
      count++;
    }
    std::cout << "\tGot " << count << " edges" << std::endl;
  }

  std::cout << "Building kdtree" << std::endl;
  { kd = kd_create(2);
    for (const auto& kv : nodes_index) {
      const int& node = kv.first;
      const std::pair<double, double>& point = kv.second;
      double ptArr[2] = {point.first, point.second};
      assert(kd_insert(kd, ptArr, (void*)&node) == 0);
    }
    std::cout << "\tDone" << std::endl;
  }

  std::cout << "Loading trips" << std::endl;
  { std::ifstream f_trip(opt_f);
    if (!f_trip.good()) {
      std::cout << "Could not open " << opt_f << std::endl;
      return 1;
    }
    std::string timestamp, _;
    double origin_lng, origin_lat, destination_lng, destination_lat;
    int count = 0;
    for (int i = 0; i < 6; ++i) std::getline(f_trip, _);
    while (f_trip >> _ >> _ >> timestamp >> origin_lng >> origin_lat
                  >> _ >> _ >> destination_lng >> destination_lat) {
      if (haversine(origin_lng, origin_lat, destination_lng, destination_lat) < MIN_TRIP_LENGTH)
        continue;
      Trip trip;
      trip.early = timestamp2early(timestamp)/opt_x;
      std::cout << trip.early << "     \r";
      if (trip.early > opt_r*60) {
        std::cout << "\tDuration reached" << std::endl;
        break;
      }
      trip.id = ++count;
      trip.origin_lng = origin_lng;
      trip.origin_lat = origin_lat;
      trip.origin = -1;
      trip.destination_lng = destination_lng;
      trip.destination_lat = destination_lat;
      trip.destination = -1;
      trip.late = -1;
      if (trips_index.count(trip.early) == 0) trips_index[trip.early] = {};
      trips_index.at(trip.early).push_back(trip);
    }
    std::cout << "\tGot " << count << " trips" << std::endl;
  }

  std::cout << "Loading gtree (" << opt_g << ")" << std::endl;
  GTree::load(opt_g);
  gtree = GTree::get();
  std::cout << "\tDone" << std::endl;

  std::cout << "Parsing vehicles" << std::endl;
  auto i = trips_index.end();
  int x = 0;
  while (x < opt_r*60 && i == trips_index.end()) {
    i = trips_index.find(x);
    x++;
  }
  auto j = i;
  { if (i == trips_index.end()) {
      std::cout << "No trips match begin hour!" << std::endl;
      return 1;
    }
    if (i == trips_index.begin()) {
      std::cout << "No trips available for vehicles!" << std::endl;
      return 1;
    }
    i = std::prev(i, 1);
    int count = 1;
    while (count <= opt_m) {
      for (Trip& trip : i->second) {
        if (haversine(trip.origin_lng, trip.origin_lat, trip.destination_lng,
                      trip.destination_lat) > MIN_TRIP_LENGTH) {
          trip.origin = point2node(trip.origin_lng, trip.origin_lat);
          if (trip.origin == -2) {
              std::cout << "Road has no nodes!" << std::endl;
              return 1;
          }
          if (trip.origin != -1) {
            trip.destination = point2node(trip.destination_lng, trip.destination_lat);
            if (trip.destination != -1) {
              if (opt_t == 1) {
                trip.destination = -1;
                trip.early = 0;
                trip.late = -1;
              } else {
                trip.early = (-1)*trip.early;
                trip.late = trip.early + gtree.search(trip.origin, trip.destination)/opt_s + opt_d*60;
              }
              std::cout << count << "     \r";
              vehicles.push_back(trip);
              count++;
              if (count > opt_m) break;
            }
          }
        }
      }
      i = std::prev(i, 1);
      if (i == trips_index.begin()) {
        std::cout << "No more trips!" << std::endl;
        return 1;
      }
    }
    if (count < opt_m) {
      std::cout << "Not enough trips for vehicles!" << std::endl;
      return 1;
    } else
      std::cout << "\tGot " << vehicles.size() << " vehicles" << std::endl;
  }

  std::cout << "Parsing customers" << std::endl;
  { int count = 1;
    while (j != trips_index.end()) {
      for (Trip& trip : j->second) {
        if (haversine(trip.origin_lng, trip.origin_lat, trip.destination_lng,
                    trip.destination_lat) > MIN_TRIP_LENGTH) {
          trip.origin = point2node(trip.origin_lng, trip.origin_lat);
          if (trip.origin != -1) {
            trip.destination = point2node(trip.destination_lng, trip.destination_lat);
            if (trip.destination != -1) {
              trip.late = trip.early + gtree.search(trip.origin, trip.destination)/opt_s + opt_d*60;
              std::cout << count << "     \r";
              customers.push_back(trip);
              count++;
            }
          }
        }
      }
      j++;
    }
    std::cout << "\tGot " << customers.size() << " customers" << std::endl;
  }

  std::cout << "Writing instance" << std::endl;
  { int count = 1;
    std::ofstream f_out(out+".instance");
    if (!f_out.good()) {
      std::cout << "Could not open " << out << ".instance" << std::endl;
      return 1;
    }
    auto const pos = out.find_last_of('/');
    const auto leaf = out.substr(pos + 1);
    std::string rnet = "bj5";
    if (opt_i.find("cd1") != std::string::npos)
      rnet = "cd1";
    else if (opt_i.find("mny") != std::string::npos)
      rnet = "mny";
    f_out
      << leaf << '\n'
      << rnet << (opt_t == 1 ? " TAXI" : " RS") << '\n'
      << "VEHICLES " << vehicles.size() << '\n'
      << "CUSTOMERS " << customers.size() << '\n' << '\n'
      << "ID\tORIGIN\tDEST\tQ\tEARLY\tLATE\n";
    for (const Trip& vehicle : vehicles) {
      f_out << count << '\t' << vehicle.origin << '\t' << vehicle.destination << '\t'
           << (-1)*opt_c << '\t' << vehicle.early << '\t' << vehicle.late << '\n';
      count++;
    }
    for (const Trip& customer : customers) {
      f_out << count << '\t' << customer.origin << '\t' << customer.destination << '\t'
           << opt_l << '\t' << customer.early << '\t' << customer.late << '\n';
      count++;
    }
    std::cout << "All done!" << std::endl;
  }

  kd_free(kd);
  return 0;
}

int point2node(const double& lng, const double& lat) {
  if (nodes_index.size() == 0) return -2;
  double ptArr[2] = {lng, lat};
  double near_ptArr[2];
  kdres* result = kd_nearest(kd, ptArr);
  if (kd_res_size(result) == 0) {
    std::cout << "no results" << std::endl;
    return -1;
  }
  int node = *((int*)kd_res_item(result, near_ptArr));
  kd_res_free(result);

  return (haversine(lng, lat, nodes_index.at(node).first,
                    nodes_index.at(node).second) > MAP_TOLERANCE ? -1 : node);
}

int timestamp2early(const std::string& timestamp) {
  std::string buf;
  std::stringstream ss(timestamp);
  vec_t<std::string> tokens;
  while (getline(ss, buf, ':'))
    tokens.push_back(buf);
  return (std::stoi(tokens.at(0)) * 3600 + std::stoi(tokens.at(1)) * 60 +
          std::stod(tokens.at(2))) - opt_b*3600;
  return -1;
}

double haversine(const double& lng1, const double& lat1,
                 const double& lng2, const double& lat2) {
  double r = (double)EARTH_RADIUS;
  double x = (lng1 - lng2)*(M_PI/180.0);
  double y = (lat1 - lat2)*(M_PI/180.0);
  double a = std::sin(y/2)*std::sin(y/2)
      + std::sin(x/2)*std::sin(x/2)*std::cos(lat1*(M_PI/180.0))*std::cos(lat2*(M_PI/180.0));
  return r*(2*std::asin(std::sqrt(a)));  // meters
}

int main(int argc, char**argv) {
  vec_t<std::string> args(argv, argv + argc);
  if (argc < 2) return print_help();
  int i = 0;
  while (++i < argc - 1) {
    if (args.at(i) == "-m") {  // Number of vehicles
      opt_m = std::stoi(args.at(i+1));
    } else if (args.at(i) == "-c") {  // Vehicle capacity
      opt_c = std::stoi(args.at(i+1));
    } else if (args.at(i) == "-s") {  // Vehicle speed
      opt_s = std::stoi(args.at(i+1));
    } else if (args.at(i) == "-t") {  // Vehicle type
      opt_t = std::stoi(args.at(i+1));
    } else if (args.at(i) == "-d") {  // Maximum trip delay
      opt_d = std::stoi(args.at(i+1));
    } else if (args.at(i) == "-b") {  // Begin hour
      opt_b = std::stoi(args.at(i+1));
    } else if (args.at(i) == "-r") {  // Sampling duration
      opt_r = std::stoi(args.at(i+1));
    } else if (args.at(i) == "-l") {  // Customer load
      opt_l = std::stoi(args.at(i+1));
    } else if (args.at(i) == "-x") {  // Customer scaling
      opt_x = std::stof(args.at(i+1));
    } else if (args.at(i) == "-i") {  // Input road network file
      opt_i = args.at(i+1);
    } else if (args.at(i) == "-e") {  // Input road network edges
      opt_e = args.at(i+1);
    } else if (args.at(i) == "-f") {  // Input trips file
      opt_f = args.at(i+1);
    } else if (args.at(i) == "-g") {  // Input gtree file
      opt_g = args.at(i+1);
    }
  }
  std::string output = args.at(i);
  std::cout << "Set # vehicles to " << opt_m << std::endl;
  std::cout << "Set capacity to " << opt_c << std::endl;
  std::cout << "Set speed to " << opt_s << std::endl;
  std::cout << "Set type to " << (opt_t == 1 ? "taxi" : "rs" ) << std::endl;
  std::cout << "Set delay to " << opt_d << std::endl;
  std::cout << "Set begin hour to " << opt_b << std::endl;
  std::cout << "Set sampling duration to " << opt_r << std::endl;
  std::cout << "Set customer load to " << opt_l << std::endl;
  std::cout << "Set scaling to " << opt_x << "x" << std::endl;
  std::cout << "Set road network to " << opt_i << std::endl;
  std::cout << "Set edges to " << opt_e << std::endl;
  std::cout << "Set trips file to " << opt_f << std::endl;
  std::cout << "Set gtree to " << opt_g << std::endl;
  std::cout << "Set output to " << output << std::endl;
  return rspgen_c(output);
}

