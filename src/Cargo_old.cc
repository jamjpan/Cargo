// author @J.T. Hu

#include <iostream>
#include <thread>
// https://github.com/sailormoon/flags
// #include "flags.h"
#include "Cargo.h"
#include "Loader.h"
#include "readerwriterqueue.h"

namespace cargo {

  Cargo::Cargo(int argc, const char* argv[])
    : road_net_(RoadNet(road_net_path_, gtree_path_)) {
    // const flags::args args(argc, argv);

    Loader loader(road_net_, trips_path_);
    // load trips dataset
    loader.loadTrips(trips_);
  }

  void Cargo::print_usage() {
    std::cerr
      << "Usage: ./routeRS <start> <dur> [OPTS]\n\n"
      << "\t<start>      Timestamp to begin generating requests\n"
      << "\t                 Format: hh-mm-ss, e.g., \"18-00-00\"\n"
      << "\t<dur>        Simulation duration (min)\n\n"
      << "The options are:\n\n"
      << "\t-d           Datasource?\n"
      << "\t                 0 : Beijing (default)\n"
      << "\t                 1 : New York City\n"
      << "\t                 2 : Chengdu\n"
      << "\t-w [0, 1]    Save results to disk? (default: 0)\n"
      << "\t-e           Experiment?\n"
      << "\t                 0 : Approx (default)\n"
      << "\t                 1 : Exact\n"
      << "\t                 2 : FirstValid (bad)\n"
      << "\t                 3 : NearestValid (bad)\n"
      << "\t                 4 : NaiveExact\n"
      << "\t                 5 : TSHARE\n"
      << "\t                 6 : NOAH\n"
      << "\t-y [0, 1]    Export for plotting? (default: 0)\n"
      << "\t-s <int>     Scale factor (default: 1)\n"
      << "\t-m <int>     # of vehicles (default: 4000)\n"
      << "\t-c <int>     Vehicle capacity (default: 2)\n"
      << "\t-v <int>     Vehicle speed, km/hr (default: 32)\n"
      << "\t-b <int>     Max detour budget, m (default: 5200)\n"
      << "\t-p <int>     Request distance constraint, m (default: 2700)\n"
      << "\t-r [0, 1]    Replay unmatched requests (default: 1)\n"
      << "\t-R <int>     Static mode; Specify number of requests\n"
      << "\t                 (ignores dur, -s, -r)\n"
      << std::endl;
  }

  void Cargo::run(Solution* solution) {
    // initialize vehicles. could seperate vehicles and requests
    int count = 0;
    auto itr = trips.find(start);
    while (count != number_of_vehicles && itr != trips_.begin()) {
      --itr;
    }

    // The RequestQueue for storing requests Thread-safe thanks to cameron314
    // https://github.com/cameron314/readerwriterqueue
    moodycamel::ReaderWriterQueue<Customer> requestQueue;
    // ... and another queue for replayed requests
    moodycamel::ReaderWriterQueue<Customer> replayQueue;

    time_t globalTime = start_;
    bool done = false;

    // thread for processing requests
    std::thread RequestProcessor([&] {
      Customer customer(-1, -1, -1, -1, -1, -1);
      while (!done) {
        bool hasRequest = requestQueue.try_dequeue(customer);
        if (!hasRequest)
          continue; // or sleep for a while
      }
    });
  }
}
