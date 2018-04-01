// author @J.T. Hu

#include <stdexcept>
#include <climits>
#include "NearestSolution.h"
#include "../src/Schdule.h"
#include "../src/common.h"

int NearestSolution::assign(const Customer& customer) {
  // 1 search for kth nearest vehicle (k initially set to 1)
  // 2 given the current schedule of the vehicle
  // 3 try to insert the request into schedule, test if time window constraints are satisfied
  // 4 if valid schedules are found, choose the once with minimum distance
  // 5 if not, k++, go to 1

  // 1
  int k = 0;
  bool found = false;
  Vehicle& vehicle;
  Schedule& schedule;
  std::vector<stop_t> stops, testStops;
  stop_t current;

  // can be more efficient by caching knn search
  while (!found) {
    ++k;
    try {
      vehicle = mRoadNet.getKthVehicle(customer.Origin(), k);
      // if capacity is not enough
      if (vehicle.Capacity() < customer.Demand())
        continue;

      schedule = vehicle.Schedule();
      stops = schedule.Stops();
      current = vehicle.Current();

      // pick out valid stops to insert
      auto start = std::next(std::find(stops.begin(), stops.end(), current), 1);
      auto end = std::prev(stops.end(), 1);
      testStops.clear();
      testStops.push_back(stop_t {customer.Origin(), customer.Id(), PICKUP}); // only in c++11
      testStops.push_back(stop_t {customer.Destination(), customer.Id(), DROPOFF});
      for (auto itr = start; itr != end; ++itr)
        testStops.push_back(*itr); // copy into vector

      if (!isValid(testStops))
        continue;

      int matchId = -1;
      double minDistance = std::numeric_limits<double>::max();
      std::vector<stop_t>& minStops;
      do {
        std::vector<stop_t> ts = testStops;
        ts.insert(ts.begin(), stop_t {current, -1, PICKUP});
        ts.push_back(stop_t {vehicle.Destination(), -1, DROPOFF});

        double distance = mRoadNet.shortestDistance(ts);
        if (distance < minDistance) {
          matchId = vehicle.Id();
          minDistance = distance;
          minStops = ts;
        }

      } while (std::next_permutation(testStops.begin(), testStops.end()));

      if (matchId != -1) {
        found = true;
      }
    } catch (const std::exception& e) {
      return -1;
    }
  }
}
