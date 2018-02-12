// Copyright(c) 2018 James J. Pan
// Distributed under the MIT License (http://opensource.org/licenses/MIT)
//
#include "libezrs.h"    // Includes GTree
#include <cmath>

using namespace std;

ezrs::DistanceHelper::DistanceHelper() {
    mHasGtree = false;
}

ezrs::DistanceHelper::DistanceHelper(GTree::G_Tree &gt) : mGtree(gt) {
    mHasGtree = true;
}

double ezrs::DistanceHelper::Euclidean(const Node_t &u, const Node_t &v) {
  return hypot(u.Latitude - v.Latitude, u.Longitude - v.Longitude);
}

double ezrs::DistanceHelper::Haversine(const Node_t &u, const Node_t &v) {
  double r = 6372800.0; // radius of Earth (m)
  double dx = (u.Longitude - v.Longitude) * (M_PI / 180);
  double dy = (u.Latitude - v.Latitude) * (M_PI / 180);
  double a = sin(dy / 2) * sin(dy / 2) +
             sin(dx / 2) * sin(dx / 2) * cos(u.Latitude * (M_PI / 180)) *
                 cos(v.Latitude * (M_PI / 180));
  return r * (2 * asin(sqrt(a)));
}

double ezrs::DistanceHelper::Network(const Node_t &u, const Node_t &v) {
  // DANGER! GTree will segfault if this fails.
  return mHasGtree ? mGtree.search(u.Id, v.Id) : -1;
}
