// author @J.T. Hu

#pragma once

#include <string>

namespace cargo {

  const std::string BJ_GTREE = "./bjrn.gtree";
  const std::string BJ_NODE;
  const std::string BJ_TRIPS;

  const double PI = 3.141592653589793238462643383279502884L;

  typedef struct {
    // longitude latitude
    double lng, lat;
  } node_t;

  typedef struct {
    int from, to;
    double weight;
  } edge_t;

  enum Stop { PICKUP, DROPOFF };
  typedef struct {
    int node;
    int cid; // Customer id
    Stop type;
  } stop_t;

  double haversine(node_t u, node_t v) {
    double r  = 6372800.0; // radius of Earth (m)
    double x = (u.lng - v.lng)  * (PI / 180);
    double y = (u.lat - v.lat) * (PI / 180);
    double a  = sin(y / 2) * sin(y / 2) +
                sin(x / 2) * sin(x / 2) * cos(u.lat * (PI / 180)) *
                cos(v.lat * (PI / 180));
    // ROUND DOWN because the weights in the bj road network are truncated
    return std::floor(r * (2 * asin(sqrt(a))));
  }
}
