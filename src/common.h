// author @J.T. Hu

#pragma once

#include <string>

namespace cargo {

  const std::string BJ_GTREE = "./bjrn.gtree";
  const std::string BJ_NODE;

  typedef struct {
    // longitude latitude
    double lng, lat;
  } node_t;

  typedef struct {
    int from, to;
    double weight;
  } edge_t;
}