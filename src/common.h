// author @J.T. Hu

#ifndef COMMON_H
#define COMMON_H

#pragma once

#include <cmath>

/*
  const std::string BJ_GTREE = "./bjrn.gtree";
  const std::string BJ_NODE;
  const std::string BJ_TRIPS;
  */

const double PI = 3.141592653589793238462643383279502884L;

typedef struct
{
    // longitude latitude
    double lng;
    double lat;
} node_t;

typedef struct
{
    unsigned int from;
    unsigned int to;
    double weight;
} edge_t;

typedef struct
{
    unsigned int id;
    unsigned int from;
    unsigned int to;
    unsigned int tw_start; // time window
    unsigned int tw_end;
    int demand;
    bool finish;
} trip_t;

enum StopType
{
    PICKUP,
    DROPOFF
};

typedef struct
{
    int node_id;
    int cust_id; // Customer id
    StopType type;
} stop_t;

double haversine(node_t, node_t);

// double haversine(node_t u, node_t v)
// {
//     double r = 6372800.0; // radius of Earth (m)
//     double x = (u.lng - v.lng) * (PI / 180);
//     double y = (u.lat - v.lat) * (PI / 180);
//     double a = sin(y / 2) * sin(y / 2) +
//                sin(x / 2) * sin(x / 2) * cos(u.lat * (PI / 180)) *
//                    cos(v.lat * (PI / 180));
//     // ROUND DOWN because the weights in the bj road network are truncated
//     return std::floor(r * (2 * asin(sqrt(a))));
// }

#endif
