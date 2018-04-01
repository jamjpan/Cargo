//
// Copyright(c) 2018 James J. Pan
// Distributed under the MIT License (http://opensource.org/licenses/MIT)
//
#pragma once

#define LIBEZRS_VERSION "0.01.0"

#include <ctime> // for time_t
#include "gtree/GTree.h"

namespace ezrs {

typedef struct {
  int Id;
  double Longitude;
  double Latitude;
} Node_t;

typedef enum { WAITING = -1, INPROGRESS = 1, COMPLETED = 0 } RequestFlag;

typedef enum { DROPOFF = 0, PICKUP = 1 } StopAction;

class DistanceHelper {
public:
  DistanceHelper();
  DistanceHelper(GTree::G_Tree &);
  double Euclidean(const Node_t &, const Node_t &);
  double Haversine(const Node_t &, const Node_t &);
  double Network(const Node_t &, const Node_t &);

private:
  bool mHasGtree;
  GTree::G_Tree mGtree;
};

class Trip {
public:
  Trip(time_t, Node_t, Node_t);
  time_t Created() const;
  Node_t Origin() const;
  Node_t Destination() const;

private:
  time_t mCreated;
  Node_t mOrigin;
  Node_t mDestination;
};

class Request : public Trip {
public:
  Request(time_t, Node_t, Node_t);
  int Id() const;
  int Status() const;
  double BaseDistance() const;

private:
  int mId;
  int mStatus;
  double mBaseDistance;
};

class Stop {
public:
  Stop(Node_t, StopAction);
  Node_t Location() const;
  double DistanceTo() const;
  int RequestId() const;

private:
  Node_t mLocation;
  double mDistanceTo;
  int mRequestId;
};

}
