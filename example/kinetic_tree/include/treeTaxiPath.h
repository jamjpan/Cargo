// Copyright the original authors
// See https://github.com/uakfdotb/tsharesim
//
// (The original code is unlicensed. The the code falls under default
// copyright law. We... have not obtained permission from the author.)
//
// Portions copyright 2018 the Cargo authors
#ifndef KINETICTREES_INCLUDE_TREETAXIPATH_H_
#define KINETICTREES_INCLUDE_TREETAXIPATH_H_
#include <queue>
#include <vector>

#include "libcargo.h"

using namespace cargo;

// class TaxiPath;
// class TreeNode;
// class ShortestPath;

class TreeNode {
 public:
  bool start; // true if this is a pickup point, false if this is a dropoff
              // point
  bool pickupRemoved; // true if the pickup has already been reached, in which
                      // limit should decrease for dropoff point
  NodeId owner;

  TreeNode *parent;
  std::vector<TreeNode *> children;
  //ShortestPath *shortestPath;

  long insert_uid; // unique id for the inserted pair

  double time; // time relative to parent
  double absoluteTime;
  double rootTime; // time relative to root
  double pairTime; // if start is false, this is the time between pickup
                   // location and this dropoff point

  double totalSlackTime; // slack time for this node, considering all of its
                         // children

  double limit; // tolerance threshold on rootTime or pairTime (i.e., the
                // constraint)
  int bestChild; // best child following this one in a route

  //vertex *vert;
  NodeId loc;
  NodeId dest; // <-- the final destination of the vehicle

  // constructs a root gentree node
  //TreeNode(ShortestPath *shortestPath, vertex *vert);
  // Cargo: vehicles have destinations specified. Param1: orig; param2: dest
  TreeNode(NodeId, NodeId, NodeId);

  // constructs a normal gentree node
  //TreeNode(TreeNode *parent, vertex *vert, bool start, long insert_uid,
  //         double limit, bool pickupRemoved, double totalSlackTime,
  //         ShortestPath *shortestPath);
  TreeNode(TreeNode *, NodeId, NodeId, bool, long, double, bool, double);

  // large constructor in case feasibility checking has already been done
  //TreeNode(TreeNode *parent, vertex *vert, bool start, long insert_uid,
  //         double limit, bool pickupRemoved, double time, double pairTime,
  //         double totalSlackTime, ShortestPath *shortestPath);
  TreeNode(TreeNode *, NodeId, NodeId, bool, long, double, bool, double, double,
           double);

  ~TreeNode();

  // same as creating a new node with the given parameters, but doesn't allocate
  // memory if it's infeasible
  // instead, if infeasible, this will return null
  //static TreeNode *safeConstructNode(TreeNode *parent, vertex *vert, bool start,
  //                                   long insert_uid, double limit,
  //                                   bool pickupRemoved, double totalSlackTime,
  //                                   ShortestPath *shortestPath);
  static TreeNode *safeConstructNode(TreeNode *, NodeId, NodeId, bool, long, double,
                                     bool, double);

  // notify the gentree that the taxi has moved a certain distance
  // if pair is valid (-1 is definitely invalid), then it means that pickup
  // point has been reached so corresponding dropoff point will be updated with
  // pickupRemoved = true this way, we can decrease the limit for those dropoff
  // points whose corresponding pickup has been reached
  void step(double distanceTraveled, long pair);

  // returns total path time, and stores the best child
  // uses simple DFS-based shortest path algorithm
  // should be called each time path may be affected by a node update (i.e., an
  // insert; no need when we simple do a step)
  double bestTime();

  // returns total number of nodes in the tree
  int getNumberNodes();

  // returns the best child to follow this node in a route identified by
  // bestTime()
  int getBestChild() { return bestChild; }

  // returns whether or not copy was successful; if not successful, parent maybe
  // should delete
  bool copyNodes(std::vector<TreeNode *> *source, std::vector<TreeNode *> *doInsert);

  // returns whether or not this node is feasible
  bool feasible();

  // this is this node's slack time
  // field totalSlackTime considers children's slack times; this does not
  double slackTime();

  // calculates total slack time and stores in totalSlackTime field
  double calculateTotalSlackTime();

  // checks that newNode works with at least one of our children
  bool checkSlack(TreeNode *newNode);

  // creates a copy of this node with a new parent
  TreeNode *copy(TreeNode *new_parent);

  // creates a copy of this node with a new parent
  // or returns null if infeasible
  TreeNode *copySafe(TreeNode *new_parent);

  // creates a copy of this node (identical parent)
  TreeNode *clone();

  void print();
};

//class TreeTaxiPath : public TaxiPath {
class TreeTaxiPath {
 private:
  TreeNode *root;
  TreeNode *rootTemp;
  bool flag; // 1 if value was just called, 0 otherwise

  long nextPair;

  double pickupConstraint;
  double serviceConstraint;

  //vertex *curr_vert; // <-- from TaxiPath
  NodeId curr_loc;

 public:
  // currNode is index of the vertex that taxi is currently at or heading towards
  //TreeTaxiPath(ShortestPath *shortestPath, vertex *curr);
  /* Cargo: vehicles have destinations; param1: orig, param2: dest */
  TreeTaxiPath(NodeId, NodeId, NodeId);
  ~TreeTaxiPath();

  //virtual void moved(double distance);
  void moved(double distance);

  /* Cargo: use move() to move the root to a new node */
  void move(const NodeId &);

  /* Return the node id of root */
  NodeId head();
  NodeId next();

  // tests pushing (source, dest)
  //virtual double value(vertex *curr, vertex *source, vertex *dest);
  /* Cargo: value() uses a global pickup/service constraint for source and
   * dest nodes. Cargo models each customer has its own constraints, based on
   * his time window. We modify value to accept specific constraints. */
  double value(NodeId source, NodeId dest, NodeId owner,
               DistInt pickup_rng, DistInt max_travel);

  // cancels the value() call
  //virtual void cancel();
  void cancel();

  // completes the pushing after value() is called
  //virtual void push();
  void push();

  // next() becomes curr(); goes one vertex forward, when curr() is passed
  //virtual bool step(bool move);
  bool step();

  // set new dest (for permanent taxis)
  NodeId get_dest();
  void set_dest(NodeId);

  // returns next vertex that the taxi should goto after curr() is passed; -1 if no next
  //virtual queue<vertex *> next(); 
  // std::queue<NodeId> next();

  // returns the vertex that the taxi is currently at or heading towards
  //virtual queue<vertex *> curr(vertex *curr);
  //std::queue<NodeId> curr(NodeId);

  //virtual int size() { return 0; }
  int size() { return 0; }

  // prints current point array
  //virtual void printPoints();
  void printPoints();
  void printTempPoints();

  // Output sequence along best path
  void printStopSequence(std::vector<std::tuple<NodeId, NodeId, bool>> &);
  void printTempStopSequence(std::vector<std::tuple<NodeId, NodeId, bool>> &);

  // calls getNumberNodes on the root node
  //virtual int getNumberNodes() {
  int getNumberNodes() {
    return root->getNumberNodes();
  }

  // whether this taxipath supports dynamic constraints
  //virtual bool dynamicConstraints() {
  bool dynamicConstraints() {
    return true;
  }

  //virtual void setConstraints(double pickup_constraint, double service_constraint) {
  void setConstraints(double pickup_constraint, double service_constraint) {
    pickupConstraint = pickup_constraint;
    serviceConstraint = service_constraint;
  };

//private:
//  double euclidean(double ax, double ay, double bx,
//                   double by); // returns shortest path from source to dest
};

#endif  // KINETICTREES_INCLUDE_TREETAXIPATH_H_

