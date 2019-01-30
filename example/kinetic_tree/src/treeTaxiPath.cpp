// Copyright the original authors
// See https://github.com/uakfdotb/tsharesim
//
// (The original code is unlicensed. The implication is that the code falls
// under default copyright law, preventing us (Cargo) from freely using it.
// So... use of the code here is illegal, until permission can be obtained
// from the author...)
//
// Portions copyright 2018 the Cargo authors
//
#include <algorithm> /* std::max, std::min */
#include <iostream>
#include <queue>
#include <vector>

#include "treeTaxiPath.h"
//#include "includes.h"
//#include "shortestPath.h"
//#include "taxiPath.h"
//#include "vertex.h"

#include "libcargo.h"

// constructs a root gentree node
//TreeNode ::TreeNode::TreeNode(ShortestPath *shortestPath, vertex *vert) {
TreeNode::TreeNode(NodeId orig, NodeId dest, NodeId owner) {
  parent = nullptr;
  this->loc = orig;
  this->dest = dest;
  this->owner = owner;

  insert_uid = -1;
  start = true;

  time = 0;
  absoluteTime = 0;
  //rootTime = 0;
  rootTime = Cargo::now();
  pickupRemoved = false;
  limit = -1;
  totalSlackTime = 0;
}

// constructs a normal gentree node
//TreeNode ::TreeNode(TreeNode *parent, vertex *vert, bool start, long insert_uid,
//                    double limit, bool pickupRemoved, double totalSlackTime,
//                    ShortestPath *shortestPath) {
TreeNode::TreeNode(TreeNode *parent, NodeId loc, NodeId owner, bool start, long insert_uid,
                    double limit, bool pickupRemoved, double totalSlackTime) {
  this->parent = parent;
  this->loc = loc;
  this->owner = owner;
  this->dest = (parent != nullptr? parent->dest : -1);
  this->insert_uid = insert_uid;
  this->start = start;
  this->limit = limit;
  this->pickupRemoved = pickupRemoved;
  this->totalSlackTime = totalSlackTime; // our actual slack time can only be
                                         // less, and it's easier to recalculate
                                         // later
  //this->shortestPath = shortestPath;

  if (this->parent != nullptr) {
    //time = shortestPath->shortestDistance(parent->vert, vert);
    time = get_shortest_path(this->parent->loc, loc)/Cargo::vspeed(); // <-- libcargo/distance.h
    absoluteTime = this->parent->absoluteTime + time;

    // Really hacky...
    if ((this->parent)->parent == nullptr)
      rootTime = time + Cargo::now();
    else
      rootTime = this->parent->rootTime + time;

    // find our pairTime if this is a dropoff who's pair is still a sub-root
    // tree node
    if (!start && !pickupRemoved) {
      TreeNode *n = this->parent;
      while (n->insert_uid != insert_uid) {
        n = n->parent;
      }

      pairTime = absoluteTime - n->absoluteTime;
    } else {
      pairTime = 0;
    }
  } else {
    time = 0;
    absoluteTime = 0;
    //rootTime = 0;
    rootTime = Cargo::now();
    pairTime = 0;
  }
}

// large constructor in case feasibility checking has already been done
//TreeNode ::TreeNode(TreeNode *parent, vertex *vert, bool start, long insert_uid,
//                    double limit, bool pickupRemoved, double time,
//                    double pairTime, double totalSlackTime,
//                    ShortestPath *shortestPath) {
TreeNode::TreeNode(TreeNode *parent, NodeId loc, NodeId owner, bool start, long insert_uid,
                    double limit, bool pickupRemoved, double time,
                    double pairTime, double totalSlackTime) {
  this->parent = parent;
  this->loc = loc;
  this->owner = owner;
  this->dest = (parent != nullptr? parent->dest : -1);
  this->insert_uid = insert_uid;
  this->start = start;
  this->limit = limit;
  this->pickupRemoved = pickupRemoved;
  this->totalSlackTime = totalSlackTime;

  this->time = time;
  absoluteTime = parent->absoluteTime + time;
  rootTime = parent->rootTime + time;
  //this->shortestPath = shortestPath;

  this->pairTime = pairTime;
}

TreeNode ::~TreeNode() {
  for (size_t i = 0; i < children.size(); i++) {
    delete children[i];
  }
}

// same as creating a new node with the given parameters, but doesn't allocate
// memory if it's infeasible
// instead, if infeasible, this will return null
//TreeNode *TreeNode ::safeConstructNode(TreeNode *parent, vertex *vert,
//                                       bool start, long insert_uid,
//                                       double limit, bool pickupRemoved,
//                                       double totalSlackTime,
//                                       ShortestPath *shortestPath) {
TreeNode *TreeNode::safeConstructNode(TreeNode *parent, NodeId loc, NodeId owner, bool start,
                                      long insert_uid, double limit,
                                      bool pickupRemoved,
                                      double totalSlackTime) {
  //double time = shortestPath->shortestDistance(parent->vert, vert);
  double time = get_shortest_path(parent->loc, loc)/Cargo::vspeed();

  if (start || pickupRemoved) {
    if (parent->rootTime + time > limit) {
      return NULL;
    } else {
      //return new TreeNode(parent, vert, start, insert_uid, limit, pickupRemoved,
      //                    time, 0, totalSlackTime, shortestPath);
      return new TreeNode(parent, loc, owner, start, insert_uid, limit, pickupRemoved,
                          time, 0, totalSlackTime);
    }
  } else {
    TreeNode *n = parent;
    while (n->insert_uid != insert_uid) {
      n = n->parent;
    }

    // double pairTime = time + parent->absoluteTime - n->absoluteTime;
    //double pairTime = time + parent->absoluteTime;
    double pairTime = time + parent->rootTime;

    if (pairTime > limit) {
      return NULL;
    } else {
      //return new TreeNode(parent, vert, start, insert_uid, limit, pickupRemoved,
      //                    time, pairTime, totalSlackTime, shortestPath);
      return new TreeNode(parent, loc, owner, start, insert_uid, limit, pickupRemoved,
                          time, pairTime, totalSlackTime);
    }
  }
}

// notify the gentree that the taxi has moved a certain distance
// if pair is valid (-1 is definitely invalid), then it means that pickup point
// has been reached so corresponding dropoff point will be updated with
// pickupRemoved = true this way, we can decrease the limit for those dropoff
// points whose corresponding pickup has been reached
/** Adjusts the "time" properties of each node and their children to match
 * vehicle's motion */
//void TreeNode ::step(double distanceTraveled, long pair) {
void TreeNode ::step(double elapsed, long pair) {
  if (limit > -1) {
    // rootTime -= distanceTraveled;    // dont need because we move it in handle_vehicle

    // if (start || pickupRemoved) {
    //   limit -= distanceTraveled;
    // } else if (pair == insert_uid) {
    //   pickupRemoved = true; // don't decrease pairtime for this one because the
    //                         // current location (root) is now the pair
    // }
    if (pair == insert_uid)
      pickupRemoved = true;
    //limit -= (distanceTraveled/Cargo::vspeed());
  }

  this->rootTime += elapsed;

  for (size_t i = 0; i < children.size(); i++) {
    //children[i]->step(distanceTraveled, pair);
    children[i]->step(elapsed, pair);
  }
}

// returns total path time, and stores the best child
// uses simple greedy algorithm
// should be called each time path may be affected by a node update (i.e., an
// insert; no need when we simple do a step)
double TreeNode ::bestTime() {
  if (children.size() > 0) {
    //double bestTime = 10000000000; // todo: numeric limits<double>
    double bestTime = InfDbl;
    bestChild = -1;

    for (size_t i = 0; i < children.size(); i++) {
      double child_best = children[i]->bestTime();
      double child_time = children[i]->time;
      double childBestTime = child_best + child_time;

      if (childBestTime < bestTime) {
        bestTime = childBestTime;
        bestChild = i;
      } else if (bestChild == -1) {
          //std::cout << "errrrrr " << children[i]->bestTime() << "," << children[i]->time << "," << bestTime << std::endl;
      }
    }

    return bestTime;
  } else { // <-- no children; we are at a leaf node
    bestChild = -1;
    //return 0;
    if (this->dest == -1)
      return 0;
    else
      return get_shortest_path(this->loc, this->dest)/Cargo::vspeed();

  }
}

int TreeNode ::getNumberNodes() {
  int numNodes = 1;

  for (size_t i = 0; i < children.size(); i++) {
    numNodes += children[i]->getNumberNodes();
  }

  return numNodes;
}

// returns whether or not copy was successful; if not successful, parent maybe
// should delete
bool TreeNode ::copyNodes(std::vector<TreeNode *> *sourcePtr,
                          std::vector<TreeNode *> *doInsertPtr) {
  // recursively copy nodes from other trees if requested
  if (sourcePtr != nullptr) {
      std::vector<TreeNode *> source = *sourcePtr;
    bool fail = false;

    for (size_t i = 0; i < source.size(); i++) {
      TreeNode *myCopy = source[i]->copy(this);

      if (myCopy->feasible()) {
        children.push_back(myCopy);

        if (!myCopy->copyNodes(&(source[i]->children), nullptr)) {
          // our child has no feasible branches in it's subtree, delete
          children.pop_back();
          delete myCopy;
          fail = true;
        }
      } else {
        // child is not feasible, delete
        delete myCopy;
        fail = true;
      }
    }

    // check if we're infeasible
    // make sure to check fail flag because we might not have had to copy any
    // nodes
    if (fail && children.empty()) {
      // we have no feasible branches in our subtree, fail
      return false;
    }
  }

  // inserted new nodes into this tree if requested
  if (doInsertPtr != nullptr) {
    std::vector<TreeNode *> doInsert = *doInsertPtr;

    if (doInsert.size() > 0) {
      // take the first node to insert and put it directly underneath us
      TreeNode *insertCopy = doInsert[0]->copySafe(this);

      if (insertCopy) {
        bool fail = false;

        // first, copy other branches to our inserted copy
        // ignore slack time since this algorithm doesn't use it correctly
        if (children.empty() || checkSlack(insertCopy) || true) {
          if (!(insertCopy->copyNodes(&children, nullptr))) {
            fail = true;
          }
        } else {
          fail = true;
        }

        if (!fail && doInsert.size() >= 2) {
          // give the insertCopy our doInsert, but with first element that was
          // already copied by it removed
            std::vector<TreeNode *> doInsertClone;
          for (size_t i = 1; i < doInsert.size(); i++) {
            doInsertClone.push_back(doInsert[i]);
          }

          if (!insertCopy->copyNodes(nullptr, &doInsertClone)) {
            fail = true;
          }
        }

        // give other children our insertCopy in full
        // note that this must be executed after insertCopy updates because
        // insertCopy copies nodes from these children
        for (size_t it = 0; it < children.size(); it++) {
          TreeNode *child = children[it];

          if (!(child->copyNodes(nullptr, doInsertPtr))) {
            // child failed, so delete from children
            delete child;
            children.erase(children.begin() + it);
            it--;
          }
        }

        if (!fail) {
          // lastly, insert insertCopy into our children so we don't have
          // infinite loop
          children.push_back(insertCopy);
        } else if (children.empty()) {
          delete insertCopy;
          return false;
        } else {
          delete insertCopy;
        }
      } else {
        // if our inserted copy is not feasible, no other path will be feasible
        //  since we have to insert the same node to our children.. assuming
        //  shortest paths are shortest
        return false;
      }
    }
  }

  return true;
}

// returns whether or not this node is feasible
bool TreeNode ::feasible() {
  return rootTime < limit;
}

// this is this node's slack time
// field totalSlackTime considers children's slack times; this does not
double TreeNode ::slackTime() {
  if (start || pickupRemoved) {
    return limit - rootTime;
  } else {
    return 100000;
  }
}

// calculates total slack time and stores in totalSlackTime field
double TreeNode ::calculateTotalSlackTime() {
  if (children.empty()) {
    totalSlackTime = slackTime();
  } else {
    totalSlackTime = 0;

    for (size_t i = 0; i < children.size(); i++) {
      totalSlackTime =
          std::max(children[i]->calculateTotalSlackTime(), totalSlackTime);
    }

    // our slack time always has to be satisfied though
    totalSlackTime = std::min(totalSlackTime, slackTime());
  }

  return totalSlackTime;
}

// checks that newNode works with at least one of our children
bool TreeNode ::checkSlack(TreeNode *newNode) {
  for (size_t i = 0; i < children.size(); i++) {
    // compare slack time to the increased time due to detour
    // increased time is new tree distance minus old tree distance
    //if (children[i]->totalSlackTime >
    //    shortestPath->shortestDistance(vert, newNode->vert) +
    //        shortestPath->shortestDistance(newNode->vert, children[i]->vert) -
    //        children[i]->time) {
    if (children[i]->totalSlackTime >
        get_shortest_path(loc, newNode->loc)/Cargo::vspeed() +
        get_shortest_path(newNode->loc, children[i]->loc)/Cargo::vspeed() -
        children[i]->time) {
      return true;
    }
  }

  return false;
}

// creates a copy of this node with a new parent
TreeNode *TreeNode ::copy(TreeNode *new_parent) {
  //TreeNode *copy = new TreeNode(new_parent, vert, start, insert_uid, limit,
  //                              pickupRemoved, totalSlackTime, shortestPath);
  TreeNode *copy = new TreeNode(new_parent, loc, owner, start, insert_uid, limit,
                                pickupRemoved, totalSlackTime);
  return copy;
}

// creates a copy of this node with a new parent
// or returns null if infeasible
TreeNode *TreeNode ::copySafe(TreeNode *new_parent) {
  //TreeNode *copy =
  //    safeConstructNode(new_parent, vert, start, insert_uid, limit,
  //                      pickupRemoved, totalSlackTime, shortestPath);
  TreeNode *copy = safeConstructNode(new_parent, loc, owner, start, insert_uid, limit,
                                     pickupRemoved, totalSlackTime);
  return copy;
}

// creates a copy of this node (identical parent)
TreeNode *TreeNode ::clone() {
  //TreeNode *clone = new TreeNode(parent, vert, start, insert_uid, limit,
  //                               pickupRemoved, totalSlackTime, shortestPath);
  TreeNode *copy = new TreeNode(parent, loc, owner, start, insert_uid, limit,
                                 pickupRemoved, totalSlackTime);

  // Propogate the new rootTime to the child copies
  for (size_t i = 0; i < this->children.size(); i++) {
    TreeNode* child_copy = this->children.at(i)->clone();
    child_copy->parent = copy;
    child_copy->rootTime = child_copy->time + copy->rootTime;
    copy->children.push_back(child_copy);
    // copy->children.push_back(this->children[i]->clone());
    // copy->children[i]->parent = copy;
    // copy->children[i]->rootTime
    //   = copy->children[i]->parent->rootTime + copy->children[i]->time;;  // <--
  }

  copy->dest = this->dest;

  return copy;
}

void TreeNode ::print() {
  std::cout.precision(12);
  std::cout << "Node belonging to " << owner << " at " << loc << ": " << rootTime << "/" << (start ? "pickup" : "dropoff")
       << "/" << limit << " [";

  for (size_t i = 0; i < children.size(); i++) {
    std::cout << children[i]->loc << "  ";
  }

  std::cout << "]" << std::endl;

  for (size_t i = 0; i < children.size(); i++) {
    children[i]->print();
  }
}

// ************************************************************************** //
//TreeTaxiPath ::TreeTaxiPath(ShortestPath *shortestPath, vertex *curr)
//    : TaxiPath(shortestPath, curr) {
TreeTaxiPath ::TreeTaxiPath(NodeId orig, NodeId dest, NodeId owner) {
  root = new TreeNode(orig, dest, owner);
  flag = false;

  nextPair = 0;
}

TreeTaxiPath ::~TreeTaxiPath() { delete root; }

void TreeTaxiPath ::moved(double distance) {
  root->step(distance, -1);
}

void TreeTaxiPath::move(const NodeId& curloc) {
  root->loc = curloc;
}

NodeId TreeTaxiPath::head() {
  return root->loc;
}

NodeId TreeTaxiPath::next() {
  if (root->children.size() > 0)
    return root->children.at(root->bestChild)->loc;
  else
    return -1;
}

// ************************ //
// 1) Call value() to "test" insert a customer into a kinetic tree
// 2) Call cancel() to cancel the "test", and call push to accept it
//
//double TreeTaxiPath ::value(vertex *curr, vertex *source, vertex *dest) {
double TreeTaxiPath ::value(NodeId source, NodeId dest, NodeId owner,
                            DistInt pickup_rng, DistInt max_travel) {
  flag = true;

  // root->loc = loc;

  //TreeNode *pick = new TreeNode(NULL, source, true, nextPair++,
  //                              pickupConstraint, false, 0, shortestPathC);
  // TreeNode *pick =
  //    new TreeNode(nullptr, source, true, nextPair++, pickup_rng, false, 0);
  // TreeNode *pick =  // <<-- the `limit` parameter should be latest possible pickup time, converted to distance? just use max_travel for now
  //     new TreeNode(nullptr, source, true, nextPair++, max_travel, false, 0);
   TreeNode *pick =
       new TreeNode(nullptr, source, owner, true, nextPair++, pickup_rng, false, 0);
  //TreeNode *drop = new TreeNode(
  //    NULL, dest, false, pick->insert_uid,
  //    serviceConstraint * shortestPathC->shortestDistance(source, dest), false,
  //    0, shortestPathC);
  TreeNode *drop = new TreeNode(nullptr, dest, owner, false, pick->insert_uid,
                                max_travel, false, 0);

  std::vector<TreeNode *> doInsert;
  doInsert.push_back(pick);
  doInsert.push_back(drop);

  rootTemp = root->clone();
  bool copyResult = rootTemp->copyNodes(nullptr, &doInsert);

  // clear our insertion vector elements
  delete pick;
  delete drop;

  // update node values to reflect insertion
  double time = rootTemp->bestTime();
  rootTemp->calculateTotalSlackTime();

  // bestTime() does not compensate for current time,
  // so we don't have to here either!
  if (time != 0 && copyResult) {
    // return (time-Cargo::now())*Cargo::vspeed();
    return time*Cargo::vspeed();
  } else
    return -1;
}

void TreeTaxiPath ::cancel() {
  if (flag) {
    delete rootTemp;

    flag = false;
  }
}

void TreeTaxiPath ::push() {
  delete root;
  root = rootTemp;
  rootTemp = nullptr;

  flag = false;
}

/** step through the kinetic tree
 * A call to TreeTaxiPath::step() will move the root to its best child,
 * deleting adjacent childs. Returns true if the best child is a cust dropoff */
bool TreeTaxiPath ::step(/*bool move*/) {
  int rootBestChild = root->getBestChild();

  for (int i = root->children.size() - 1; i >= 0; i--) {
    if (i != rootBestChild) {
      delete root->children[i];
      root->children.erase(root->children.begin() + i);
    }
  }

  TreeNode *removedChild =
      root->children.back(); // should be only remaining node
  root->children.pop_back();

  for (size_t i = 0; i < removedChild->children.size(); i++) {
    root->children.push_back(removedChild->children[i]);
    root->children[i]->parent = root;
  }

  root->bestChild = removedChild->getBestChild();
  root->step(0, removedChild->insert_uid); // todo: if move is true we should
                                           // try to move from current root
                                           // position to removed node

  bool droppedPassenger = !(removedChild->start);
  removedChild->children.clear();
  delete removedChild;

  return droppedPassenger; // return true if we dropped a passenger
}

NodeId TreeTaxiPath::get_dest() {
  return root->dest;
}

void TreeTaxiPath::set_dest(NodeId nid) {
  root->dest = nid;
}

// std::queue<NodeId> TreeTaxiPath ::next() {
//     std::queue<NodeId> ret;
//   return ret; // todo
// }

//std::queue<NodeId> TreeTaxiPath ::curr(NodeId loc) {
//  root->loc = loc;
//
//  if (root->children.size() > 0) {
//    int rootBestChild = root->getBestChild();
//    //return shortestPathC->shortestPath(root->vert,
//    //                                   root->children[rootBestChild]->vert);
//  } else {
//      std::queue<NodeId> emptyqueue;
//    return emptyqueue;
//  }
//}

void TreeTaxiPath ::printPoints() {
  root->print();

  std::cout << std::endl << "**best**" << std::endl;

  TreeNode *curr = root;

  while (curr->children.size() > 0) {
    curr = curr->children[curr->getBestChild()];
    std::cout << curr->loc << std::endl;
  }
}

void TreeTaxiPath ::printTempPoints() {
  rootTemp->print();

  std::cout << std::endl << "**best**" << std::endl;

  TreeNode *curr = rootTemp;

  while (curr->children.size() > 0) {
    curr = curr->children[curr->getBestChild()];
    std::cout << curr->loc << std::endl;
  }
}

void TreeTaxiPath::printStopSequence(std::vector<std::tuple<NodeId, NodeId, bool>>& out) {
  TreeNode* curr = root;
  auto outbound = std::make_tuple(curr->owner, curr->loc, curr->start);
  out.push_back(outbound);
  while (curr->children.size() > 0) {
    curr = curr->children[curr->getBestChild()];
    outbound = std::make_tuple(curr->owner, curr->loc, curr->start);
    out.push_back(outbound);
  }
  outbound = std::make_tuple(root->owner, curr->dest, curr->start);
  out.push_back(outbound); // <-- at a leaf node now
}

void TreeTaxiPath::printTempStopSequence(std::vector<std::tuple<NodeId, NodeId, bool>>& out) {
  TreeNode* curr = rootTemp;
  auto outbound = std::make_tuple(curr->owner, curr->loc, curr->start);
  out.push_back(outbound);
  while (curr->children.size() > 0) {
    curr = curr->children[curr->getBestChild()];
    outbound = std::make_tuple(curr->owner, curr->loc, curr->start);
    out.push_back(outbound);
  }
  outbound = std::make_tuple(root->owner, curr->dest, curr->start);
  out.push_back(outbound); // <-- at a leaf node now
}

//double TreeTaxiPath ::euclidean(double ax, double ay, double bx, double by) {
//  double d1 = ax - bx;
//  double d2 = ay - by;
//  return sqrt(d1 * d1 + d2 * d2);
//}

