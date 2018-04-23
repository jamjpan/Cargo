#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include "common.h"
#include "gtree/GTree.h"

class Sim
{
  public:
    Sim(std::string gtree_path, std::string trips_path, std::string roads_path,
        unsigned int scale, unsigned int speed, unsigned int update_timing);
    void Init();
    void Start();
    void Stop();
    void UpdateRoute(unsigned int, const std::vector<unsigned int>&);
    int Size();

  private:
    std::unordered_map<unsigned int, node_t> nodes_;
    std::unordered_map<unsigned int, edge_t> edges_;
    std::unordered_map<unsigned int, trip_t> trips_;
    std::unordered_map<unsigned int, std::vector<unsigned int> > trips_idx_; // time -> list of trips
    std::unordered_map<unsigned int, std::vector<unsigned int> > routes_;
    std::unordered_map<unsigned int, unsigned int> current_; // vehicle_id(trip_id) -> node_id
    std::unordered_map<unsigned int, int> residual_; // vehicle_id -> residual travel distance, may be negative value
    GTree::G_Tree gtree_;
    std::string gtree_path_;
    std::string trips_path_;
    std::string roads_path_;
    std::string instance_name_;

    bool stop_;
    unsigned int scale_;
    unsigned int speed_;
    unsigned int update_timing_;
    unsigned int customer_count_;
    unsigned int vehicle_count_;
};
