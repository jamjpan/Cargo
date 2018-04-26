// MIT License
//
// Copyright (c) 2018 the Cargo authors
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include <chrono>
#include <thread>
#include <iterator>
#include <vector>
#include <algorithm>

#include "libcargo/simulator.h"
#include "libcargo/types.h"
#include "libcargo/options.h"
#include "libcargo/file.h"
#include "libcargo/message.h"
#include "gtree/gtree.h"

namespace cargo {

#define DEBUG true

using opts::Options;
using file::ReadNodes;
using file::ReadEdges;
using file::ReadProblemInstance;
using  msg::MessageType;

// --------------------------------------------------------
// Simulator
// --------------------------------------------------------

Simulator::Simulator()
    : status_(SimulatorStatus::RUNNING), t_(0), tmin_(0), tmax_(0),
      count_active_(0) {
    // Better way to set these?
    PRINT.type = MessageType::DEFAULT;
    INFO.type = MessageType::INFO;
    WARN.type = MessageType::WARNING;
    ERROR.type = MessageType::ERROR;
    SUCCESS.type = MessageType::SUCCESS;
}

void Simulator::SetOptions(Options opts) {
    opts_ = opts;
}

void Simulator::Initialize() {
    // Wrap Initialize() in try..catch because these throw exceptions.
    INFO << "begin Simulator::Initialize()\n";
    PRINT << "read road network\n";
    ReadNodes(opts_.RoadNetworkPath, nodes_);
    ReadEdges(opts_.EdgeFilePath, edges_);
    PRINT << "read gtree\n";
    GTree::load(opts_.GTreePath);
    gtree_ = GTree::get();
    PRINT << "read problem instance\n";
    ReadProblemInstance(opts_.ProblemInstancePath, pi_);

    PRINT << "parse vehicles\n";
    for (const auto &kv : pi_.trips) {
        for (const auto &trip : kv.second) {
            // Set tmin_
            if (trip.early > tmin_)
                tmin_ = trip.early;
            if (trip.demand < 1) {
                vehicles_.insert(std::make_pair(trip.id, Vehicle(trip)));
                Vehicle &veh = vehicles_.at(trip.id);
                gtree_.find_path(veh.oid, veh.did, veh.route);
                veh.nnd = (-1)*edges_.at(veh.oid).at(veh.route.at(1));
                Stop o = {veh.id, veh.oid, StopType::VEHICLE_ORIGIN, -1};
                Stop d = {veh.id, veh.did, StopType::VEHICLE_DEST, -1};
                veh.sched.push_back(o);
                veh.sched.push_back(d);
                // Set tmax_
                if (veh.late > tmax_)
                    tmax_ = veh.late;
            }
        }
    }
    // Set the sleep time based on the time scale option.
    sleep_ = std::round((float)1000/opts_.Scale);
    INFO << "done Simulator::Initialize()\n";
}

void Simulator::MoveVehicles() {
    for (auto &kv : vehicles_) {
        Vehicle &veh = kv.second;
        #if DEBUG
        // PRINT << "veh" << veh.id << "(" << veh.route.at(veh.lv_node) << ")"
        //       << "nnd=" << veh.nnd << " early=" << veh.early << " active=" << veh.is_active << std::endl;
        #endif
        if (t_ > veh.early) {
            veh.nnd += opts_.VehicleSpeed;
            while (veh.nnd >= 0 && veh.is_active) {
              veh.lv_node++;
                #if DEBUG
                INFO << "veh" << veh.id << " moved from " << veh.route.at(veh.lv_node-1)
                     << " to " << veh.route.at(veh.lv_node) << std::endl;
                #endif
              if (veh.route.at(veh.lv_node) == veh.sched.at(veh.lv_stop + 1).node_id) {
                  veh.lv_stop++;
                  if (veh.sched.at(veh.lv_stop).type == StopType::VEHICLE_DEST) {
                      veh.is_active = false;
                      count_active_--;
                  }
                  else if (veh.sched.at(veh.lv_stop).type == StopType::CUSTOMER_ORIGIN)
                      veh.load++;
                  else
                      veh.load--;
               }
               veh.nnd -= edges_.at(veh.route.at(veh.lv_node-1)).at(veh.route.at(veh.lv_node));
            }
        }
    }
}

bool Simulator::Run() {
    // Blocks until
    // (1) minimum duration reached (t_ > tmin_) and
    // (2) no more active vehicles (count_active_ == 0)
    while (!(t_ > tmin_ && count_active_ == 0)) {
        // Start the clock for this interval
        auto t_start = std::chrono::high_resolution_clock::now();
        PRINT << "tmin=" << tmin_ << "/t=" << t_ << "/tmax=" << tmax_ << std::endl;

        // Move all the vehicles
        if (t_ > 0)
            MoveVehicles();
        // Broadcast new trips as customers or vehicles
        if (pi_.trips.find(t_) != pi_.trips.end()) {
            for (auto trip : pi_.trips.at(t_)) {
                if (trip.demand < 0) {
                    count_active_++;
                    // TODO: broadcast a new vehicle
                } else {
                    // TODO: customer_online() embark
                }
            }
        }

        // If the elapsed time is too long, the simulator's run thread is too
        // slow to fit inside real-time. Reset the Scale option to be 1. If the
        // thread is still too slow, increase the scale; but then, the
        // simulation won't be real-time anymore; it will be slowed down.
        auto t_end = std::chrono::high_resolution_clock::now();
        int elapsed = std::round(
            std::chrono::duration<double, std::milli>(t_end - t_start).count());
        #if DEBUG
        WARN << elapsed << " ms" << std::endl;
        #endif
        if (elapsed > sleep_) {
            ERROR << "Scale too big, exiting\n";
            return true;
        }

        // Sleep until the next time interval
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ - elapsed));

        // Advance the simulation time
        t_++;
    }
    return false;
}

} // namespace cargo
