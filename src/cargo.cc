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
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include <algorithm>
#include <errno.h>
#include <iterator>
#include <thread>

#include "libcargo/cargo.h"

namespace cargo {

Cargo::Cargo(const Options& opt)
    : print_info(MessageType::Info), print_warning(MessageType::Warning),
      print_error(MessageType::Error), print_success(MessageType::Success)
{

    print_out << "Initializing Cargo\n";

    print_out << "Reading nodes...";
    size_t nnodes = read_nodes(opt.path_to_roadnet, nodes_, bbox_);
    print_out << nnodes << "\n";
    print_out << "\tBounding box: ("
        << bbox().lower_left.lng << ", " << bbox().lower_left.lat << "), ("
        << bbox().upper_right.lng << ", " << bbox().upper_right.lat << ")\n";

    print_out << "Reading edges...";
    size_t nedges = read_edges(opt.path_to_edges, edges_);
    print_out << nedges << "\n";

    print_out << "Reading gtree...";
    GTree::load(opt.path_to_gtree);
    gtree_ = GTree::get();
    print_out << "Done\n";

    print_out << "Reading problem...";
    size_t ntrips = read_problem(opt.path_to_problem, probset_);
    print_out << ntrips << "\n";
    print_out << "\t" << name() << " on " << road_network() << "\n";

    sleep_interval_ = std::round((float)1000 / opt.time_multiplier);

    print_success << "Cargo initialized!" << std::endl;;
}

const BoundingBox& Cargo::bbox() const { return bbox_; }
const std::string& Cargo::name() { return probset_.name(); }
const std::string& Cargo::road_network()
{
    return probset_.road_network();
}

void Cargo::run() {}

//void Cargo::Initialize() {
//    PRINT << "parse vehicles\n"; for (const auto &kv : pi_.trips) { for
//    (const auto &trip : kv.second) { // Set tmin_ if (trip.early > tmin_)
//    tmin_ = trip.early; if (trip.demand < 1) {
//    vehicles_.insert(std::make_pair(trip.id, Vehicle(trip))); Vehicle &veh =
//    vehicles_.at(trip.id); gtree_.find_path(veh.oid, veh.did, veh.route);
//#if SPBUG if (veh.id == SP) { for (auto &item : veh.route) INFO << item << "
//"; INFO << std::endl; } #endif veh.nnd = (-1) *
//edges_.at(veh.oid).at(veh.route.at(1)); // no need to set, already in
//constructor // veh.lv_node = 0; // >Hu modified for the change of Stop Stop o
//= {veh.id, veh.oid, StopType::VEHICLE_ORIGIN, trip.early, trip.early}; Stop d
//= {veh.id, veh.did, StopType::VEHICLE_DEST, -1, trip.late};
//veh.sched.push_back(o); veh.sched.push_back(d); // Set tmax_ if (veh.late >
//tmax_) tmax_ = veh.late;
//
//                // add vehicle into database da_.AddVehicle(&veh); } } }
//}

void Cargo::step() {
//    for (auto &kv : vehicles_) {
//        Vehicle &veh = kv.second;
//#if DEBUG
//// PRINT << "veh" << veh.id << "(" << veh.route.at(veh.lv_node) << ")"
////       << "nnd=" << veh.nnd << " early=" << veh.early << " active=" <<
////       veh.is_active << std::endl;
//#endif
//#if SPBUG
//        if (veh.id == SP && t_ % 10 == 0)
//            PRINT << "veh" << veh.id << "(" << veh.route.at(veh.lv_node) << ")"
//                  << "nnd=" << veh.nnd << " early=" << veh.early
//                  << " active=" << veh.is_active << std::endl;
//#endif
//        if (t_ > veh.early && veh.is_active) {
//            veh.nnd += opts_.VehicleSpeed;
//            while (veh.nnd >= 0 && veh.is_active) {
//                veh.lv_node++;
//#if SPBUG
//                if (veh.id == SP)
//                    INFO << "veh" << veh.id << " moved from "
//                         << veh.route.at(veh.lv_node - 1) << " to "
//                         << veh.route.at(veh.lv_node) << std::endl;
//#endif
//                if (veh.lv_node >= veh.route.size()) {
//                    ERROR << "veh " << veh.id << " " << veh.route.size() << " "
//                          << veh.lv_node << std::endl;
//                    exit(0);
//                }
//                // @James originally is if
//                while (veh.is_active &&
//                       veh.route.at(veh.lv_node) ==
//                           veh.sched.at(veh.lv_stop + 1).node_id) {
//                    veh.lv_stop++;
//                    Stop &stop = veh.sched.at(veh.lv_stop);
//                    StopType type = stop.type;
//#if SPBUG
//                    if (veh.id == SP) {
//                        ERROR << "node: " << veh.sched.at(veh.lv_stop).node_id
//                              << "[" << int(type) << "]" << std::endl;
//                    }
//#endif
//                    // std::cout << "size: " << veh.sched.size() << std::endl;
//                    // if (type == StopType::CUSTOMER_ORIGIN)
//                    //     std::cout << "ORIGIN" << std::endl;
//                    if (type == StopType::VEHICLE_DEST) {
//                        veh.is_active = false;
//                        count_active_--;
//                        stop.visit_time = t_;
//#if !SPBUG
//                        SUCCESS << "veh " << veh.id << "\t[finish]"
//                                << std::endl;
//#endif
//                    } else if (type == StopType::CUSTOMER_ORIGIN) {
//                        veh.load++;
//                        stop.visit_time = t_;
//#if !SPBUG
//                        SUCCESS << "req " << veh.sched.at(veh.lv_stop).trip_id
//                                << "\t[pickup]" << std::endl;
//#endif
//                    } else {
//                        veh.load--;
//                        stop.visit_time = t_;
//#if !SPBUG
//                        SUCCESS << "req " << veh.sched.at(veh.lv_stop).trip_id
//                                << "\t[dropoff]" << std::endl;
//#endif
//                    }
//                }
//                veh.nnd -= edges_.at(veh.route.at(veh.lv_node - 1))
//                               .at(veh.route.at(veh.lv_node));
//            }
//            // update in intervals
//            if ((t_ - veh.early) % opts_.GPSTiming == 0) {
//                da_.UpdateLocation(veh.id, veh.lv_node, veh.nnd);
//                solution_->UpdateVehicle(veh, t_);
//            }
//        } else if (t_ == veh.early) {
//#if !SPBUG
//            SUCCESS << "veh " << veh.id << "\t[start]" << std::endl;
//#endif
//            solution_->UpdateVehicle(veh, t_);
//            // SUCCESS << "veh " << veh.id << "\t<<<start>>>" << std::endl;
//        }
//    }
}

//bool Cargo::Run() {
//    // Blocks until
//    // (1) minimum duration reached (t_ > tmin_) and
//    // (2) no more active vehicles (count_active_ == 0)
//    while (!(t_ > tmin_ && count_active_ == 0)) {
//        // Start the clock for this interval
//        auto t_start = std::chrono::high_resolution_clock::now();
//        // #if DEBUG
//        if (t_ % 50 == 0)
//            PRINT << "tmin=" << tmin_ << "/t=" << t_ << "/tmax=" << tmax_
//                  << "/alive=" << count_active_ << std::endl;
//        // #endif
//
//        // Move all the vehicles
//        if (t_ >= 0)
//            MoveVehicles();
//        // Broadcast new trips as customers or vehicles
//        if (pi_.trips.find(t_) != pi_.trips.end()) {
//            for (auto trip : pi_.trips.at(t_)) {
//                if (trip.demand < 0) {
//                    count_active_++;
//                    // TODO: broadcast a new vehicle
//                    solution_->VehicleOnline(trip);
//                } else {
//                    // TODO: customer_online() embark
//                    solution_->RequestOnline(trip);
//                    broadcast_time_[trip.id] =
//                        std::chrono::high_resolution_clock::now();
//                }
//            }
//        }
//
//        // If the elapsed time is too long, the simulator's run thread is too
//        // slow to fit inside real-time. Reset the Scale option to be 1. If the
//        // thread is still too slow, increase the scale; but then, the
//        // simulation won't be real-time anymore; it will be slowed down.
//        auto t_end = std::chrono::high_resolution_clock::now();
//        int elapsed = std::round(
//            std::chrono::duration<double, std::milli>(t_end - t_start).count());
//#if DEBUG
//        WARN << elapsed << " ms" << std::endl;
//#endif
//        if (elapsed > sleep_) {
//            ERROR << "Scale too big, exiting\n";
//            return true;
//        }
//
//        // Sleep until the next time interval
//        std::this_thread::sleep_for(
//            std::chrono::milliseconds(sleep_ - elapsed));
//
//        // Advance the simulation time
//        t_++;
//    }
//    //    mq_close(mq_);
//    return false;
//}

// it's just like inform the vehicle who is the customer
//bool Cargo::RequestMatched(const Trip &customer, const VehicleId &vid,
//                               const Schedule &schedule, const Route &route) {
//    // TODO: validate the route
//    Route &old_route = vehicles_[vid].route;
//    int end = vehicles_[vid].lv_node;
//    // validity check !important
//    for (int i = 0; i <= end; ++i) {
//        // if the old route doesn't exist in the new route or the oid is in the
//        // old route
//        if (route[i] != old_route[i] || customer.oid == old_route[i]) {
//            ERROR << "req " << customer.id << " to veh " << vid << " REFUSED"
//                  << std::endl;
//            total_refuse_++;
//            return false;
//        }
//    }
//
//    // Get the runtime on the request
//    auto t_end = std::chrono::high_resolution_clock::now();
//    int elpased = std::round(std::chrono::duration<double, std::milli>(
//                                 t_end - broadcast_time_[customer.id])
//                                 .count());
//#if SPBUG
//    if (vid == SP) {
//        ERROR << "request " << customer.id << " matched to veh " << vid
//              << " in " << elpased << " ms" << std::endl;
//        for (auto &item : route)
//            INFO << item << " ";
//        INFO << std::endl;
//        for (auto &item : schedule)
//            INFO << item.node_id << " ";
//        INFO << std::endl;
//    }
//#endif
//#if !SPBUG
//    ERROR << "request " << customer.id << " matched to veh " << vid << " in "
//          << elpased << " ms" << std::endl;
//#endif
//    total_time_ += elpased;
//    total_match_++;
//
//    try {
//        vehicles_[vid].sched.clear();
//        vehicles_[vid].route.clear();
//        std::copy(schedule.begin(), schedule.end(),
//                  std::back_inserter(vehicles_[vid].sched));
//        std::copy(route.begin(), route.end(),
//                  std::back_inserter(vehicles_[vid].route));
//        vehicles_[vid].load++;
//        return true;
//    } catch (std::exception &e) {
//        ERROR << "IIIIIIIIIIIIII'm here" << std::endl;
//        return true;
//    }
//}

} // namespace cargo

