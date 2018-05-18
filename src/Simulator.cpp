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
// using message queue
#include <mqueue.h>
#include <errno.h>
#include <string.h>

#include "libcargo/Simulator.h"
#include "libcargo/types.h"
#include "libcargo/options.h"
#include "libcargo/file.h"
#include "libcargo/message.h"
#include "gtree/gtree.h"
// since there's only a forward delaration in the header
#include "libcargo/Solution.h"

namespace cargo
{

#define DEBUG false
#define SPBUG false
#define SP 1

using file::ReadEdges;
using file::ReadNodes;
using file::ReadProblemInstance;
using msg::MessageType;
using opts::Options;

// --------------------------------------------------------
// Simulator
// --------------------------------------------------------

Simulator::Simulator()
    : status_(SimulatorStatus::RUNNING), t_(0), tmin_(0), tmax_(0),
      count_active_(0)
{
    // Better way to set these?
    PRINT.type = MessageType::DEFAULT;
    INFO.type = MessageType::INFO;
    WARN.type = MessageType::WARNING;
    ERROR.type = MessageType::ERROR;
    SUCCESS.type = MessageType::SUCCESS;
}

void Simulator::SetOptions(Options opts)
{
    opts_ = opts;
}

void Simulator::SetSolution(Solution *solution)
{
    solution_ = solution;
}

void Simulator::Initialize()
{
    // @James any better method to check?
    if (solution_ == nullptr)
    {
        ERROR << "No solution injected" << std::endl;
        exit(-1);
    }

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
    for (const auto &kv : pi_.trips)
    {
        for (const auto &trip : kv.second)
        {
            // Set tmin_
            if (trip.early > tmin_)
                tmin_ = trip.early;
            if (trip.demand < 1)
            {
                vehicles_.insert(std::make_pair(trip.id, Vehicle(trip)));
                Vehicle &veh = vehicles_.at(trip.id);
                gtree_.find_path(veh.oid, veh.did, veh.route);
#if SPBUG
                if (veh.id == SP)
                {
                    for (auto &item : veh.route)
                        INFO << item << " ";
                    INFO << std::endl;
                }
#endif
                veh.nnd = (-1) * edges_.at(veh.oid).at(veh.route.at(1));
                // no need to set, already in constructor
                // veh.lv_node = 0;
                // >Hu modified for the change of Stop
                Stop o = {veh.id, veh.oid, StopType::VEHICLE_ORIGIN, trip.early, trip.early};
                Stop d = {veh.id, veh.did, StopType::VEHICLE_DEST, -1, trip.late};
                veh.sched.push_back(o);
                veh.sched.push_back(d);
                // Set tmax_
                if (veh.late > tmax_)
                    tmax_ = veh.late;
            }
        }
    }
    // Set the sleep time based on the time scale option.
    sleep_ = std::round((float)1000 / opts_.Scale);

    INFO << "done Simulator::Initialize()\n";
}

void Simulator::MoveVehicles()
{
    for (auto &kv : vehicles_)
    {
        Vehicle &veh = kv.second;
#if DEBUG
// PRINT << "veh" << veh.id << "(" << veh.route.at(veh.lv_node) << ")"
//       << "nnd=" << veh.nnd << " early=" << veh.early << " active=" << veh.is_active << std::endl;
#endif
#if SPBUG
        if (veh.id == SP && t_ % 10 == 0)
            PRINT << "veh" << veh.id << "(" << veh.route.at(veh.lv_node) << ")"
                  << "nnd=" << veh.nnd << " early=" << veh.early << " active=" << veh.is_active << std::endl;
#endif
        if (t_ > veh.early && veh.is_active)
        {
            veh.nnd += opts_.VehicleSpeed;
            while (veh.nnd >= 0 && veh.is_active)
            {
                veh.lv_node++;
#if SPBUG
                if (veh.id == SP)
                    INFO << "veh" << veh.id << " moved from " << veh.route.at(veh.lv_node - 1)
                         << " to " << veh.route.at(veh.lv_node) << std::endl;
#endif
                if (veh.lv_node >= veh.route.size())
                {
                    ERROR << "veh " << veh.id << " " << veh.route.size() << " " << veh.lv_node << std::endl;
                    exit(0);
                }
                // @James originally is if
                while (veh.is_active && veh.route.at(veh.lv_node) == veh.sched.at(veh.lv_stop + 1).node_id)
                {
                    veh.lv_stop++;
                    Stop &stop = veh.sched.at(veh.lv_stop);
                    StopType type = stop.type;
#if SPBUG
                    if (veh.id == SP)
                    {
                        ERROR << "node: " << veh.sched.at(veh.lv_stop).node_id << "[" << int(type) << "]" << std::endl;
                    }
#endif
                    // std::cout << "size: " << veh.sched.size() << std::endl;
                    // if (type == StopType::CUSTOMER_ORIGIN)
                    //     std::cout << "ORIGIN" << std::endl;
                    if (type == StopType::VEHICLE_DEST)
                    {
                        veh.is_active = false;
                        count_active_--;
                        stop.visit_time = t_;
#if !SPBUG
                        SUCCESS << "veh " << veh.id << "\t[finish]" << std::endl;
#endif
                    }
                    else if (type == StopType::CUSTOMER_ORIGIN)
                    {
                        veh.load++;
                        stop.visit_time = t_;
#if !SPBUG
                        SUCCESS << "req " << veh.sched.at(veh.lv_stop).trip_id << "\t[pickup]" << std::endl;
#endif
                    }
                    else
                    {
                        veh.load--;
                        stop.visit_time = t_;
#if !SPBUG
                        SUCCESS << "req " << veh.sched.at(veh.lv_stop).trip_id << "\t[dropoff]" << std::endl;
#endif
                    }
                }
                veh.nnd -= edges_.at(veh.route.at(veh.lv_node - 1)).at(veh.route.at(veh.lv_node));
            }
            // update in intervals
            if ((t_ - veh.early) % opts_.GPSTiming == 0)
            {
                solution_->UpdateVehicle(veh, t_);
            }
        }
        else if (t_ == veh.early)
        {
#if !SPBUG
            SUCCESS << "veh " << veh.id << "\t[start]" << std::endl;
#endif
            solution_->UpdateVehicle(veh, t_);
            // SUCCESS << "veh " << veh.id << "\t<<<start>>>" << std::endl;
        }
    }
}

bool Simulator::Run()
{
    // Blocks until
    // (1) minimum duration reached (t_ > tmin_) and
    // (2) no more active vehicles (count_active_ == 0)
    while (!(t_ > tmin_ && count_active_ == 0))
    {
        // Start the clock for this interval
        auto t_start = std::chrono::high_resolution_clock::now();
        // #if DEBUG
        if (t_ % 50 == 0)
            PRINT << "tmin=" << tmin_ << "/t=" << t_ << "/tmax=" << tmax_ << "/alive=" << count_active_ << std::endl;
        // #endif

        // Move all the vehicles
        if (t_ > 0)
            MoveVehicles();
        // Broadcast new trips as customers or vehicles
        if (pi_.trips.find(t_) != pi_.trips.end())
        {
            for (auto trip : pi_.trips.at(t_))
            {
                if (trip.demand < 0)
                {
                    count_active_++;
                    // TODO: broadcast a new vehicle
                    solution_->VehicleOnline(trip);
                }
                else
                {
                    // TODO: customer_online() embark
                    solution_->RequestOnline(trip);
                    broadcast_time_[trip.id] = std::chrono::high_resolution_clock::now();
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
        if (elapsed > sleep_)
        {
            ERROR << "Scale too big, exiting\n";
            return true;
        }

        // Sleep until the next time interval
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ - elapsed));

        // Advance the simulation time
        t_++;
    }
//    mq_close(mq_);
    return false;
}

// it's just like inform the vehicle who is the customer
bool Simulator::RequestMatched(const Trip &customer, const VehicleId &vid, const Schedule &schedule, const Route &route)
{
    // TODO: validate the route
    Route &old_route = vehicles_[vid].route;
    int end = vehicles_[vid].lv_node;
    // validity check !important
    for (int i = 0; i <= end; ++i)
    {
        // if the old route doesn't exist in the new route or the oid is in the old route
        if (route[i] != old_route[i] || customer.oid == old_route[i])
        {
            ERROR << "req " << customer.id << " to veh " << vid << " REFUSED" << std::endl;
            total_refuse_++;
            return false;
        }
    }

    // Get the runtime on the request
    auto t_end = std::chrono::high_resolution_clock::now();
    int elpased = std::round(std::chrono::duration<double, std::milli>(t_end - broadcast_time_[customer.id]).count());
#if SPBUG
    if (vid == SP)
    {
        ERROR << "request " << customer.id << " matched to veh " << vid << " in " << elpased << " ms" << std::endl;
        for (auto &item : route)
            INFO << item << " ";
        INFO << std::endl;
        for (auto &item : schedule)
            INFO << item.node_id << " ";
        INFO << std::endl;
    }
#endif
#if !SPBUG
    ERROR << "request " << customer.id << " matched to veh " << vid << " in " << elpased << " ms" << std::endl;
#endif
    total_time_ += elpased;
    total_match_++;

    try
    {
        vehicles_[vid].sched.clear();
        vehicles_[vid].route.clear();
        std::copy(schedule.begin(), schedule.end(), std::back_inserter(vehicles_[vid].sched));
        std::copy(route.begin(), route.end(), std::back_inserter(vehicles_[vid].route));
        return true;
    }
    catch (std::exception &e)
    {
        ERROR << "IIIIIIIIIIIIII'm here" << std::endl;
        return true;
    }
}

} // namespace cargo
