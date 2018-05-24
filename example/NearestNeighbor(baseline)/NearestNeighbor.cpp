#include <iostream>
#include <vector>
#include <map>
#include <limits>
#include <algorithm>
#include <thread>
#include <mutex>
#include <cmath>
// #include "common.h"
// #include "gtree/gtree.h"
#include "NearestNeighbor.h"
// #include "libcargo/Solution.h"
// #include "libcargo.h"

// Input: set of customers R
// Output: set of assignments A

typedef std::vector<int> RequestBatch;
typedef std::vector<int> Assignments;

std::mutex g_mutex_veh;

auto selector = [](std::pair<VehicleId, Vehicle> pair) { return pair.second.route[pair.second.lv_node]; };
auto selector2 = [](std::pair<VehicleId, Vehicle> pair) { return pair.first; };

NearestNeighbor::NearestNeighbor(Simulator *sim) : Solution(sim)
{
    // temporarily hard-coded
    GTree::load("../../data/roadnetworks/cd1.gtree");
    gtree_ = GTree::get();
}

void NearestNeighbor::VehicleOnline(const Trip &vehicle)
{
    // routes_[vehicle.id] = std::vector<NodeId> {vehicle.oid, vehicle.did};
    // Trip replay;
    // while (replay_queue_.try_dequeue(replay))
    //     request_queue_.enqueue(replay);
}

void NearestNeighbor::RequestOnline(const Trip &request)
{
    request_queue_.enqueue(request);
    // Trip replay;
    // while (replay_queue_.try_dequeue(replay))
    //     request_queue_.enqueue(replay);
}

void NearestNeighbor::UpdateVehicle(const Vehicle &vehicle, const SimTime time)
{
    // this function should be considered as "maintenance"
    // just get a copy of vehicle
    // make_pair makes a copy
    // replay
    // Trip replay;
    // while (replay_queue_.try_dequeue(replay))
    // request_queue_.enqueue(replay);

    std::lock_guard<std::mutex> guard(g_mutex_veh);
    vehicles_[vehicle.id] = vehicle;
    // auto iter = vehicles_.find(vehicle.id);
    // if (iter == vehicles_.end())
    //     vehicles_.insert(std::make_pair(vehicle.id, vehicle));
    // else
    // {
    // vehicles_.at(vehicle.id) = vehicle;
    // }
    updates_[vehicle.id] = time;
    // std::cout << "veh " << vehicle.id << " received" << std::endl;
}

// void NearestNeighbor::Receive()
// {
//     mq_unlink("/update_vehicle");
//     mqd_t mq = mq_open("/update_vehicle", O_CREAT | O_RDWR, 0655, mq_attr{0, 1000000, 1024, 0});
//     SimTime st;
//     Vehicle v;
//     while (!done_)
//     {
//         int err = mq_receive(mq, (char *)(&st), 1024, NULL);
//         if (err == -1)
//         {
//             std::cout << strerror(errno) << std::endl;
//             done_ = true;
//         }
//         mq_receive(mq, (char *)(&v), 1024, NULL);
//         std::cout << "veq " << v.id << " received" << std::endl;
//     }
// }

// Run function, should contain a loop
// If you want to batch requests, you are on your own
void NearestNeighbor::Run()
{

    std::cout << "gtree(1,2)=" << gtree_.search(1,2) << "\n";
    std::cout << "gtree(2,5)=" << gtree_.search(2,5) << "\n";


    std::cout << "running" << std::endl;
    while (!done_)
    {
        Trip request;
        bool succeed = request_queue_.try_dequeue(request);
        if (succeed)
        {
            const int increment = 10;
            // lock the map for retrieve
            g_mutex_veh.lock();

            const int n = vehicles_.size();
            std::vector<int> locations(n);
            std::vector<int> indexes(n);
            std::transform(vehicles_.begin(), vehicles_.end(), locations.begin(), selector);
            std::transform(vehicles_.begin(), vehicles_.end(), indexes.begin(), selector2);

            g_mutex_veh.unlock();

            int base = 0;
            int k = 0;
            bool matched = false;
            while (!matched && base < n)
            {
                k = base + increment;
                // std::cout << "n: " << n << " k: " << k << std::endl;
                if (k > n)
                    k = n;
                // return value is the ordered indexes of the k locations
                std::vector<int> candidates = gtree_.KNN(request.oid, k, locations);
                int i = base;
                while (!matched && i < k)
                {
                    float distance = 0.0f;
                    int vid;
                    try
                    {
                        vid = indexes[candidates[i]];
                        // std::cerr << "----------" << vid << std::endl;
                        Vehicle &veh = vehicles_[vid];
                        // std::cerr << "----------" << vid << std::endl;
                        Route new_route;
                        Schedule new_sched;
                        int lv_stop = veh.lv_stop;
                        int lv_node = veh.lv_node;
                        // v1: originally its lv_stop+1, but that's not correct, due to the distance
                        // from lv_stop to the first in schedule is quite large
                        // v2: back to lv_stop + 1, and consider the next node in the route as the start
                        std::copy(veh.sched.begin() + lv_stop + 1, veh.sched.end(), std::back_inserter(new_sched));
                        if (InsertSchedule(request, veh, new_sched, distance, new_route))
                        {
                            matched = true;
                            // concatenate schedule
                            int insert_pos = 0;
                            for (auto iter = veh.sched.begin(); iter != veh.sched.begin() + lv_stop + 1; ++iter)
                            {
                                new_sched.insert(new_sched.begin() + insert_pos, *iter);
                                insert_pos++;
                            }
                            // concatenate route
                            for (int i = lv_node; i >= 0; --i)
                                new_route.insert(new_route.begin(), veh.route[i]);
                            // temporarily just make copy to local map
                            vehicles_[veh.id].sched = new_sched;
                            vehicles_[veh.id].route = new_route;

                            if (!sim_->RequestMatched(request, veh.id, new_sched, new_route))
                                matched = false;
                        }
                    }
                    catch (std::exception &e)
                    {
                        std::cout << "VID: " << vid << std::endl;
                        sim_->Terminate();
                        Terminate();
                    }
                    i++;
                }
                base += increment;
            }
            if (!matched)
                request_queue_.enqueue(request);
        }
        // std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    // t_receive.join();
    std::cout << "done" << std::endl;
}

bool NearestNeighbor::InsertSchedule(const Trip &request, const Vehicle &vehicle, Schedule &schedule, float &distance, Route &new_route)
{

    // TODO: rethink, the schedule should start from the next node in the route rather than last visited stop

    // +1 get the route from schedule[lv_stop+1]
    // +2 get a copy of schedule[lv_stop+1:end] as test_sch
    // step 1,2 should be done in Run(), schedule here is test_sch
    // +3 insert request into test_sch[begin:end]
    // +4 check the time window
    // +5 if valid, compute the shortest path from begin to end, set as new_route

    // insert with O(n^2)
    // auto o_iter = schedule.begin();
    // not 0 because the first stop is the last visited stop, so any assigned pickup can't be ahead of it
    int o_iter = 0;
    int size = schedule.size();
    int min_distance = std::numeric_limits<int>::max();
    Schedule min_schedule;
    bool found = false;
    // load is negative for not-full vehicle
    int load = vehicle.load;

    // v1: schedule[0] is the last visited node
    // v2: correct_time is the time when the vehicle will visit the next node in the route (round up)
    SimTime correct_time = updates_[vehicle.id] - std::ceil((float)vehicle.nnd / speed_);
    NodeId schedule_start = vehicle.route[vehicle.lv_node + 1];
    // while (o_iter != schedule.end())
    // not size+1 because the last stop is vehicle destination
    while (o_iter != size)
    {
        // insert pickup stop of the request
        schedule.insert(schedule.begin() + o_iter, Stop{request.id, request.oid, StopType::CUSTOMER_ORIGIN, -1, request.early});
        auto d_iter = o_iter + 1;
        // while (d_iter != schedule.end())
        // not size+2 because the last stop is vehicle destination
        while (d_iter != size + 1)
        {
            // insert dropoff stop of the request
            schedule.insert(schedule.begin() + d_iter, Stop{request.id, request.did, StopType::CUSTOMER_DEST, -1, request.late});
            // correct the time to the last visited stop, using '-' because nnd is negative
            SimTime time = correct_time;
            // check the time window & capacity validity
            bool not_begin = false;
            bool valid = true;
            int total_distance = 0;
            int dis;
            for (int k = 0; k < size + 2; ++k)
            // for (auto iter = schedule.begin(); iter != schedule.end(); ++iter)
            {
                auto iter = schedule.begin() + k;
                if (not_begin)
                {
                    dis = gtree_.search((iter - 1)->node_id, iter->node_id);
   //                 std::cout << "gtree(" << (iter-1)->node_id << ", " << iter->node_id << ") = " << dis << "\n";
                }
                else
                {
                    dis = gtree_.search(schedule_start, iter->node_id);
  //                  std::cout << "gtree(" << schedule_start << ", " << iter->node_id << ") = " << dis << "\n";
                }
                total_distance += dis;
 //               if (vehicle.id == 1)
//                    std::cout << "distance: " << total_distance << std::endl;
                time += (int)(dis / speed_);
                /*
                if (vehicle.id == 1)
                    std::cout << time << std::endl;
                    */
                not_begin = true;

                if (iter->type == StopType::CUSTOMER_ORIGIN || iter->type == StopType::VEHICLE_ORIGIN)
                {
                    // early time window
                    // the case will never came up
                    if (time < iter->time_limit)
                        time = iter->time_limit;
                    // if is customer origin, capacity--
                    if (iter->type == StopType::CUSTOMER_ORIGIN)
                        load++;
                    // check capacity validity
                    if (load > 0)
                    {
                        valid = false;
                        break;
                    }
                }
                else
                {
                    // late time window
                    if (time > iter->time_limit)
                    {
                        valid = false;
                        /// std::cout << veh "time exceed" << std::endl;
                        break;
                    }
                    if (iter->type == StopType::CUSTOMER_DEST)
                        load--;
                }
            }
            if (schedule[size + 1].type != StopType::VEHICLE_DEST)
                valid = false;
            if (valid)
            {
                // take into consideration
                if (total_distance < min_distance)
                {
                    min_schedule = schedule;
                    min_distance = total_distance;
                    found = true;
                }
            }
            schedule.erase(schedule.begin() + d_iter);
            d_iter++;
        }
        schedule.erase(schedule.begin() + o_iter);
        o_iter++;
    }
    if (found)
    {
        schedule = min_schedule;
        distance = min_distance;
        // new route. but there may be some problems with the boundray of old route and new route
        gtree_.find_path(schedule_start, schedule[0].node_id, new_route);
        Route temp_route;
        for (int i = 1; i < size + 2; ++i)
        {
            gtree_.find_path(schedule[i-1].node_id, schedule[i].node_id, temp_route);
            std::copy(temp_route.begin()+1, temp_route.end(), std::back_inserter(new_route));
        }
    }
    return found;
}

/*
Assignments NearestNeighbor(RequestBatch R)
{
    Assignments A;
    const int increment = 10;
    for (int j = 0; j < R.size(); ++j)
    {
        int base = 0;
        int k = 0;
        bool matched = false;
        Request r = R.at(j);
        while (!matched && base <= n)
        {
            k = base + increment;
            std::vector<int> candidates = gtree.KNN(r.origin, k, vehicles);
            int i = base;
            while (!matched && i < k)
            {
                Schedule T;
                Schedule S = candidates[i].schedule;
                double c = schedule_insert_add(S, r, T);
                if (c != -1)
                {
                    A[j] = i;
                    matched = true;
                }
                else
                    ++i;
            }
            if (!matched)
                base += increment;
        }
    }
    return A;
}
*/

int main()
{
    Options op;
    op.RoadNetworkPath = "../../data/roadnetworks/cd1.rnet";
    op.GTreePath = "../../data/roadnetworks/cd1.gtree";
    op.EdgeFilePath = "../../data/roadnetworks/cd1.edges";
    op.ProblemInstancePath = "../../data/benchmarks/cd1/rs-md-10.instance";
    //op.ProblemInstancePath = "../../data/dataset_5000+500_0";
    op.Scale = 2;
    op.VehicleSpeed = 10;
    op.GPSTiming = 1;

    Simulator *sim = new Simulator();
    // move here because solution need options in simulator
    sim->SetOptions(op);
    Solution *solution = new NearestNeighbor(sim);
    sim->SetSolution(solution);
    sim->Initialize();

    std::thread t([&sim, &solution]() {
        sim->Run();
        solution->Terminate();
    });
    solution->Run();
    t.join();
    // solution->Terminate();

    std::cout << "Total match:" << sim->TotalMatch() << std::endl;
    std::cout << "Total time:" << sim->TotalTime() << std::endl;
    std::cout << "Average: " << (sim->TotalMatch() == 0 ? 0 : sim->TotalTime() / sim->TotalMatch()) << std::endl;
    //std::cout << "Average: " << sim->TotalTime() / sim->TotalMatch() << std::endl;
    std::cout << "Refused rate: " << float(sim->TotalRefuse()) / (sim->TotalMatch() + sim->TotalRefuse()) * 100 << std::endl;
}
