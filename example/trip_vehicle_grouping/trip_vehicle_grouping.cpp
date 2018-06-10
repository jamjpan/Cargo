#include <mutex>
#include <thread>
#include <chrono>
#include "PNAS.h"

/*
params: batch time
steps:
1. pairwise request-vehicle RV graph
(a) which requests can be pairwise combined(a virtual vehicle involved)
(b) which vehicles can serve which requests individually(given current passengers)
*/

std::mutex g_mutex_request;
std::mutex g_mutex_veh;

PNAS::PNAS(Simulator *sim, int batch_time) : Solution(sim)
{
    batch_time_ = batch_time;
    request_count_ = 0;
    GTree::load("../../data/roadnetworks/cd1.gtree");
    gtree_ = GTree::get();
}

void PNAS::VehicleOnline(const Trip &vehicle)
{
    Trip replay;
    while (replay_queue_.try_dequeue(replay))
        request_queue_.enqueue(replay);
}

void PNAS::RequestOnline(const Trip &request)
{
    g_mutex_request.lock();
    request_queue_.enqueue(request);
    request_count_++;
    Trip trip;
    while (replay_queue_.try_dequeue(trip))
    {
        request_queue_.enqueue(trip);
        request_count_++;
    }
    g_mutex_request.unlock();
}

void PNAS::UpdateVehicle(const Vehicle &vehicle, const SimTime time)
{
    std::lock_guard<std::mutex> guard(g_mutex_veh);
    vehicles_[vehicle.id] = vehicle;
    updates_[vehicle.id] = time;
}

void PNAS::Run()
{
    while (!done_)
    {
        auto t_start = std::chrono::high_resolution_clock::now();
        // define variables
        std::vector<Trip> requests;

        // get request batch
        g_mutex_request.lock();
        int batch_size = request_count_;
        request_count_ = 0;
        g_mutex_request.unlock();
        Trip temp_trip;
        for (int i = 0; i < batch_size; ++i)
        {
            bool success = request_queue_.try_dequeue(temp_trip);
            if (success)
                requests.push_back(temp_trip);
            else
                break;
        }
        batch_size = requests.size();

        // generate rv-graph
        std::vector<std::pair<CustomerId, CustomerId>> rr_edges;
        std::vector<std::pair<CustomerId, VehicleId>> rv_edges;
        // get valid request pairs
        for (int i = 0; i < batch_size - 1; ++i)
        {
            for (int j = i + 1; j < batch_size; ++j)
            {
                if (RRPairValid(requests[i], requests[j]))
                {
                    rr_edges.push_back(std::make_pair(requests[i].id, requests[j].id));
                    std::cout << "PAIR: " << requests[i].id << "\t" << requests[j].id << std::endl;
                }
            }
        }

        for (Trip trip : requests)
        {
            replay_queue_.enqueue(trip);
        }

        auto t_end = std::chrono::high_resolution_clock::now();
        int elapsed = std::round(std::chrono::duration<double, std::milli>(t_end - t_start).count());
        std::cout << "ELAPSED " << elapsed << "\tBATCH " << batch_size << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(batch_time_ * 1000 - elapsed));
    }
}

bool PNAS::RRPairValid(const Trip &request1, const Trip &request2)
{
    std::vector<Stop> stops;
    stops.push_back(Stop{request1.id, request1.oid, StopType::CUSTOMER_ORIGIN, -1, request1.early});
    stops.push_back(Stop{request1.id, request1.did, StopType::CUSTOMER_DEST, -1, request1.late});

    for (int i = 0; i < 2; ++i)
    {
        int time = request1.early, dis;
        stops.insert(stops.begin() + i, Stop{request2.id, request2.oid, StopType::CUSTOMER_ORIGIN, -1, request2.early});
        for (int j = i + 1; j < 3; ++j)
        {
            bool trip_valid = true;
            stops.insert(stops.begin() + j, Stop{request2.id, request2.did, StopType::CUSTOMER_DEST, -1, request2.late});
            for (int k = 1; k < 4 && trip_valid; ++k)
            {
                dis = gtree_.search(stops[k - 1].node_id, stops[k].node_id);
                time += dis / speed_;
                if (stops[k].type == StopType::CUSTOMER_ORIGIN)
                {
                    if (time < stops[k].time_limit)
                        time = stops[k].time_limit;
                }
                else
                {
                    if (time > stops[k].time_limit)
                        trip_valid = false;
                }
            }
            if (trip_valid)
                return true;
            stops.erase(stops.begin() + j);
        }
        stops.erase(stops.begin() + i);
    }
    return false;
}

bool PNAS::RVPairValid(const Trip &request, const Vehicle &vehicle)
{
    int lv_stop = vehicle.lv_stop;
    int o_iter = lv_stop + 1;
    // make a copy
    Schedule schedule = vehicle.sched;
    int size = schedule.size();
    int min_distance = std::numeric_limits<int>::max();
    Schedule min_schedule;
    SimTime correct_time = schedule[0].visit_time;
    bool found = false;
    // load is negative for not-full vehicle
    int load = vehicle.load;
    // while (o_iter != schedule.end())
    // not size+1 because the last stop is vehicle destination
    while (o_iter != size)
    {
        // insert pickup stop of the request
        schedule.insert(schedule.begin() + o_iter, Stop{request.id, request.oid, StopType::CUSTOMER_ORIGIN, -1, request.early});
        auto d_iter = o_iter + 1;
        // while (d_iter != schedule.end())
        while (d_iter != size + 1)
        {
            // insert dropoff stop of the request
            schedule.insert(schedule.begin() + d_iter, Stop{request.id, request.did, StopType::CUSTOMER_DEST, -1, request.late});
            // correct the time to first node, using '-' because nnd is negative
            SimTime time = correct_time;
            // check the time window & capacity validity
            bool valid = true;
            int total_distance = 0;
            for (int k = lv_stop; k < size + 2; ++k)
            {
                auto iter = schedule.begin() + k;
                int dis = gtree_.search((iter - 1)->node_id, iter->node_id);
                total_distance += dis;
                time += (int)(dis / speed_);
                if (iter->type == StopType::CUSTOMER_ORIGIN || iter->type == StopType::VEHICLE_ORIGIN)
                {
                    // early time window
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
    }
    return found;
    
}

int main()
{
    Options op;
    op.RoadNetworkPath = "../../data/roadnetworks/cd1.rnet";
    op.GTreePath = "../../data/roadnetworks/cd1.gtree";
    op.EdgeFilePath = "../../data/roadnetworks/cd1.edges";
    // op.ProblemInstancePath = "../data/benchmarks/cd1/cd1-SR-n10m5-0";
    op.ProblemInstancePath = "../../data/dataset_50+150_0";
    op.Scale = 5;
    op.VehicleSpeed = 10;
    op.GPSTiming = 5;

    Simulator *sim = new Simulator();
    // move here because solution need options in simulator
    sim->SetOptions(op);
    Solution *solution = new PNAS(sim, 10);
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
    std::cout << "Average: " << sim->TotalTime() / sim->TotalMatch() << std::endl;
    std::cout << "Refused rate: " << float(sim->TotalRefuse()) / (sim->TotalMatch() + sim->TotalRefuse()) * 100 << std::endl;
}