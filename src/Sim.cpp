#include <fstream>
#include <thread>
#include <chrono>
#include <iostream>
#include <ctime>
#include <iomanip>
#include <algorithm>
#include "Sim.h"

Sim::Sim(std::string gtree_path, std::string trips_path, std::string roads_path,
         unsigned int scale, unsigned int speed, unsigned int update_timing)
{
    gtree_path_ = gtree_path;
    trips_path_ = trips_path;
    roads_path_ = roads_path;
    stop_ = false;
    scale_ = scale;
    speed_ = speed;
    update_timing_ = update_timing;
}

void Sim::Init()
{
    // initialize gtree
    GTree::load(gtree_path_);
    gtree_ = GTree::get();

    // initialize nodes and edges
    std::ifstream ifs(roads_path_);
    unsigned int node_id, o, d;
    double ox, oy, dx, dy;
    while (ifs >> node_id >> o >> d >> ox >> oy >> dx >> dy)
    {
        node_t no{ox, oy};
        node_t nd{dx, dy};
        nodes_[o] = no;
        nodes_[d] = nd;
        edges_[node_id] = edge_t{o, d, haversine(no, nd)};
    }
    ifs.close();

    // initialize trips
    ifs.open(trips_path_);
    std::string v, c, line;
    int count = 3, demand;
    unsigned int trip_id, from, to, early, late;
    ifs >> instance_name_ >> v >> vehicle_count_ >> c >> customer_count_;
    std::cout << "Instance: " << instance_name_ << std::endl;
    std::cout << "Vehicles: " << vehicle_count_ << std::endl;
    std::cout << "Customers: " << customer_count_ << std::endl;
    while (count-- && std::getline(ifs, line))
        std::cout << line << std::endl;
    while (ifs >> trip_id >> from >> to >> demand >> early >> late)
    {
        trips_[trip_id] = trip_t{trip_id, from, to, early, late, demand, false};
        // auto iter = trips_idx_.find(early);
        // if (iter == trips_idx_.end())
        // trips_idx_[early] = std::vector<unsigned int>();
        trips_idx_[early].push_back(trip_id);
    }
    ifs.close();

    // for (auto iter = trips_.begin(); iter != trips_.end(); ++iter)
    // std::cout << iter->first << " " << iter->second.id << std::endl;
}

void Sim::Start()
{
    unsigned int global_time = 0;
    unsigned int sleep = std::round((float)1000 / scale_ * update_timing_);
    while (!stop_)
    {
        std::cout << "GLOBAL TIME: " << global_time << std::endl;

        // std::clock_t c_start = std::clock();
        auto t_start = std::chrono::high_resolution_clock::now();

        // get all trips according to time
        std::vector<unsigned int> &trips = trips_idx_[global_time];

        // I don't know why the empty methods is valid for those values that are not set in the map
        // UPDATE: http://en.cppreference.com/w/cpp/container/map/operator_at
        if (trips.empty())
            std::cout << "empty" << std::endl;
        else
        {
            // std::cout << "trips at time " << global_time << ": " << trips.size() << std::endl;
            for (auto iter = trips.begin(); iter != trips.end(); ++iter)
            {
                trip_t &trip = trips_[(*iter)];
                if (trip.demand < 0)
                {
                    // TODO: vehicle_online() embark

                    // set a default route of vehicle
                    // TODO: maybe add from-to validation check
                    std::vector<int> route; // the vector in gtree is int
                    int distance = gtree_.find_path(trip.from, trip.to, route);
                    // for (auto it = route.begin(); it != route.end(); ++it)
                    //     std::cout << *it << " -> ";
                    // std::cout << "Distance: " << distance << std::endl;
                    // copy from vector<int> to vector<unsigned int>
                    std::copy(route.begin(), route.end(), std::back_inserter(routes_[trip.id]));

                    // set the initial location
                    current_[trip.id] = (unsigned int)route[0];
                    residual_[trip.id] = 0;
                }
                else
                {
                    // TODO: customer_online() embark
                }
            }
        }

        // update vehicle status
        for (const auto& kv : current_)
        {
            const unsigned int veh_id = kv.first;
            // check if finished
            if (trips_[veh_id].finish)
                continue;
            unsigned int location = kv.second;
            int residual = residual_[veh_id] + speed_;
            int distance;
            std::vector<unsigned int> &route = routes_[veh_id];
            auto pos = std::find(route.begin(), route.end(), location);
            if (pos == route.end())
            {
                std::cerr << "Invalid modification on route" << std::endl;
                exit(0);
            }
            auto next = pos + 1;
            while (next != route.end() && residual > 0)
            {
                distance = gtree_.search(*pos, *next);
                residual -= distance;
                pos = next;
                ++next;
            }
            if (next == route.end() && residual >= 0)
            {
                // vehicle finish route
                std::cout << "vehicle " << veh_id << " finish route" << std::endl;
            }
            if (current_[veh_id] != *pos)
                std::cout << "vehicle " << veh_id << " move to node " << *pos << std::endl;
            current_[veh_id] = *pos;
            residual_[veh_id] = residual;
        }

        // std::clock_t c_end = std::clock();
        auto t_end = std::chrono::high_resolution_clock::now();
        // std::cout << std::fixed << std::setprecision(2) << "CPU time used: "
        //           << 1000.0 * (c_end - c_start) / CLOCKS_PER_SEC << " ms\n"
        //           << "Wall clock time passed: "
        //           << std::chrono::duration<double, std::milli>(t_end - t_start).count()
        //           << " ms" << std::endl;
        unsigned int elapsed = std::round(std::chrono::duration<double, std::milli>(t_end - t_start).count());
        // protect from too big scale
        if (elapsed > sleep)
        {
            std::cerr << "Scale too big, exit" << std::endl;
            exit(0);
        }
        std::cout << "sleep, elapsed: " << sleep - elapsed << " " << elapsed << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep - elapsed));

        ++global_time;
    }
}

void Sim::Stop()
{
    stop_ = true;
}

void Sim::UpdateRoute(unsigned int veh_id, const std::vector<unsigned int> &route)
{
    routes_.erase(veh_id);
    std::copy(route.begin(), route.end(), std::back_inserter(routes_[veh_id]));
}

int Sim::Size()
{
    return trips_.size();
}
