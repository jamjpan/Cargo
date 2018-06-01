#include <iostream>
#include "Controller.h"

Controller::Controller(Sim &sim, Solution &solution) : sim_(sim), solution_(solution)
{
	total_time_ = 0;
	total_match_ = 0;
}

void Controller::RequestOnline(const trip_t &request)
{
	solution.RequestOnline(request);
	t_starts_[request.id] = std::chrono::high_resolution_clock::now();
}

void Controller::VehicleOnline(const trip_t &vehicle)
{
	solution.VehicleOnline(vehicle);
}

void Controller::UpdateRoute(unsigned int veh_id, const std::vector<unsigned int> &route)
{
	sim.UpdateRoute(veh_id, route);
}

void Controller::SetMatch(unsigned int req_id, unsigned int veh_id)
{
	// call the logger
	auto t_end = std::chrono::high_resolution_clock::now();
	int elpased = std::round(std::chrono::duration<double, std::milli>(t_end - t_starts_[req_id]).count());
	std::cout << "request " << req_id << " matched in " << elpased << " ms" << std::endl;
	total_time_ += elpased;
	total_match_++;
}	

int Controller::AverageMatchTime()
{
	return std::round((double)total_time_ / total_match_);
}