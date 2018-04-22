#include <iostream>
#include "Controller.h"

Controller::Controller(Sim &sim, Solution &solution) : sim_(sim), solution_(solution)
{
}

Controller::RequestOnline(const trip_t &request)
{
	solution.RequestOnline(request);
	t_starts_[request.id] = std::chrono::high_resolution_clock::now();
}

Controller::VehicleOnline(const trip_t &vehicle)
{
	solution.VehicleOnline(vehicle);
}

Controller::UpdateRoute(unsigned int veh_id, const std::vector<unsigned int> &route)
{
	sim.UpdateRoute(veh_id, route);
}

Controller::SetMatch(unsigned int req_id, unsigned int veh_id)
{
	// call the logger
	auto t_end = std::chrono::high_resolution_clock::now();
	int elpased = std::round(std::chrono::duration<double, std::milli>(t_end - t_starts_[req_id]).count());
	std::cout << "request " << req_id << " matched in " << elpased << " ms" << std::endl;
}	
