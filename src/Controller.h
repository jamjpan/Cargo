#include <unordered_map>
#include <vector>
#include <chrono>
#include "common.h"

class Controller
{
public:
	Controller(Sim&, Solution&);
	void RequestOnline(const trip_t&);
	void VehicleOnline(const trip_t&);
	void UpdateRoute(unsigned int, const std::vector<unsigned int>&);
	void SetMatch(unsigned int, unsigned int);
	int AverageMatchTime();
private:
	Sim &sim_;
	Solution &solution_;
	std::unordered_map<unsigned int, std::chrono::time_point<std::chrono::high_resolution_clock> > t_starts_;
	int total_time_;
	int total_match_;
}
