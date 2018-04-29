#include <thread>
#include <chrono>
#include <iostream>
#include "Sim.h"

int main()
{
    Options op;
    op.RoadNetworkPath = "../data/roadnetworks/cd1.rnet";
    op.GTreePath = "../data/roadnetworks/cd1.gtree";
    op.EdgeFilePath = "../data/roadnetworks/cd1.edges";
    // op.ProblemInstancePath = "../data/benchmarks/cd1/cd1-SR-n10m5-0";
    op.ProblemInstancePath = "../data/dataset_500+1000_0";
    op.Scale = 10;
    op.VehicleSpeed = 10;

    Simulator sim;
    Solution &nn_solution = new NearestNeighbor(sim);

    sim.SetOptions(op);
    sim.SetSolution(nn_solution);
    sim.Initialize();

    // run the simulator in another thread
    std::thread t([&sim]() {
        sim.Run();
    });

    t.join();
}