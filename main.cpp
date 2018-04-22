#include <thread>
#include <chrono>
#include <iostream>
#include "Sim.h"

int main()
{
    Sim sim("./chengdu.gtree", "./dataset_500+1000_0", "./chengdu.rnet", 1, 12, 1);
    sim.Init();
    std::thread t([&sim](){
        std::this_thread::sleep_for(std::chrono::milliseconds(120000));
        sim.Stop();
    });
    sim.Start();
    t.join();
}