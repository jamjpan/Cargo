#include "catch.hpp"

#include "libcargo.h"

using namespace cargo;

using opts::Options;
using  msg::Message;
using  msg::MessageType;

TEST_CASE("Messages print nice colors", "[msg]") {
    Message info(MessageType::INFO);
    Message warn(MessageType::WARNING);
    Message err(MessageType::ERROR);
    Message succ(MessageType::SUCCESS);

    info << "this is an info " << "msg " << 42 << std::endl;
    warn << "this is a warn" << "ing " << 4.2 << std::endl;
    err <<  "this is an err" << "or msg " << 4*2 << std::endl;
    succ << "this is success" << std::endl;
}

TEST_CASE("Simulator can be initialized and executed", "[simulator]") {
    Options op;
    op.RoadNetworkPath = "../data/roadnetworks/cd1.rnet";
    op.GTreePath = "../data/roadnetworks/cd1.gtree";
    op.EdgeFilePath = "../data/roadnetworks/cd1.edges";
    op.ProblemInstancePath = "../data/benchmarks/cd1/cd1-SR-n10m5-0";

    Simulator sim;
    sim.SetOptions(op);
    sim.Initialize();
}

