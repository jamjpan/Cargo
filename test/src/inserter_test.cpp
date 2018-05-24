#include "catch.hpp"

#include "libcargo.h"

using namespace cargo;

using  msg::Message;
using  msg::MessageType;

TEST_CASE("Inserter works", "[inserter]") {
    Message m(MessageType::DEFAULT);
    m << "Inserter test" << std::endl;
}
