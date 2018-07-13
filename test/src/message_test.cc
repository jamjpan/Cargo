#include "catch.hpp"
#include "libcargo.h"
// =============================================================================
// Messages
// =============================================================================
using namespace cargo;

TEST_CASE("Messages print nice colors", "[msg]") {
    Message print;

    print << "Running message test...\n";
    print << "CARGO" << std::endl;
    print(MessageType::Info) << "ARGOC" << std::endl;
    print(MessageType::Warning) << "RGOCA" << std::endl;
    print(MessageType::Error) <<  "GOCAR" << std::endl;
    print(MessageType::Success) << "OCARG" << std::endl;
    print << "Message test complete." << std::endl;
}

