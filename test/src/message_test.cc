#include "catch.hpp"
#include "libcargo.h"
// =============================================================================
// Messages
// =============================================================================
using namespace cargo;

TEST_CASE("Messages print nice colors", "[msg]") {
    Message msg1(MessageType::Default, "msg_test");
    Message info(MessageType::Info, "msg_test");
    Message warn(MessageType::Warning, "msg_test");
    Message err(MessageType::Error, "msg_test");
    Message succ(MessageType::Success, "msg_test");

    msg1 << "Running message test...\n";
    msg1 << "CARGO" << std::endl;
    info << "ARGOC" << std::endl;
    warn << "RGOCA" << std::endl;
    err <<  "GOCAR" << std::endl;
    succ << "OCARG" << std::endl;
    msg1 << "Message test complete." << std::endl;
}

