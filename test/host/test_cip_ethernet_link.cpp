// Host tests for the CIP Ethernet Link object (class 0xF6) helpers used by the
// `ethlink` console command.

#include "CipEthernetLink.h"
#include "unity.h"

#include <cstring>

using eip::Bytes;

void setUp() {}
void tearDown() {}

void test_get_request_bytes() {
    const Bytes req = eip::buildEthLinkGetRequest(1, eip::EthLinkAttr::kMediaCounters);
    // GetAttributeSingle, 3-word path: class 0xF6, instance 1, attribute 5.
    const uint8_t expect[] = {0x0E, 0x03, 0x20, 0xF6, 0x24, 0x01, 0x30, 0x05};
    TEST_ASSERT_EQUAL_UINT32(sizeof(expect), static_cast<uint32_t>(req.size()));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expect, req.data(), sizeof(expect));
}

void test_get_and_clear_request_bytes() {
    const Bytes req = eip::buildEthLinkGetAndClearRequest(2, eip::EthLinkAttr::kInterfaceCounters);
    const uint8_t expect[] = {0x4C, 0x03, 0x20, 0xF6, 0x24, 0x02, 0x30, 0x04};
    TEST_ASSERT_EQUAL_UINT32(sizeof(expect), static_cast<uint32_t>(req.size()));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expect, req.data(), sizeof(expect));
}

void test_flags_negotiated_full_duplex() {
    // link up, full duplex, negotiation status 3.
    const eip::EthLinkFlags f = eip::decodeEthLinkFlags(0x0Fu);
    TEST_ASSERT_TRUE(f.link_up);
    TEST_ASSERT_TRUE(f.full_duplex);
    TEST_ASSERT_EQUAL_UINT8(3, f.negotiation);
    TEST_ASSERT_FALSE(f.hardware_fault);
}

void test_flags_negotiation_failed_half_duplex() {
    // link up, half duplex, status 1 (failed -> 10M half default), hw fault.
    const eip::EthLinkFlags f = eip::decodeEthLinkFlags(0x45u);
    TEST_ASSERT_TRUE(f.link_up);
    TEST_ASSERT_FALSE(f.full_duplex);
    TEST_ASSERT_EQUAL_UINT8(1, f.negotiation);
    TEST_ASSERT_TRUE(f.hardware_fault);
    TEST_ASSERT_TRUE(std::strstr(eip::ethNegotiationName(f.negotiation), "FAILED") != nullptr);
}

void test_parse_flags_and_speed() {
    eip::EthLinkFlags f;
    TEST_ASSERT_TRUE(eip::parseEthLinkFlags(Bytes{0x0F, 0x00, 0x00, 0x00}, f));
    TEST_ASSERT_EQUAL_UINT8(3, f.negotiation);
    uint32_t speed = 0;
    TEST_ASSERT_TRUE(eip::parseEthUdint(Bytes{100, 0, 0, 0}, speed));
    TEST_ASSERT_EQUAL_UINT32(100, speed);
    TEST_ASSERT_FALSE(eip::parseEthUdint(Bytes{100, 0}, speed));
}

void test_parse_media_counters() {
    Bytes data;
    for (uint32_t i = 0; i < eip::kEthMediaCounterCount; ++i) {
        const uint32_t v = i * 1000u + 7u;
        data.push_back(static_cast<uint8_t>(v & 0xFF));
        data.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
        data.push_back(static_cast<uint8_t>((v >> 16) & 0xFF));
        data.push_back(static_cast<uint8_t>((v >> 24) & 0xFF));
    }
    uint32_t out[eip::kEthMediaCounterCount] = {};
    TEST_ASSERT_TRUE(eip::parseEthCounters(data, out, eip::kEthMediaCounterCount));
    TEST_ASSERT_EQUAL_UINT32(1007, out[1]);  // fcs_errors
    TEST_ASSERT_EQUAL_UINT32(6007, out[6]);  // late_collisions
    TEST_ASSERT_EQUAL_STRING("fcs_errors", eip::ethMediaCounterName(1));
    TEST_ASSERT_EQUAL_STRING("late_collisions", eip::ethMediaCounterName(6));

    data.pop_back();
    TEST_ASSERT_FALSE(eip::parseEthCounters(data, out, eip::kEthMediaCounterCount));
}

void test_parse_interface_control() {
    eip::EthInterfaceControl c;
    TEST_ASSERT_TRUE(eip::parseEthInterfaceControl(Bytes{0x01, 0x00, 0x00, 0x00}, c));
    TEST_ASSERT_EQUAL_UINT16(1, c.control_bits);
    TEST_ASSERT_EQUAL_UINT16(0, c.forced_speed);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_get_request_bytes);
    RUN_TEST(test_get_and_clear_request_bytes);
    RUN_TEST(test_flags_negotiated_full_duplex);
    RUN_TEST(test_flags_negotiation_failed_half_duplex);
    RUN_TEST(test_parse_flags_and_speed);
    RUN_TEST(test_parse_media_counters);
    RUN_TEST(test_parse_interface_control);
    return UNITY_END();
}
