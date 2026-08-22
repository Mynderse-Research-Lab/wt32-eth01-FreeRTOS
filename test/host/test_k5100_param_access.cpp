/**
 * @file test_k5100_param_access.cpp
 * @brief Host tests for Kinetix 5100 Param Access functions.
 */

#include <unity.h>
#include "Kinetix5100ParamAccess.h"
#include "CipMessageRouter.h"

void setUp(void) {}
void tearDown(void) {}

void test_build_get_parameter_request_epath(void)
{
    eip::Bytes req = k5100::buildGetParameterRequest(k5100::kParamGainAdjustMode);
    
    // Check request framing: service 0x0E (GetAttributeSingle)
    TEST_ASSERT_EQUAL_UINT8(0x0E, req[0]);
    TEST_ASSERT_GREATER_THAN_UINT32(0, req.size());
}

void test_build_set_parameter_request_16bit(void)
{
    eip::Bytes req = k5100::buildSetParameterRequest(k5100::kParamGainAdjustMode, 1, 2);
    TEST_ASSERT_EQUAL_UINT8(0x10, req[0]); // SetAttributeSingle
    // Ends with data: 01 00 (little-endian 16-bit)
    TEST_ASSERT_EQUAL_UINT8(0x01, req[req.size() - 2]);
    TEST_ASSERT_EQUAL_UINT8(0x00, req[req.size() - 1]);
}

void test_build_set_parameter_request_32bit(void)
{
    eip::Bytes req = k5100::buildSetParameterRequest(k5100::kParamTotalInertia, 0x12345678, 4);
    TEST_ASSERT_EQUAL_UINT8(0x10, req[0]);
    TEST_ASSERT_EQUAL_UINT8(0x78, req[req.size() - 4]);
    TEST_ASSERT_EQUAL_UINT8(0x56, req[req.size() - 3]);
    TEST_ASSERT_EQUAL_UINT8(0x34, req[req.size() - 2]);
    TEST_ASSERT_EQUAL_UINT8(0x12, req[req.size() - 1]);
}

void test_parse_get_parameter_response_success_16bit(void)
{
    eip::Bytes mr_response = {0x8E, 0x00, 0x00, 0x00, 0x05, 0x00};
    uint32_t val = 0;
    bool success = k5100::parseGetParameterResponse(mr_response, val, 2);
    TEST_ASSERT_TRUE(success);
    TEST_ASSERT_EQUAL_UINT32(5, val);
}

void test_parse_get_parameter_response_success_32bit(void)
{
    eip::Bytes mr_response = {0x8E, 0x00, 0x00, 0x00, 0x78, 0x56, 0x34, 0x12};
    uint32_t val = 0;
    bool success = k5100::parseGetParameterResponse(mr_response, val, 4);
    TEST_ASSERT_TRUE(success);
    TEST_ASSERT_EQUAL_UINT32(0x12345678, val);
}

void test_parse_get_parameter_response_failure(void)
{
    eip::Bytes mr_response = {0x8E, 0x00, 0x14, 0x00}; // general_status = 0x14
    uint32_t val = 0;
    bool success = k5100::parseGetParameterResponse(mr_response, val, 2);
    TEST_ASSERT_FALSE(success);
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_build_get_parameter_request_epath);
    RUN_TEST(test_build_set_parameter_request_16bit);
    RUN_TEST(test_build_set_parameter_request_32bit);
    RUN_TEST(test_parse_get_parameter_response_success_16bit);
    RUN_TEST(test_parse_get_parameter_response_success_32bit);
    RUN_TEST(test_parse_get_parameter_response_failure);
    return UNITY_END();
}
