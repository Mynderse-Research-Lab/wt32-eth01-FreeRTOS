/**
 * @file test_hcs01_comws_client.cpp
 * @brief Unit tests for HCS01 COMWS HTTP client.
 */

#include "Hcs01ComwsClient.h"
#include "unity.h"
#include <string>
#include <vector>
#include <cstring>
#include <algorithm>

using namespace hcs01;

class FakeTcpClient : public eip::ITcpClient {
public:
    bool connect(const char* host, uint16_t port) override {
        connected = true;
        return true;
    }
    
    ssize_t send(const uint8_t* data, size_t len) override {
        last_request.assign(reinterpret_cast<const char*>(data), len);
        return len;
    }
    
    ssize_t recv(uint8_t* buf, size_t max_len, uint32_t timeout_ms) override {
        if (response_data.empty()) return 0;
        size_t n = std::min(max_len, response_data.size());
        std::memcpy(buf, response_data.data(), n);
        response_data.erase(response_data.begin(), response_data.begin() + n);
        return n;
    }
    
    void close() override {
        connected = false;
    }
    
    bool isConnected() const override {
        return connected;
    }

    void setResponse(const std::string& str) {
        response_data.assign(str.begin(), str.end());
    }

    std::string last_request;
    std::vector<uint8_t> response_data;
    bool connected = false;
};

void setUp(void) {
}

void tearDown(void) {
}

void test_normalize_idn_adds_suffix(void) {
    char out[64] = {0};
    Hcs01ComwsClient::normalizeIdn("P-0-0162", out, sizeof(out));
    TEST_ASSERT_EQUAL_STRING("P-0-0162.0.0", out);
}

void test_normalize_idn_preserves_existing(void) {
    char out[64] = {0};
    Hcs01ComwsClient::normalizeIdn("P-0-0162.0.0", out, sizeof(out));
    TEST_ASSERT_EQUAL_STRING("P-0-0162.0.0", out);
}

void test_parse_status_word_decimal(void) {
    TEST_ASSERT_EQUAL_UINT16(3, Hcs01ComwsClient::parseStatusWord("3"));
}

void test_parse_status_word_hex(void) {
    TEST_ASSERT_EQUAL_UINT16(3, Hcs01ComwsClient::parseStatusWord("0x0003"));
}

void test_parse_status_word_binary(void) {
    TEST_ASSERT_EQUAL_UINT16(3, Hcs01ComwsClient::parseStatusWord("0b0000000000000011"));
}

void test_getvar_parses_response(void) {
    FakeTcpClient tcp;
    Hcs01ComwsClient client(tcp);
    
    tcp.setResponse("HTTP/1.0 200 OK\r\n\r\nLogin OK");
    client.connect();
    
    tcp.setResponse("HTTP/1.0 200 OK\r\n\r\n0,1,P-0-4010.0.0=1234\r\n");
    
    char val[32] = {0};
    bool ok = client.getVar("P-0-4010.0.0", val, sizeof(val));
    
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_STRING("1234", val);
    TEST_ASSERT_FALSE(tcp.isConnected());
}

void test_protected_idns_are_blocked(void) {
    TEST_ASSERT_TRUE(Hcs01ComwsClient::isProtectedIdn("P-0-4089.0.13"));
    TEST_ASSERT_TRUE(Hcs01ComwsClient::isProtectedIdn("P-0-4089.0.14"));
    TEST_ASSERT_TRUE(Hcs01ComwsClient::isProtectedIdn("P-0-4089.0.15"));
    TEST_ASSERT_TRUE(Hcs01ComwsClient::isProtectedIdn("S-0-1020"));
    TEST_ASSERT_TRUE(Hcs01ComwsClient::isProtectedIdn("S-0-1021.0.0"));
    TEST_ASSERT_TRUE(Hcs01ComwsClient::isProtectedIdn("S-0-1022.0.0"));
    TEST_ASSERT_FALSE(Hcs01ComwsClient::isProtectedIdn("S-0-0092"));
    TEST_ASSERT_FALSE(Hcs01ComwsClient::isProtectedIdn("S-0-0109.0.0"));
    TEST_ASSERT_FALSE(Hcs01ComwsClient::isProtectedIdn("P-0-0109"));

    FakeTcpClient tcp;
    Hcs01ComwsClient client(tcp);
    tcp.setResponse("HTTP/1.0 200 OK\r\n\r\nLogin OK");
    client.connect();
    char reply[64] = {0};
    TEST_ASSERT_FALSE(client.setVar("P-0-4089.0.13", "192.168.1.99"));
    TEST_ASSERT_FALSE(client.setVarElement7("S-0-1020", "192.168.1.99", reply, sizeof(reply)));
}

void test_setvar_element7_succeeds(void) {
    FakeTcpClient tcp;
    Hcs01ComwsClient client(tcp);
    tcp.setResponse("HTTP/1.0 200 OK\r\n\r\nLogin OK");
    client.connect();

    tcp.setResponse("HTTP/1.0 200 OK\r\n\r\nOK\r\n");
    char reply[64] = {0};
    TEST_ASSERT_TRUE(client.setVarElement7("S-0-0092", "322.5", reply, sizeof(reply)));
    TEST_ASSERT_EQUAL_STRING("OK", reply);
    TEST_ASSERT_NOT_NULL(std::strstr(tcp.last_request.c_str(), "var=0,1,S-0-0092.0.0.7&value=322.5"));
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_normalize_idn_adds_suffix);
    RUN_TEST(test_normalize_idn_preserves_existing);
    RUN_TEST(test_parse_status_word_decimal);
    RUN_TEST(test_parse_status_word_hex);
    RUN_TEST(test_parse_status_word_binary);
    RUN_TEST(test_getvar_parses_response);
    RUN_TEST(test_protected_idns_are_blocked);
    RUN_TEST(test_setvar_element7_succeeds);
    return UNITY_END();
}
