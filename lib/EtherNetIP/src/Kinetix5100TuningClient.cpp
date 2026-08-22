/**
 * @file Kinetix5100TuningClient.cpp
 * @brief Implementation of Kinetix5100TuningClient.
 */

#include "Kinetix5100TuningClient.h"
#include "CipMessageRouter.h"

#ifdef ESP_PLATFORM
#include "esp_log.h"
#else
#include <cstdio>
#define ESP_LOGE(tag, fmt, ...) std::fprintf(stderr, "E [%s] " fmt "\n", tag, ##__VA_ARGS__)
#endif

namespace k5100 {

Kinetix5100TuningClient::Kinetix5100TuningClient(eip::EipSession& session) : session_(session) {}

bool Kinetix5100TuningClient::readParam16(uint16_t param_id, uint16_t& out_value) {
    eip::Bytes req = buildGetParameterRequest(param_id);
    eip::Bytes resp;
    if (!session_.sendExplicit(req, resp)) {
        return false;
    }
    uint32_t val32 = 0;
    if (!parseGetParameterResponse(resp, val32, 2)) {
        return false;
    }
    out_value = static_cast<uint16_t>(val32);
    return true;
}

bool Kinetix5100TuningClient::readParam32(uint16_t param_id, uint32_t& out_value) {
    eip::Bytes req = buildGetParameterRequest(param_id);
    eip::Bytes resp;
    if (!session_.sendExplicit(req, resp)) {
        return false;
    }
    return parseGetParameterResponse(resp, out_value, 4);
}

bool Kinetix5100TuningClient::writeParam16(uint16_t param_id, uint16_t value) {
    eip::Bytes req = buildSetParameterRequest(param_id, value, 2);
    eip::Bytes resp;
    if (!session_.sendExplicit(req, resp)) {
        return false;
    }
    eip::MessageRouterResponse mr_resp;
    if (!eip::parseMessageRouterResponse(resp, mr_resp) || !mr_resp.isSuccess()) {
        return false;
    }
    return true;
}

bool Kinetix5100TuningClient::readTuningSnapshot(TuningSnapshot& out_snapshot) {
    bool ok = true;
    ok &= readParam16(kParamGainAdjustMode, out_snapshot.gain_adjust_mode);
    ok &= readParam16(kParamSysGainResponseLevel, out_snapshot.sys_gain_response_level);
    ok &= readParam16(kParamLoadInertiaRatio, out_snapshot.load_inertia_ratio);
    ok &= readParam16(kParamPositionPropGain, out_snapshot.position_prop_gain);
    ok &= readParam16(kParamVelocityPropGain, out_snapshot.velocity_prop_gain);
    ok &= readParam16(kParamVelocityIntGain, out_snapshot.velocity_int_gain);
    ok &= readParam32(kParamTotalInertia, out_snapshot.total_inertia);
    return ok;
}

bool Kinetix5100TuningClient::setGainAdjustMode(uint16_t mode) {
    return writeParam16(kParamGainAdjustMode, mode);
}

bool Kinetix5100TuningClient::setSysGainResponseLevel(uint16_t level) {
    return writeParam16(kParamSysGainResponseLevel, level);
}

} // namespace k5100
