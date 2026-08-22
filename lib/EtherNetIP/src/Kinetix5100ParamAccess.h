/**
 * @file Kinetix5100ParamAccess.h
 * @brief Kinetix 5100 CIP Parameter Access definitions and functions.
 */

#ifndef KINETIX_5100_PARAM_ACCESS_H
#define KINETIX_5100_PARAM_ACCESS_H

#include <cstdint>
#include "EipByteBuffer.h"

namespace k5100 {

// CIP Class 0x0F (Parameter Object)
constexpr uint16_t kCipClassParameter = 0x0F;
constexpr uint16_t kCipAttributeValue = 1;

// Kinetix 5100 Parameter IDs (from EDS)
constexpr uint16_t kParamGainAdjustMode = 217;
constexpr uint16_t kParamSysGainResponseLevel = 216;
constexpr uint16_t kParamLoadInertiaRatio = 144;
constexpr uint16_t kParamPositionPropGain = 185;
constexpr uint16_t kParamVelocityPropGain = 189;
constexpr uint16_t kParamVelocityIntGain = 191;
constexpr uint16_t kParamTotalInertia = 659;
constexpr uint16_t kParamNotchFilter1Freq = 208;
constexpr uint16_t kParamNotchFilter1Depth = 209;
constexpr uint16_t kParamResonanceSuppLpfTime = 210;
constexpr uint16_t kParamVelocityFbLpfTime = 232;

// Gain Adjustment Mode values
constexpr uint16_t kGainModeManual = 0;
constexpr uint16_t kGainModeMode1 = 1;
constexpr uint16_t kGainModeMode2 = 2;
constexpr uint16_t kGainModeReset = 4;

// Build a CIP GetAttributeSingle request for Class 0x0F parameter
eip::Bytes buildGetParameterRequest(uint16_t param_id);

// Build a CIP SetAttributeSingle request for Class 0x0F parameter
// size_bytes: 2 for UINT (most params) or 4 for UDINT (e.g., TotalInertia, Overshoot)
eip::Bytes buildSetParameterRequest(uint16_t param_id, uint32_t value, uint8_t size_bytes);

// Parse a GetAttributeSingle response. Returns false on CIP error or parse failure.
bool parseGetParameterResponse(const eip::Bytes& mr_response_data, uint32_t& out_value, uint8_t size_bytes);

struct TuningSnapshot {
    uint16_t gain_adjust_mode = 0;
    uint16_t sys_gain_response_level = 0;
    uint16_t load_inertia_ratio = 0;
    uint16_t position_prop_gain = 0;
    uint16_t velocity_prop_gain = 0;
    uint16_t velocity_int_gain = 0;
    uint32_t total_inertia = 0;
};

} // namespace k5100

#endif // KINETIX_5100_PARAM_ACCESS_H
