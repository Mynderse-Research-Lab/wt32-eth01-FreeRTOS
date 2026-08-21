/**
 * @file Kinetix5100TuningClient.h
 * @brief High-level client for Kinetix 5100 autotuning via CIP explicit messaging.
 */

#ifndef KINETIX_5100_TUNING_CLIENT_H
#define KINETIX_5100_TUNING_CLIENT_H

#include "EipSession.h"
#include "Kinetix5100ParamAccess.h"

namespace k5100 {

class Kinetix5100TuningClient {
public:
    explicit Kinetix5100TuningClient(eip::EipSession& session);

    bool readTuningSnapshot(TuningSnapshot& out_snapshot);
    bool setGainAdjustMode(uint16_t mode);
    bool setSysGainResponseLevel(uint16_t level);

private:
    bool readParam16(uint16_t param_id, uint16_t& out_value);
    bool readParam32(uint16_t param_id, uint32_t& out_value);
    bool writeParam16(uint16_t param_id, uint16_t value);

    eip::EipSession& session_;
};

} // namespace k5100

#endif // KINETIX_5100_TUNING_CLIENT_H
