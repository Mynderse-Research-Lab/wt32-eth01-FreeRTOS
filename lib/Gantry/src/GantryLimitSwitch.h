#ifndef GANTRY_LIMIT_SWITCH_H
#define GANTRY_LIMIT_SWITCH_H

#include <stdint.h>

namespace Gantry {

class GantryLimitSwitch {
public:
    GantryLimitSwitch();

    void configure(int pin, bool activeLow = true, bool enablePullup = true,
                   uint8_t debounceCycles = 6);
    bool begin();
    void update(bool force = false);

    bool isConfigured() const;
    bool isActive() const;
    int getPin() const;

private:
    int pin_;
    bool activeLow_;
    bool enablePullup_;
    uint8_t debounceCycles_;

    bool sampleState_;
    bool stableState_;
    uint8_t stableCount_;
    bool initialized_;
};

} // namespace Gantry

#endif // GANTRY_LIMIT_SWITCH_H
