#ifndef GANTRY_LIMIT_SWITCH_H
#define GANTRY_LIMIT_SWITCH_H

#include <stdint.h>

namespace Gantry {

class GantryLimitSwitch {
public:
    enum class Source {
        kGpio,          // Read from MCP23S17 or direct GPIO (step/dir path)
        kDriveManaged   // State set externally by EIP axis adapter via drive feedback
    };

    GantryLimitSwitch();

    // GPIO-backed configuration (existing path).
    void configure(int pin, bool activeLow = true, bool enablePullup = true,
                   uint8_t debounceCycles = 6);

    // Drive-managed: firmware does not read a GPIO; the axis adapter calls
    // setExternalActive() each cycle from the EIP assembly feedback.
    void configureExternal();

    bool begin();
    void update(bool force = false);

    bool isConfigured() const;
    bool isActive() const;
    int getPin() const;

    Source source() const { return source_; }

    // Set the limit state from an external source (EIP assembly feedback).
    // Only meaningful when source_ == kDriveManaged.
    void setExternalActive(bool active);

private:
    Source source_ = Source::kGpio;

    int pin_;
    bool activeLow_;
    bool enablePullup_;
    uint8_t debounceCycles_;

    bool sampleState_;
    bool stableState_;
    uint8_t stableCount_;
    bool initialized_;

    // External state when source_ == kDriveManaged.
    bool externalActive_ = false;
};

} // namespace Gantry

#endif // GANTRY_LIMIT_SWITCH_H
