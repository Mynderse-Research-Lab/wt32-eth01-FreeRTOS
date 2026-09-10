#ifndef UI_INPUT_H
#define UI_INPUT_H

#include <cstdint>

namespace display {

enum class UiEvent {
    NONE = 0,
    ROTATE_CW,
    ROTATE_CCW,
    PUSH_CLICK,
    BACK_CLICK
};

class UiInput {
public:
    UiInput();

    /**
     * @brief Process raw MCP23S17 Port B register reading.
     * @param port_b_val 8-bit Port B value from MCP23S17
     * @return UiEvent detected (if any)
     */
    UiEvent update(uint8_t port_b_val);

    void reset();

private:
    uint8_t prev_quad_{0};
    int8_t step_accum_{0};

    bool prev_push_down_{false};
    uint8_t push_debounce_{0};

    bool prev_ko_down_{false};
    uint8_t ko_debounce_{0};
};

} // namespace display

#endif // UI_INPUT_H
