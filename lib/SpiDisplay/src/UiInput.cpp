#include "UiInput.h"

namespace display {

// Quadrature transition lookup table
// Index: (prev_quad << 2) | curr_quad
// Values: 0 = no move, +1 = CW, -1 = CCW
static const int8_t QUAD_TABLE[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

UiInput::UiInput() {
    reset();
}

void UiInput::reset() {
    prev_quad_ = 0;
    step_accum_ = 0;
    prev_push_down_ = false;
    push_debounce_ = 0;
    prev_ko_down_ = false;
    ko_debounce_ = 0;
}

UiEvent UiInput::update(uint8_t port_b_val) {
    // Port B layout per REV02 schematic:
    // PB0 = KO button (active LOW)
    // PB1 = Encoder push button (active LOW)
    // PB2 = Encoder B
    // PB3 = Encoder A
    bool ko_down   = ((port_b_val >> 0) & 1) == 0; // PB0, Active low
    bool push_down = ((port_b_val >> 1) & 1) == 0; // PB1, Active low
    uint8_t enc_b  = (port_b_val >> 2) & 1;        // PB2
    uint8_t enc_a  = (port_b_val >> 3) & 1;        // PB3

    // 1. Process encoder quadrature
    uint8_t curr_quad = (enc_a << 1) | enc_b;
    uint8_t idx = ((prev_quad_ << 2) | curr_quad) & 0x0F;
    int8_t diff = QUAD_TABLE[idx];
    prev_quad_ = curr_quad;

    if (diff != 0) {
        step_accum_ += diff;
        // Standard mechanical encoders have 2 or 4 transitions per detent click
        if (step_accum_ >= 4) {
            step_accum_ = 0;
            return UiEvent::ROTATE_CW;
        } else if (step_accum_ <= -4) {
            step_accum_ = 0;
            return UiEvent::ROTATE_CCW;
        }
    }

    // 2. Process push button
    if (push_down) {
        if (push_debounce_ < 3) push_debounce_++;
        if (push_debounce_ == 2 && !prev_push_down_) {
            prev_push_down_ = true;
            return UiEvent::PUSH_CLICK;
        }
    } else {
        push_debounce_ = 0;
        prev_push_down_ = false;
    }

    // 3. Process KO button
    if (ko_down) {
        if (ko_debounce_ < 3) ko_debounce_++;
        if (ko_debounce_ == 2 && !prev_ko_down_) {
            prev_ko_down_ = true;
            return UiEvent::BACK_CLICK;
        }
    } else {
        ko_debounce_ = 0;
        prev_ko_down_ = false;
    }

    return UiEvent::NONE;
}

} // namespace display
