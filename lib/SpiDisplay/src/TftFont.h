#ifndef TFT_FONT_H
#define TFT_FONT_H

#include <cstdint>

namespace display {

constexpr uint8_t FONT_WIDTH = 8;
constexpr uint8_t FONT_HEIGHT = 16;
constexpr char FONT_FIRST_CHAR = 32;  // Space
constexpr char FONT_LAST_CHAR = 126;  // Tilde

// Standard 8x16 monospace font bitmap table for ASCII 32..126
extern const uint8_t font8x16[95][16];

} // namespace display

#endif // TFT_FONT_H
