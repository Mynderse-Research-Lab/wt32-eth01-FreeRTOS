#include "W5500BusMutex.h"

namespace w5500 {

std::mutex& spiBusMutex() {
  static std::mutex m;
  return m;
}

}  // namespace w5500
