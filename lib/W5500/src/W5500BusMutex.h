// Shared SPI bus mutex for W5500 chip + socket register access.
#ifndef W5500_BUS_MUTEX_H
#define W5500_BUS_MUTEX_H

#include <mutex>

namespace w5500 {

/// Process-wide SPI mutex (chip recover + socket I/O).
std::mutex& spiBusMutex();

}  // namespace w5500

#endif  // W5500_BUS_MUTEX_H
