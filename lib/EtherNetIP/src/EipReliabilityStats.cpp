#include "EipReliabilityStats.h"

namespace eip {

ReliabilityStats& reliabilityStats() {
  static ReliabilityStats s;
  return s;
}

}  // namespace eip
