// One polar-alignment error reading, as any error source delivers it.
#pragma once
#include <cstdint>

namespace paloop {

struct Observation {
  double azArcmin = 0;
  double altArcmin = 0;
  uint32_t receivedMs = 0;  // device clock when the reading arrived
};

}  // namespace paloop
