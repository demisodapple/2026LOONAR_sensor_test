#pragma once
#include <math.h>
#include "mag_calibration_params.h"

namespace magcal {
inline bool apply(float x, float y, float z, float out[4]) {
  if (!kEnabled || !isfinite(x) || !isfinite(y) || !isfinite(z)) return false;
  const float shifted[3] = {x - kOffsetUt[0], y - kOffsetUt[1],
                            z - kOffsetUt[2]};
  for (int axis = 0; axis < 3; ++axis) {
    out[axis] = 0.0f;
    for (int input = 0; input < 3; ++input)
      out[axis] += kMatrix[axis][input] * shifted[input];
    if (!isfinite(out[axis])) return false;
  }
  out[3] = sqrtf(out[0] * out[0] + out[1] * out[1] + out[2] * out[2]);
  return isfinite(out[3]);
}
}  // namespace magcal
