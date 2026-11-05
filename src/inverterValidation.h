// SPDX-License-Identifier: LGPL-2.1
#pragma once

#include <arm_math.h>
#include <math.h>

namespace inverter_validation {
// PI implementations use reciprocals; reject values that cannot represent them.
inline bool positive(float32_t value) {
    return isfinite(value) && value > 0.0F && isfinite(1.0F / value);
}

inline bool timing(float32_t frequency, float32_t period) {
    const float32_t step = frequency * period;
    return positive(frequency) && positive(period) && positive(step) && step < PI;
}
}
