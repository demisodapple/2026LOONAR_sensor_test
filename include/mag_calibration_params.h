#pragma once

// Safe default: raw readings are logged, but no unverified correction is applied.
// tools/fit_magnetometer.py replaces this file after a validated, installed test.
namespace magcal {
inline constexpr bool kEnabled = false;
inline constexpr bool kScienceReady = false;
inline constexpr char kId[] = "UNVERIFIED";
inline constexpr float kOffsetUt[3] = {0.0f, 0.0f, 0.0f};
inline constexpr float kMatrix[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f},
};
}  // namespace magcal
