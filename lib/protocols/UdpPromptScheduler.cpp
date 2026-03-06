#include "UdpPromptScheduler.h"

namespace UdpPromptScheduler {
int64_t computeNextIntervalUs(int64_t elapsed_us, int64_t jitter_us)
{
    const int64_t base_interval_us =
        elapsed_us < kPromptWarmupUs ? kWarmupIntervalUs : kSteadyIntervalUs;
    const int64_t jitter_limit_us =
        elapsed_us < kPromptWarmupUs ? kWarmupJitterUs : kSteadyJitterUs;

    if (jitter_us > jitter_limit_us) jitter_us = jitter_limit_us;
    if (jitter_us < -jitter_limit_us) jitter_us = -jitter_limit_us;

    int64_t interval_us = base_interval_us + jitter_us;
    if (interval_us < kMinPromptIntervalUs) {
        interval_us = kMinPromptIntervalUs;
    }

    return interval_us;
}
}
