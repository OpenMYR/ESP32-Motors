#ifndef MYR_UDP_PROMPT_SCHEDULER_H
#define MYR_UDP_PROMPT_SCHEDULER_H

#include <stdint.h>

namespace UdpPromptScheduler {
constexpr int64_t kPromptWarmupUs = 30LL * 1000 * 1000;
constexpr int64_t kWarmupIntervalUs = 1LL * 1000 * 1000;
constexpr int64_t kWarmupJitterUs = 200LL * 1000;
constexpr int64_t kSteadyIntervalUs = 10LL * 1000 * 1000;
constexpr int64_t kSteadyJitterUs = 2LL * 1000 * 1000;
constexpr int64_t kMinPromptIntervalUs = 250LL * 1000;

int64_t computeNextIntervalUs(int64_t elapsed_us, int64_t jitter_us);
}

#endif // MYR_UDP_PROMPT_SCHEDULER_H
