#include "PulseEngine.h"

#if MYR_PULSE_ENGINE_TYPE == MYR_PULSE_ENGINE_TYPE_LEDC
#include "PulseEngineLedc.cpp"
#elif MYR_PULSE_ENGINE_TYPE == MYR_PULSE_ENGINE_TYPE_RMT
#include "PulseEngineRMT.cpp"
#else
#include "PulseEngineGptimer.cpp"
#endif
