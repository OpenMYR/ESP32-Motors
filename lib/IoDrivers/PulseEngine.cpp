#include "PulseEngine.h"

#if MYR_PULSE_ENGINE_TYPE == MYR_PULSE_ENGINE_TYPE_LEDC
#include "PulseEngineLedc.cpp"
#else
#include "PulseEngineGptimer.cpp"
#endif
