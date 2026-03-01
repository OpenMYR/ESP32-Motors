#ifndef MYR_CONFIG_H
#define MYR_CONFIG_H

#if defined(PIO_UNIT_TESTING)
#include "config/DefaultConfig.h"
#elif __has_include("config/LocalConfig.h")
#include "config/LocalConfig.h"
#else
#include "config/DefaultConfig.h"
#endif

#endif // MYR_CONFIG_H
