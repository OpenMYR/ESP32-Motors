#ifndef MYR_CONFIG_H
#define MYR_CONFIG_H

#if __has_include("config/LocalConfig.h")
#include "config/LocalConfig.h"
#else
#include "config/DefaultConfig.h"
#endif

#endif // MYR_CONFIG_H
