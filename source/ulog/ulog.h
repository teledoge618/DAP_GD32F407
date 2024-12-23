#ifndef __ULOG_H__
#define __ULOG_H__

#include <inttypes.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "ulog_config.h"

enum {
    shell_color_black = 30U,
    shell_color_red = 31U,
    shell_color_green = 32U,
    shell_color_yellow = 33U,
    shell_color_blue = 34U,
    shell_color_purple = 35U,
    shell_color_cyan = 36U,
    shell_color_white = 37U,
};

void ulog_printf(uint8_t level, char const *const format, ...);

#if (ULOG_LEVEL >= ULOG_LEVEL_ERROR)
#define ULOG_ERROR(...) ulog_printf(ULOG_LEVEL_ERROR, __VA_ARGS__)
#else
#define ULOG_ERROR(...)
#endif

#if (ULOG_LEVEL >= ULOG_LEVEL_WARN)
#define ULOG_WARN(...) ulog_printf(ULOG_LEVEL_WARN, __VA_ARGS__)
#else
#define ULOG_WARN(...)
#endif

#if (ULOG_LEVEL >= ULOG_LEVEL_INFO)
#define ULOG_INFO(...) ulog_printf(ULOG_LEVEL_INFO, __VA_ARGS__)
#else
#define ULOG_INFO(...)
#endif

#if (ULOG_LEVEL >= ULOG_LEVEL_DEBUG)
#define ULOG_DEBUG(...) ulog_printf(ULOG_LEVEL_DEBUG, __VA_ARGS__)
#else
#define ULOG_DEBUG(...)
#endif

#endif  // !__ULOG_H__
