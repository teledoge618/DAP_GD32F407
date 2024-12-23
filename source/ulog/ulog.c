#include "ulog.h"

#include "printf.h"

#if (ULOG_INDEX_ENABLE == 1)
static uint32_t _ulog_index = 0;
#endif

/* 前缀 */
static const char *ulog_level_type[] = {
    "[     ] ",  // (never used)
    "[ERROR] ",  //
    "[WARN ] ",  //
    "[INFO ] ",  //
    "[DEBUG] ",  //
    "[     ] ",  // (never used)
};

/* LOG等级对应颜色 */
#if (ULOG_COLOR_ENABLE == 1)
static uint8_t ulog_level_color[] = {
    shell_color_white,   // (never used)
    shell_color_red,     // ERROR
    shell_color_yellow,  // WARN
    shell_color_white,   // INFO
    shell_color_green,   // DEBUG
    shell_color_white    // (never used)
};
#endif

void ulog_printf(uint8_t level, char const *const format, ...) {
#if (ULOG_COLOR_ENABLE == 1)
    printf("\033[%dm", ulog_level_color[level]);
#endif

#if (ULOG_INDEX_ENABLE == 1)
    printf("%d %s", _ulog_index++, ulog_level_type[level]);
#else
    printf("%s", ulog_level_type[level]);
#endif

    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);

    // printf("\r\n");

#if (ULOG_COLOR_ENABLE == 1)
    printf("\033[0m");
#endif
}

void ulog_clean(void) { printf("\033[2J"); }
