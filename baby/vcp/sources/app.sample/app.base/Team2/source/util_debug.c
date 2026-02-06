// log_util_ctx.c
#include "util_debug.h"
#include <debug.h>
#include <app_cfg.h>
#include <stdarg.h>
#include <stdio.h>

static inline uint32 get_tick_ms(void)
{
    uint32 tick = 0;
    (void)SAL_GetTickCount(&tick);
    return tick * portTICK_PERIOD_MS;
}

int Log_EveryMsCtx(log_every_ctx_t* ctx, uint32_t period_ms)
{
    if (!ctx) return 0;

    uint32 now = get_tick_ms();
    if (period_ms == 0) return 1;

    if (ctx->last_ms == 0) {
        ctx->last_ms = now;
        return 1;
    }

    if ((uint32)(now - ctx->last_ms) >= period_ms) {
        ctx->last_ms = now;
        return 1;
    }
    return 0;
}

void Log_Print(log_every_ctx_t* ctx, uint32_t period_ms, const char* fmt, ...)
{
    if (!Log_EveryMsCtx(ctx, period_ms)) return;

    va_list ap;
    va_start(ap, fmt);
    char buf[256];
    (void)vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    mcu_printf("%s", buf);
}
