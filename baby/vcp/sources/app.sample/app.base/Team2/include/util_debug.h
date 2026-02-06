// log_util_ctx.h
#pragma once
#include <stdint.h>
#include <stdarg.h>

typedef struct {
    uint32_t last_ms;
} log_every_ctx_t;

int Log_EveryMsCtx(log_every_ctx_t* ctx, uint32_t period_ms);
void Log_Print(log_every_ctx_t* ctx, uint32_t period_ms, const char* fmt, ...);
