#include "wps_fifo.h"

static inline uint16_t wrap_next(uint16_t v) {
    v++;
    if (v >= (uint16_t)WPS_FIFO_CAPACITY) v = 0;
    return v;
}

int wps_fifo_push(wps_fifo_t *q, const wp_t *item)
{
    if (!q || !item) return -1;
    if (q->count >= (uint16_t)WPS_FIFO_CAPACITY) return -1; // full

    q->buf[q->head] = *item;
    q->head = wrap_next(q->head);
    q->count++;
    return 0;
}

int wps_fifo_pop(wps_fifo_t *q, wp_t *out)
{
    if (!q || !out) return -1;
    if (q->count == 0) return -1; // empty

    *out = q->buf[q->tail];
    q->tail = wrap_next(q->tail);
    q->count--;
    return 0;
}

int wps_fifo_peek(const wps_fifo_t *q, wp_t *out)
{
    if (!q || !out) return -1;
    if (q->count == 0) return -1;

    *out = q->buf[q->tail];
    return 0;
}
