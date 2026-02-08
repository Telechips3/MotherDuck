#ifndef WPS_FIFO_H
#define WPS_FIFO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===== 너 프로젝트에 맞게 타입만 바꾸면 됨 ===== */
typedef struct {
    float x_m;
    float y_m;
} wp_t;

/* ===== FIFO (ring buffer) ===== */
#define WPS_FIFO_CAPACITY  (64u)   // 필요에 맞게 변경 (2의 거듭제곱이면 더 깔끔)

typedef struct {
    wp_t     buf[WPS_FIFO_CAPACITY];
    uint16_t head;   // next write
    uint16_t tail;   // next read
    uint16_t count;  // stored items
} wps_fifo_t;

/* init/reset */
static inline void wps_fifo_init(wps_fifo_t *q) {
    q->head = 0;
    q->tail = 0;
    q->count = 0;
}

static inline void wps_fifo_reset(wps_fifo_t *q) {
    wps_fifo_init(q);
}

/* status */
static inline uint16_t wps_fifo_count(const wps_fifo_t *q) { return q->count; }
static inline uint16_t wps_fifo_capacity(void) { return (uint16_t)WPS_FIFO_CAPACITY; }
static inline int wps_fifo_is_empty(const wps_fifo_t *q) { return (q->count == 0); }
static inline int wps_fifo_is_full (const wps_fifo_t *q) { return (q->count >= WPS_FIFO_CAPACITY); }

/* push/pop/peek
 * return: 0 success, -1 full/empty
 */
int wps_fifo_push(wps_fifo_t *q, const wp_t *item);
int wps_fifo_pop (wps_fifo_t *q, wp_t *out);
int wps_fifo_peek(const wps_fifo_t *q, wp_t *out);

#ifdef __cplusplus
}
#endif

#endif
