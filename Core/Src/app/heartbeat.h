#ifndef APP_HEARTBEAT_H
#define APP_HEARTBEAT_H

#include <stdint.h>

typedef enum {
    HEARTBEAT_NORMAL = 0,
    HEARTBEAT_FAULT  = 1,
} heartbeat_mode_t;

void heartbeat_init(heartbeat_mode_t mode);
void heartbeat_set_mode(heartbeat_mode_t mode);
void heartbeat_tick(void);   /* call from super-loop; uses HAL_GetTick internally */

#endif
