/* Firmware application orchestration. */
#ifndef APP_H
#define APP_H

#include <stdbool.h>
#include <stdint.h>

void app_init(void);
void app_poll(void);   /* one non-blocking main-loop iteration */

#endif
