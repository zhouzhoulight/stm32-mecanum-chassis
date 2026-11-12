/* Firmware entry point for the standalone LD19 F407VE bench board.
 * No vehicle motor/servo/IMU outputs are ever initialised here. */
#include "main.h"

#include "app.h"
#include "board_config.h"
#include "timebase.h"

int main(void)
{
    HAL_Init();
    timebase_dwt_init();
    app_init();
    for (;;) {
        app_poll();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    for (;;) {
        /* Safe stop: no motor/servo outputs exist on this board firmware. */
    }
}
