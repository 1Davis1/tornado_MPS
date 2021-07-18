#include <common/main.h>

#include <stdint.h>
#include <stdbool.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include <ipp.h>

#include <hal/rpmsg.h>
#include <hal/assert.h>
#include <hal/io.h>


#define TASK_STACK_SIZE 256

static void task_rpmsg(void *param) {
    hal_rpmsg_init();

    hal_rpmsg_channel channel;
    hal_assert(HAL_SUCCESS == hal_rpmsg_create_channel(&channel, 0));
#ifdef HAL_PRINT_RPMSG
    hal_io_rpmsg_init(&channel);
#endif

    // Receive message

    uint8_t *buffer = NULL;
    size_t len = 0;
    hal_rpmsg_recv_nocopy(&channel, &buffer, &len, HAL_WAIT_FOREVER);

    IppMsgAppAny app_msg;
    IppLoadStatus st = ipp_msg_app_load(&app_msg, buffer, len);
    hal_assert(IPP_LOAD_OK == st);
    if (IPP_APP_START == app_msg.type) {
        hal_log_info("Start message received");
    } else {
        hal_log_error("Message error: type mismatch: %d", (int)app_msg.type);
        hal_panic();
    }
    hal_rpmsg_free_rx_buffer(&channel, buffer);
    buffer = NULL;
    len = 0;

    // Send message back
    hal_assert(HAL_SUCCESS == hal_rpmsg_alloc_tx_buffer(&channel, &buffer, &len, HAL_WAIT_FOREVER));
    IppMsgMcuAny mcu_msg = {
        .type = IPP_MCU_DEBUG,
        .debug = {
            .message = "Response message",
        },
    };
    ipp_msg_mcu_store(&mcu_msg, buffer);
    hal_assert(HAL_SUCCESS == hal_rpmsg_send_nocopy(&channel, buffer, ipp_msg_mcu_len(&mcu_msg)));

    hal_assert(HAL_SUCCESS == hal_rpmsg_destroy_channel(&channel));
    hal_rpmsg_deinit();
}

int common_main() {
    /* Create tasks. */
    xTaskCreate(
        task_rpmsg, "RPMSG task",
        TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL
    );

    /* Start FreeRTOS scheduler. */
    vTaskStartScheduler();

    /* Should never reach this point. */
    //PANIC_("End of main()");
    return 0;
}
