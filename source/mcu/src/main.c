#include <FreeRTOS.h>
#include <task.h>

#include <hal/assert.h>
#include <hal/io.h>

#include "device/board.h"
#include "device/clock_config.h"
#include "device/rsc_table.h"
#include "device/MPS.h"

#include <tasks/stats.h>
#include <tasks/control.h>
#include <tasks/rpmsg.h>
#include <tasks/sync.h>
#include <tasks/Indicate.h>


// The stack of `main` is tiny, so we store our state as globals.
//#ifdef GENERATE_SYNC
#define SYNC_PERIOD_US 100
#define MS_TICK 1000/SYNC_PERIOD_US
SyncGenerator sync;
//#endif
Statistics stats;
Control control;
Rpmsg rpmsg;
PS_Control MPS={0};


int main(void) {
    // M7 has its local cache and enabled by default, need to set smart subsystems (0x28000000 ~ 0x3FFFFFFF) non-cacheable
    // before accessing this address region
    BOARD_InitMemory();

    // Board specific RDC settings
    BOARD_RdcInit();

    BOARD_BootClockRUN();

    // Initialize UART I/O
    BOARD_InitUART1Pin();
    BOARD_InitUART3Pin();
    hal_io_uart_init(1);
    copyResourceTable();

#ifdef MCMGR_USED
    // Initialize MCMGR before calling its API
    (void)MCMGR_Init();
#endif

    hal_print("\n\r\n\r");
    hal_log_info("** Board started **");

    stats_reset(&stats);
    MPS.ms_tick = MS_TICK;
    MPS.Fault_Clear_Count = 100L*MPS.ms_tick;
    MPS.K.Iout = -130653L;
    MPS.Offset.Iout = 1070L;
    MPS.K.Vout = -39390L;
    MPS.Offset.Vout = -2805L;
    MPS.K.Vreg = 12718L;
    MPS.Offset.Vreg = 0;
    MPS.K.tHS1 = 1250L;
    MPS.Offset.tHS1 = 0L;
    MPS.K.tHS2 = 1250L;
    MPS.Offset.tHS2 = 0L;
    MPS.K.tHS3 = 1250L;
    MPS.Offset.tHS3 = 0L;
    MPS.K.Iset = 1000000L;
    MPS.K.Vset = 100000L;
    MPS.Feedback.KP = 20000L;
    MPS.Feedback.KI = 4000L;
    MPS.Feedback.KD = 0L;
    MPS.Feedback.KP2 = 0L;
    MPS.Feedback.KI2 = 0L;

    
//#ifdef GENERATE_SYNC
    sync_generator_init(&sync, SYNC_PERIOD_US, &stats, &MPS);
//#endif
    control_init(&control, &stats, &MPS);
    rpmsg_init(&rpmsg, &control, &stats);

    hal_log_info("Enable statistics report");
    stats_report_run(&stats);

    hal_log_info("Start MPS150 SkifIO control process");
    control_run(&control);

    hal_log_info("Start RPMSG communication");
    rpmsg_run(&rpmsg);

//#ifdef GENERATE_SYNC
    hal_log_info("Start sync generator");
    sync_generator_run(&sync);
//#endif

    hal_log_info("Start Indication");
    indication_run(&MPS);

    vTaskStartScheduler();
    // Must never return.
    hal_panic();
}
