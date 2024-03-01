//LCD indication task
//Communication UART1 19200 8bit 1stop npar
//Format: \n\r terminated string "\n[OPT1]=%ld\t[OPT2]=%ld\t...\n\r"
//OPT
//IOUT=%ld   Iout mA
//VOUT=%ld   Vout mV
//REFSET=%ld Iset mA
//DACSET=%ld Dacset x100uV
//STAT=%ld   ||__|__|__|__|SoftStart|PS_ON|Operate|Ready||Board|Line|OverVolt|GndMon|OverHeat|Water|EXT2|EXT1||DCCT|__|__|Punit3|PUnit2|PUnit1|FBLost|OverCurr||
//THS=%d
//TSYS=%d
//VDC=%d x100mV
//IDC=%d x100mA
//IP=%s ddd.ddd.ddd.ddd
//MAC=%s xxxxxxxxxxxx
#include <stdint.h>
#include <stdio.h>
#include <hal/assert.h>
#include <hal/io.h>
#include "fsl_common.h"
//#include "fsl_debug_console.h"
#include "fsl_iomuxc.h"
#include "fsl_adapter_uart.h"
#include "device/board.h"
#include "Indicate.h"
#include <MIMX8MN6_cm7.h>

#define WAIT_FLAG  while(MPS->Flag.f5Hz==0); MPS->Flag.f5Hz=0
char text_buffer[128];
char recv_buffer[128];
uint8_t RSim;
uint8_t txbuff[]   = "Uart polling example\r\nBoard will send back received characters\r\n";

status_t status;
hal_uart_config_t config_UART;
hal_uart_handle_t hUART;

static void DataTransfer(PS_Control* MPS, uint32_t len){
    uint32_t idx=0;
    char* substr=NULL;
    int32_t data;
    HAL_UartSendBlocking(hUART, (uint8_t*)text_buffer, len);
    idx=0; 
    do {HAL_UartReceiveBlocking(hUART,(uint8_t*)&RSim,1);recv_buffer[idx++]=RSim;}while(RSim!='\r');
    recv_buffer[idx]=0;
    if(strstr(recv_buffer,"FBMODE=CC")){
        MPS->Flag.fCCMode=1;
        MPS->Flag.fLocale = 1;
    }
    if(strstr(recv_buffer,"FBMODE=CV")){
        MPS->Flag.fCCMode=0;
        MPS->Flag.fLocale = 1;
    }
    substr = strstr(recv_buffer,"REF_SET=");
    if(substr){
        //hal_log_info("%s",recv_buffer);
        //hal_log_info("%s",substr);
        data = atol(substr+8);
        if(data>ISETMAX)
            data = ISETMAX;
        if(data<0) 
            data = 0;
        MPS->Ref_Set = data;
        MPS->Flag.fLocale = 1;
        //hal_log_info("%ld",MPS->Ref_Set );
        }
    substr = strstr(recv_buffer,"DAC_SET=");
    if(substr){
        data = atol(substr+8);
        if(data>VSETMAX*10L)
            data = VSETMAX*10L;
        if(data<0) 
            data = 0;
        MPS->VRef_Set = data;
        MPS->Flag.fLocale = 1;
        //hal_log_info("%s",recv_buffer);
        //hal_log_info("%ld",MPS->VRef_Set );
        }
    if(strstr(recv_buffer,"MAINPOWER=1")){
        if(MPS->Flag.PS_ON) MPS->Operate = 1;
        else MPS->Flag.PS_ON = 1;
        MPS->Flag.fLocale = 1;
  
    }
    if(strstr(recv_buffer,"MAINPOWER=0")){
        if(MPS->Operate) MPS->Operate = 0;
        else MPS->Flag.PS_ON = 0;  
        MPS->Flag.fLocale = 1;
    }  
}

static void indication_task(void *param) {
    PS_Control *MPS = (PS_Control*)param;
    clock_root_control_t clock_root = kCLOCK_RootUart3;
    clock_ip_name_t clock = kCLOCK_Uart3;
    uint32_t uartClkSrcFreqHz = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / CLOCK_GetRootPreDivider(clock_root)
        / CLOCK_GetRootPostDivider(clock_root) / 10;
    CLOCK_EnableClock(clock);
    config_UART.srcClock_Hz  = uartClkSrcFreqHz;
    config_UART.baudRate_Bps = 115200U;
    config_UART.parityMode   = kHAL_UartParityDisabled;
    config_UART.stopBitCount = kHAL_UartOneStopBit;
    config_UART.enableRx     = 1U;
    config_UART.enableTx     = 1U;
    config_UART.enableRxRTS  = 0U;
    config_UART.enableTxCTS  = 0U;
    config_UART.instance     = 3U;
    hal_log_info("UART1 Init...");
    if (kStatus_Success != HAL_UartInit(hUART, &config_UART)) {
        hal_log_warn("!UART1 Init FAIL... Task suspend!");
        vTaskSuspend(NULL);
    }
    HAL_UartSendBlocking(hUART, txbuff, sizeof(txbuff) - 1);
    hal_log_info("UART1 Init Successfuly");
    for(;;){
        uint32_t text_len;
        WAIT_FLAG;
        DataTransfer(MPS,snprintf(text_buffer,128,"\nIOUT=%ld\tVOUT=%ld\tTHS=%d\n\r",MPS->mIout.val,MPS->mVout.val/10,(int16_t)MPS->tHeatsink));
        WAIT_FLAG;
        DataTransfer(MPS,snprintf(text_buffer,128,"\nIOUT=%ld\tVOUT=%ld\tVSET=%ld\n\r",MPS->mIout.val,MPS->mVout.val/10,MPS->Vreg));
        WAIT_FLAG;
        if(MPS->Flag.fCCMode)
        DataTransfer(MPS,snprintf(text_buffer,128,"\nIOUT=%ld\tVOUT=%ld\tREFSET=%d\n\r",MPS->mIout.val,MPS->mVout.val/10,MPS->Ref_Set));
        else
        DataTransfer(MPS,snprintf(text_buffer,128,"\nIOUT=%ld\tVOUT=%ld\tDACSET=%d\n\r",MPS->mIout.val,MPS->mVout.val/10,MPS->VRef_Set));
        WAIT_FLAG;
        uint32_t STAT = (((uint32_t)MPS->Flag.fLocale)<<20)+(((uint32_t)MPS->Flag.PS_ON)<<18)+(MPS->Operate<<17)+(MPS->Ready<<16)+\
        (((uint32_t)MPS->Faults.Board)<<15)+(((uint32_t)MPS->Faults.Line)<<14)+(((uint32_t)MPS->Faults.Overvoltage)<<13)+(((uint32_t)MPS->Faults.GndMon)<<12)+\
        (((uint32_t)MPS->Faults.Overheat)<<11)+(((uint32_t)MPS->Faults.ExtLock2)<<9)+(((uint32_t)MPS->Faults.ExtLock1)<<8)+\
        (((uint32_t)MPS->Faults.DCCT)<<7)+(((uint32_t)MPS->Faults.Unit2)<<3)+(((uint32_t)MPS->Faults.Unit1)<<2)+\
        (((uint32_t)MPS->Faults.FBLost)<<1)+(((uint32_t)MPS->Faults.Overcurrent)<<0); 
        DataTransfer(MPS,snprintf(text_buffer,128,"\nIOUT=%ld\tVOUT=%ld\tSTAT=%ld\n\r",MPS->mIout.val,MPS->mVout.val/10,STAT));
    }

    // This task must never end.
    hal_unreachable();
}
void indication_run(PS_Control *MPS) {
    hal_assert(xTaskCreate(indication_task, "Indication_task", TASK_STACK_SIZE, (void *)MPS, INDICATION_TASK_PRIORITY, NULL) == pdPASS);
}

