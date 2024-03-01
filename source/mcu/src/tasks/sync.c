#include "sync.h"

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include <fsl_iomuxc.h>

#include <hal/assert.h>
#include <hal/gpt.h>

#include "stats.h"


#define SYNC_10K_MUX IOMUXC_SAI3_TXC_GPIO5_IO00
// #define SYN_10K_MUX IOMUXC_SAI3_TXC_GPT1_COMPARE2
#define SYNC_10K_PIN 5, 0

#define SYNC_1_MUX IOMUXC_SAI3_RXD_GPIO4_IO30
// #define SYN_1_MUX IOMUXC_SAI3_RXD_GPT1_COMPARE1
#define SYNC_1_PIN 4, 30

#define GPT_CHANNEL 1

#define SET_FAULT(X) { self->MPS->Fault_Clear_Count=1000 * self->MPS->ms_tick; self->MPS->Faults.X = 1; }

#define LED_FAULT_MUX IOMUXC_SAI2_TXC_GPIO4_IO25
#define LED_FAULT_PIN 4, 25
#define LED_1_MUX IOMUXC_GPIO1_IO06_GPIO1_IO06
#define LED_1_PIN 1, 6
#define LED_MODE_MUX IOMUXC_GPIO1_IO07_GPIO1_IO07
#define LED_MODE_PIN 1, 7 
#define LED_10K_MUX IOMUXC_SAI2_TXFS_GPIO4_IO24
#define LED_10K_PIN 4, 24

//#define FORCE_SYNC_10K_MUX IOMUXC_SAI3_RXC_GPIO4_IO29
//#define FORCE_SYNC_10K_PIN 4, 29


static void update_pins(SyncGenerator *self, LEDMask mask) {
    hal_gpio_pin_write(&self->pins[0], self->counter % 2 != 0);
    hal_gpio_pin_write(&self->pins[2], mask.LED_10kHz);
    hal_gpio_pin_write(&self->pins[3], mask.LED_1Hz);
    hal_gpio_pin_write(&self->pins[4], mask.LED_Mode);
    hal_gpio_pin_write(&self->pins[5], mask.LED_Flt);
    
}

void sync_generator_init(SyncGenerator *self, uint32_t period_us, Statistics *stats, PS_Control *MPS) {
    self->period_us = period_us;

    IOMUXC_SetPinMux(SYNC_10K_MUX, 0u);
    IOMUXC_SetPinMux(SYNC_1_MUX, 0u);
    IOMUXC_SetPinMux(LED_10K_MUX, 0u);
    IOMUXC_SetPinMux(LED_1_MUX, 0u);
    IOMUXC_SetPinMux(LED_MODE_MUX, 0u);
    IOMUXC_SetPinMux(LED_FAULT_MUX, 0u);

    hal_gpio_group_init(&self->group);
    hal_gpio_pin_init(&self->pins[0], &self->group, SYNC_10K_PIN,   HAL_GPIO_OUTPUT, HAL_GPIO_INTR_DISABLED);
    hal_gpio_pin_init(&self->pins[1], &self->group, SYNC_1_PIN,     HAL_GPIO_INPUT, HAL_GPIO_INTR_DISABLED);
    hal_gpio_pin_init(&self->pins[2], &self->group, LED_10K_PIN,   HAL_GPIO_OUTPUT, HAL_GPIO_INTR_DISABLED);
    hal_gpio_pin_init(&self->pins[3], &self->group, LED_1_PIN,     HAL_GPIO_OUTPUT, HAL_GPIO_INTR_DISABLED);
    hal_gpio_pin_init(&self->pins[4], &self->group, LED_MODE_PIN,  HAL_GPIO_OUTPUT, HAL_GPIO_INTR_DISABLED);
    hal_gpio_pin_init(&self->pins[5], &self->group, LED_FAULT_PIN, HAL_GPIO_OUTPUT, HAL_GPIO_INTR_DISABLED);
 
 
    self->sem = xSemaphoreCreateBinary();
    hal_assert(self->sem != NULL);

    self->stats = stats;
    self->MPS = MPS;

    self->counter = 0;
    LEDMask mask={0};
    update_pins(self,mask);
}

//GPT Interrupt 50uSec period
static void handle_gpt(void *data) {
    BaseType_t hptw = pdFALSE;
    SyncGenerator *self = (SyncGenerator *)data;
    // Update state and pins
    self->counter += 1;
 
    // Notify target task
    xSemaphoreGiveFromISR(self->sem, &hptw);

    // Yield to higher priority task
    portYIELD_FROM_ISR(hptw);
}

void sync_generator_task(void *param) {
    SyncGenerator *self = (SyncGenerator *)param;

    HalGpt gpt;
    hal_assert(hal_gpt_init(&gpt, 1) == HAL_SUCCESS);
    hal_log_info("GPT initialized");

    hal_assert(hal_gpt_start(&gpt, GPT_CHANNEL, self->period_us / 2, handle_gpt, (void *)self) == HAL_SUCCESS);
    for (size_t i = 0;; ++i) {
        if (xSemaphoreTake(self->sem, 10000) != pdTRUE) {
            hal_log_info("GPT semaphore timeout %x", i);
            self->MPS->Ready=0;
            SET_FAULT(Board);
            continue;
        }
        int64_t Iout_val = SCALE(Iout,Ain[0]);
        FILTER(mIout,Iout_val,t250ms);
        self->MPS->Iout = Iout_val;
        int64_t Vout_val = SCALE(Vout,Ain[1]);
        FILTER(mVout,Vout_val,t250ms);
        self->MPS->Vout = Vout_val;
        int64_t Vreg_val = SCALE(Vreg,Ain[2]);
        self->MPS->Vreg = Vreg_val;
        int64_t tHS1_val = SCALE(tHS1,Ain[3]);
        FILTER(mtHS1,tHS1_val,t250ms);
        int64_t tHS2_val = SCALE(tHS2,Ain[4]);
        FILTER(mtHS2,tHS2_val,t250ms);
        int64_t tHS3_val = SCALE(tHS3,Ain[5]);
        FILTER(mtHS3,tHS3_val,t250ms);
        self->MPS->tHeatsink = (self->MPS->mtHS1.val>self->MPS->mtHS2.val)? self->MPS->mtHS1.val:self->MPS->mtHS2.val;
        self->MPS->tHeatsink = (self->MPS->mtHS3.val>self->MPS->tHeatsink)? self->MPS->mtHS3.val:self->MPS->tHeatsink;
        //Calculate Feedcack Signal
        int32_t FB_Calc = 0, delta = 0;
        if(self->MPS->Flag.fCCMode){
            delta = (self->MPS->Ref - self->MPS->Iout);
            self->MPS->Feedback.Sum +=delta;
            int64_t smax = 0;
            if(self->MPS->Feedback.KI) smax = (160000LL*150000LL)/self->MPS->Feedback.KI;
            if(self->MPS->Feedback.Sum > smax) self->MPS->Feedback.Sum = smax;
            if(self->MPS->Feedback.Sum < -smax) self->MPS->Feedback.Sum = -smax;
            FB_Calc = ((int64_t)delta*(int64_t)self->MPS->Feedback.KP)/100LL + \
            (self->MPS->Feedback.Sum*(int64_t)self->MPS->Feedback.KI)/80000LL;
            if(self->MPS->Ref==0) {self->MPS->Feedback.Sum=0;FB_Calc=0;}
        }
        else{
            FB_Calc = self->MPS->VRef_Set;
        }
        if(FB_Calc>240000L) FB_Calc=240000L;
        if(FB_Calc<0L) FB_Calc=0L; 
        if((self->MPS->Flag.PS_ON)&&((self->MPS->Ready+self->MPS->Operate)==2)) self->MPS->Feedback.FB_Val = FB_Calc;
        else {            
            self->MPS->Ref = 0;
            self->MPS->Feedback.Sum= 0;
            self->MPS->Feedback.Sum_Add = 0;
            self->MPS->Feedback.FB_Val = 0;
        }
        //moved to MPS_Check_Faults(SkifioDin ReadDin)
        SkifioDin ReadDin = skifio_din_read();
        if((ReadDin&0x08)!=0) SET_FAULT(DCCT);      //DCCT Fault
        //if((ReadDin&0x10)!=0) SET_FAULT(GndMon);    // IGND Fault
        //if((ReadDin&0x20)!=0) SET_FAULT(Line);      // LINE Fault
        if((ReadDin&0x40)!=0) SET_FAULT(ExtLock1);  // EXT_Lock 1
        if((ReadDin&0x80)!=0) SET_FAULT(ExtLock2);  // EXT_Lock 2

        if(self->MPS->Fault_Clear_Count) { 
            self->MPS->Fault_Clear_Count--; self->MPS->Ready=0; self->MPS->Ready=0;
            self->MPS->Ref = 0;
            self->MPS->Feedback.Sum= 0;
            self->MPS->Feedback.Sum_Add = 0;
            self->MPS->Feedback.FB_Val = 0;
        }
        else {
            self->MPS->Ready=1;
            self->MPS->Ref = self->MPS->Ref_Set;
            memset(&self->MPS->Faults,0,sizeof(self->MPS->Faults));
        }
        LEDMask mask = {0};
        if(++self->timer_1Hz > 20000)   { self->timer_1Hz=0; processing_1Hz(self);} 
        if(++self->timer_5Hz > 4000)    { self->timer_5Hz=0; processing_5Hz(self);}
        size_t Pulse_2Hz = (self->timer_1Hz > 10000)? 1: 0;
        mask.LED_Flt = self->MPS->Ready*self->counter%2;
        mask.LED_10kHz =  skifio_readFlag(EXTSYNC)|Pulse_2Hz;
        mask.LED_1Hz =  skifio_readFlag(EXTSTART);
        update_pins(self,mask);
        skifio_dout_write(8*((self->counter/5)%2)+7*(self->MPS->Ready*self->MPS->Flag.PS_ON));

        if (self->counter % 2 == 0) {
            self->stats->clock_count += 1;
            //hal_gpio_pin_write(&self->pins[6],skifio_force_data_ready()!=0);
        }
        else { 
            skifio_sync_tick();
            //hal_gpio_pin_write(&self->pins[6],false);
        }
        
    }
    hal_panic();

    hal_assert(hal_gpt_stop(&gpt) == HAL_SUCCESS);
    hal_assert(hal_gpt_deinit(&gpt) == HAL_SUCCESS);
}

void sync_generator_run(SyncGenerator *self) {
    hal_assert(xTaskCreate(sync_generator_task, "sync", TASK_STACK_SIZE, (void *)self, SYNC_TASK_PRIORITY, NULL) == pdPASS);
}

void processing_1Hz(SyncGenerator *self) {
    self->MPS->Flag.f1Hz=1;
}

void processing_5Hz(SyncGenerator *self) {
    self->MPS->Flag.f5Hz=1;
}