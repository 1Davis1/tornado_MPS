#pragma once

#include <stdint.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include <hal/gpio.h>
#include <drivers/skifio.h>

#include <tasks/stats.h>
#include <device/MPS.h>

typedef struct {
    uint32_t period_us;
    HalGpioGroup group;
    HalGpioPin pins[8];
    SemaphoreHandle_t sem;
    volatile uint32_t counter;
    volatile uint32_t timer_1Hz;
    volatile uint32_t timer_5Hz;    

    Statistics *stats;
    PS_Control *MPS;

} SyncGenerator;
typedef struct {
    unsigned LED_1Hz:1;
    unsigned LED_10kHz:1;
    unsigned LED_Flt:1;
    unsigned LED_Mode:1;
} LEDMask;

void sync_generator_init(SyncGenerator *self, uint32_t period_us, Statistics *stats, PS_Control *MPS);

void sync_generator_run(SyncGenerator *self);

void processing_1Hz(SyncGenerator *self);

void processing_4Hz(SyncGenerator *self);
