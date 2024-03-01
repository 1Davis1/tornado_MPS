// MPS Control Class
// used for define control signals
// MPS 150 and MPS300
#pragma once

#include <stdint.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include <hal/gpio.h>

#define MPS_CTRL_VAR
#define ISETMAX 150000L
#define VSETMAX 24000L
#define t250ms 249L
#define t20ms 24L
#define t2ms 3L
#define SCALE(X,I) ((int64_t)(self->MPS->I-self->MPS->Offset.X) *(int64_t)self->MPS->K.X)/1000000LL
#define FILTER(X,I,T) self-> MPS->X.val = ((int64_t)self->MPS->X.val*T+I)/(T+1) ; \
self-> MPS->X.add += (self->MPS->X.val*T+I)%(T+1);\
if(self-> MPS->X.add  > (T+1)) {self->MPS->X.val++; self-> MPS->X.add-=(T+1);}\
if(self-> MPS->X.add  < -(T+1)) {self->MPS->X.val--; self-> MPS->X.add+=(T+1);}

typedef struct 
{
    int32_t val;
    int32_t add;
} Filter_Mes;
typedef struct 
{
    int32_t Iout;
    int32_t Vout;
    int32_t Vreg;
    int32_t tHS1;
    int32_t tHS2;
    int32_t tHS3;
    int32_t Iset;
    int32_t Vset;
}Channels;


typedef struct 
{
    int32_t VRef_Set;
    int32_t Ref_Set; //Reference value with 100uA discrette 10000 = 1,0000 A
    int32_t Ref;     //Applied for Feedback regulator Reference value
    int32_t FeedForward;
    struct 
    {
        int32_t FB_Val;
        int64_t Delta;
        int64_t Sum;
        int64_t Sum_Add;
        int64_t dVal;
        int32_t KP;
        int32_t KI;
        int32_t KD;
        int32_t KP2;
        int32_t KI2;
        int32_t KD2;
    }Feedback;
    volatile struct 
    {
        unsigned f5Hz:1;
        unsigned f1Hz:1;
        unsigned fLocale:1;
        unsigned fCCMode:1;
        unsigned PS_ON:1;
    }Flag;
    
    int32_t Ain[8];
    Channels K;
    Channels Offset;
    int32_t Iout;    //Measured value with 100uA discrette 10000 = 1,0000 A
    Filter_Mes mIout;
    int32_t Iout_Add;//filter cacl division reminder 
    int32_t Vout;    //Measured value with 100uV discrette 10000 = 1,0000 V
    Filter_Mes mVout;//filter cacl division reminder 
    int32_t tHS1;    //Measured value of t HeatSink#1 with 0,1C discrette 300 = 30,0 C
    Filter_Mes mtHS1;//filter cacl division reminder 
    int32_t tHS2;    //Measured value of t HeatSink#2 with 0,1C discrette 300 = 30,0 C
    Filter_Mes mtHS2;//filter cacl division reminder 
    int32_t tHS3;    //Measured value of t HeatSink#3 with 0,1C discrette 300 = 30,0 C
    Filter_Mes mtHS3;//filter cacl division reminder 
    int32_t tHeatsink;//Max value of Heatsink temperatures with 0,1C discrette 300 = 30,0 C
    int32_t Vreg;     //Measured value of VDAC readback with 100uV discrette
    int32_t Ready;   //Power Source Ready state 0 - not Ready , 1 - Ready
    int32_t Operate; //Power Source Operate state 0 - Off state, 1 - On state
    int32_t Fault_Clear_Count;
    uint32_t ms_tick;
    struct 
    {
        unsigned Overcurrent:1;
        unsigned Overvoltage:1;
        unsigned Overheat:1;
        unsigned Unit1:1;
        unsigned Unit2:1;
        unsigned Unit3:1;
        unsigned FBLost:1;
        unsigned Board:1;
        unsigned DCCT:1;
        unsigned Line:1;
        unsigned GndMon:1;
        unsigned ExtLock1:1;
        unsigned ExtLock2:1;
    } Faults;
    
} PS_Control;
