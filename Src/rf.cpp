
#include <math.h>

#include "main.h"

//TT
//BI(SI)
//BD(SD)
//ISI


#define DEFALUT_FREQUENCY       250000
uint32_t FundamentalFrequency = DEFALUT_FREQUENCY;

//TBD(Tone Burst Duration)
uint32_t ToneBurstDuration =  20;
//PRP = TBD+IDLE
uint32_t PRP ;

extern "C" void RF_init(void)
{
  return;
}

extern "C" void RF_setFrequency(void)
{

}

extern "C" uint16_t setOutputVoltage(void)
{

}

extern "C" uint16_t setTBDAndDuty(void)
{


}

extern "C" uint16_t setTiming(void)
{

}

extern "C" uint16_t setImpedance(void)
{

}