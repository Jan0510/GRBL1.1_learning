#ifndef timer_h
#define timer_h
#include "grbl.h"
void TIM3_Int_Init(u16 arr,u16 psc);
void TIM4_Int_Init(u16 arr,u16 psc);
void TIM5_Int_Init(u16 arr,u16 psc);
void TIM14_PWM_Init(u32 arr,u32 psc);
#endif

