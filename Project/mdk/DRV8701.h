#ifndef DRV8701_H
#define DRV8701_H

#define MAX_DUTY            (10000)                                             
#define DIR_L               (IO_P66 )
#define PWM_L               (PWMA_CH3P_P64 )

#define DIR_R               (IO_P62 )
#define PWM_R               (PWMA_CH1P_P60)

#include "zf_common_headfile.h"

void drv8701_init(void);
void drv8701_set_duty(int32 left_duty, int32 right_duty);
void fuya_init(uint16 target_duty);
void fuya_set_duty(uint16 duty);

#endif
