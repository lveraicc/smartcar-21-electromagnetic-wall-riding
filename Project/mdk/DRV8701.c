#include "DRV8701.h"

#define FUYA_PWM            (PWMB_CH3_P33)
#define FUYA_PWM_FREQ_HZ    (10000)
#define FUYA_START_DUTY     (1000)

int32 duty = 0;
int32 dir = 1;

static uint16 fuya_limit_duty(uint16 duty)
{
    if(duty > MAX_DUTY)
    {
        duty = MAX_DUTY;
    }

    return duty;
}

void drv8701_init(void)
{
    gpio_init(DIR_L, GPO, GPIO_HIGH, GPO_PUSH_PULL);                         
    pwm_init(PWM_L, 17000, 0);                                                
    gpio_init(DIR_R, GPO, GPIO_HIGH, GPO_PUSH_PULL);                         
    pwm_init(PWM_R, 17000, 0);     
}

void drv8701_set_duty(int32 left_duty, int32 right_duty)
{
		if(left_duty >= 0) 
    {
        gpio_set_level(DIR_L, GPIO_HIGH);
        pwm_set_duty(PWM_L, (left_duty * MAX_DUTY )/ 10000);
    }
    else  
    {
        gpio_set_level(DIR_L, GPIO_LOW);
        pwm_set_duty(PWM_L, ((-left_duty) * MAX_DUTY) / 10000);
    }

    if(right_duty >= 0)  
    {
        gpio_set_level(DIR_R, GPIO_LOW);
        pwm_set_duty(PWM_R, (right_duty * MAX_DUTY ) / 10000);
    }
    else  
    {
       gpio_set_level(DIR_R, GPIO_HIGH); 
       pwm_set_duty(PWM_R, ((-right_duty) * MAX_DUTY) / 10000);
    }
}

void fuya_init(uint16 target_duty)
{
    // Start at 10% for 1s, then switch to target duty.
    pwm_init(FUYA_PWM, FUYA_PWM_FREQ_HZ, FUYA_START_DUTY);
    system_delay_ms(1000);
    fuya_set_duty(target_duty);
}

void fuya_set_duty(uint16 duty)
{
    pwm_set_duty(FUYA_PWM, fuya_limit_duty(duty));
}
