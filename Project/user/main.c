#include "zf_common_headfile.h"
#include "zf_device_imu660rb.h"
#include "KALMAN.h"
#include "DRV8701.h"
#include "INDUCTOR.h"
#include "ENCODER.h"
#include "pid.h"

#define KEY_PIN                 (IO_P73)
#define LED1                    (IO_P52)

#define PIT_CH1                  (TIM1_PIT )
#define PIT_PRIORITY1            (TIMER1_IRQn)
#define TEST_DISABLE_NO_LINE_STOP (1)
uint8 pit_state1 = 0;

IncrementalPID pid_L;
IncrementalPID pid_R;
PositionPID	 pid_dir;

int16 encoder_data_dir_L = 0;
int16 encoder_data_dir_R = 0;

int32 error=0;

int16 base=0;
float dir_out;
int16 target_L;
int16 target_R;
int16 pwm_L;
int16 pwm_R;

int16 hundao_flag=0;
int16 hundao_in=0;
int16 hundao_count01=0;
int16 hundao_count02=0;
int16 hundao_count011=0;
int16 hundao_count022=0;
int16 hundao_count1=0;
int16 hundao_count2=0;
int16 hundao_state1=0;
int16 hundao_state2=0;

uint8 key_flag = 0;

int32 fuya=0;

int32 pitch_raw = 0;
int32 pitch_value = 0;
int32 pitch_360_value = 0;
int32 yaw_value = 0;
int16 yaw_360_value = 0;
int32 last_pitch_360_value = 0;
int16 pitch_flag2 =0;
int16 pitch_flag1 =0;
int16 pitch_flag0 =0;
int32 target_yaw_value=0;

int16 hundao_timer = 0;
int16 hundao_cool_time = 0;   
int16 hundao_run_state = -1;  
int16 hundao_stop=0; 

uint8 speed_pid=0;
uint8 surface_switch_count=0;
uint8 wall_confirm_count=0;
uint8 surface_state=0;

char ble_buf[64];

void pit_handler0 (void);
void pit_handler1 (void);
void control (void);
void surface_mode_update(void);
void speed_pid_set(uint8 new_case);
void encoder(void);
void hundao(void);
void imu660rb(void);
void car_stop(void);
void print_pitch_360_value(void);

void main()
{
    clock_init(SYSTEM_CLOCK_30M);
	debug_init();

	ble6a20_init();
	drv8701_init();
	encoder_init();
	inductor_init();
	imu660rb_init();
	KalmanGyro_Init();
	KalmanGyro_SetDt(0.01f);
	speed_pid_set(2);
	system_delay_ms(500);
	fuya_init(2000);

    pit_ms_init(PIT_CH1, 10);
	interrupt_set_priority(PIT_PRIORITY1, 1);
    tim1_irq_handler = pit_handler1;

	EA = 1;

    gpio_init(LED1, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(KEY_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
		
//		Incremental_PID_Init(&pid_L,4.0f,0.75f,5000);	
//		Incremental_PID_Init(&pid_R,4.1f,0.75f,5000);	
//		Position_PID_Init(&pid_dir,2.97f,0.4f,2000);		

		Incremental_PID_Init(&pid_L,5.2f,0.75f,6000);	
		Incremental_PID_Init(&pid_R,5.2f,0.75f,6000);	
		Position_PID_Init(&pid_dir,5.6f,30.5f,4000);	
		
    while(1)
    {
//	printf(":%ld,%ld,%ld,%ld,%ld,%d,%d\r\n",norm_val[2],norm_val[3],norm_val[0],norm_val[1],error,hundao_run_state,hundao_flag);	
//	printf(":%ld,%ld,%ld,%ld,%ld,%d\r\n",adc_raw[2],adc_raw[3],adc_raw[0],adc_raw[1],pitch_360_value/100,surface_state);			
//	printf(":%ld,%ld,%ld,%ld,%ld,%d,%d\r\n",norm_val[2],norm_val[3],norm_val[0],norm_val[1],error,encoder_data_dir_L,encoder_data_dir_R);
			printf(":%ld,%ld,%ld,%ld,%d,%d\r\n",adc_raw[2],adc_raw[3],adc_raw[0],adc_raw[1],encoder_data_dir_L,encoder_data_dir_R);
			if(gpio_get_level(KEY_PIN) == GPIO_LOW)
        {
            system_delay_ms(20);
            if(gpio_get_level(KEY_PIN) == GPIO_LOW)
            {
				fuya_set_duty(3000);	
         system_delay_ms(1000);		
				fuya_set_duty(4500);	
							
				key_flag=1;							
				while(gpio_get_level(KEY_PIN) == GPIO_LOW);
            }
        }
	base=800;		
    }
}

void pit_handler1 (void)
{
	read_inductor();
	encoder();
	imu660rb();
	
	error=deviation(norm_val[2],norm_val[3],norm_val[0],norm_val[1]);
//	surface_mode_update();
//	if(speed_pid==1)
//	{
//		base=800;
//	}
//	else
//	{
//		base=800;
//	}
	
	hundao();
			
	if(norm_val[0]<=5 && norm_val[1]<=5 && norm_val[2]<=5 && norm_val[3]<=5)
	{
		car_stop();
		return;
	}
	
	if(key_flag==1)
	{
		control ();
	}
}

void control (void)
{
	if(hundao_run_state == -1 ||hundao_run_state == 2)
	{
		dir_out=Position_PID_Calc(&pid_dir,0.0f,error);
	}
	else if(hundao_run_state == 0)
	{
		if(hundao_flag==1)		{dir_out=140;}
		if(hundao_flag==2)		{dir_out=-120;}
	}
	else if(hundao_run_state == 1)
	{
		if(hundao_flag==3)		{	dir_out=-20;}
		if(hundao_flag==4)		{	dir_out=20;}
	}
//			if(norm_val[2]<=25&&norm_val[1]<=25&&norm_val[3]<=10&&norm_val[0]<=10)
//			{
//					base=100;	
//					dir_out=0;
//			}
	target_L=base+dir_out;
	target_R=base-dir_out;

	pwm_L=Incremental_PID_Calc(&pid_L,target_L, encoder_data_dir_L );
	pwm_R=Incremental_PID_Calc(&pid_R,target_R, encoder_data_dir_R );

	drv8701_set_duty(pwm_L, pwm_R);
}

void surface_mode_update(void)
{
	if(surface_state==0)
	{
		wall_confirm_count=0;
		if(pitch_360_value>=3000&&pitch_360_value<=33000)
		{
			surface_switch_count++;
			if(surface_switch_count>=5)
			{

				surface_state=1;
				speed_pid_set(1);
				surface_switch_count=0;
			}
		}
		else
		{
			surface_switch_count=0;
		}
	}
	else if(surface_state==1)
	{
		wall_confirm_count=0;

		if(pitch_360_value<=3000||pitch_360_value>=33000)
		{
			surface_switch_count++;
			if(surface_switch_count>=5)
			{
				surface_state=0;
				speed_pid_set(2);
				surface_switch_count=0;
				wall_confirm_count=0;
			}
		}
		else
		{
			surface_switch_count=0;
		}
	}
	else
	{
		wall_confirm_count=0;
		if(pitch_360_value<=3000||pitch_360_value>=33000)
		{
			surface_switch_count++;
			if(surface_switch_count>=5)
			{
				surface_state=0;
				speed_pid_set(2);
				surface_switch_count=0;
			}
		}
		else
		{
			surface_switch_count=0;
		}
	}
}

void speed_pid_set(uint8 new_case)
{
	if(speed_pid==new_case)
	{
		return;
	}

	if(new_case==1)
	{
		pid_L.kp=4.0f;
		pid_L.ki=0.75f;
		pid_L.max_output=5000.0f;
		pid_R.kp=4.1f;
		pid_R.ki=0.75f;
		pid_R.max_output=5000.0f;
		pid_dir.kp=2.97f;
		pid_dir.kd=0.4f;
		pid_dir.max_output=1500.0f;
	}
	else if(new_case==2)
	{
		pid_L.kp=5.2f;
		pid_L.ki=0.75f;
		pid_L.max_output=6000.0f;
		pid_R.kp=5.2f;
		pid_R.ki=0.75f;
		pid_R.max_output=6000.0f;
		pid_dir.kp=5.6f;
		pid_dir.kd=30.5f;
		pid_dir.max_output=4000.0f;
	}
	else
	{
		return;
	}

	pid_dir.last_error=pid_dir.error;
	pid_dir.d=0.0f;
	speed_pid=new_case;
}

void encoder(void)
{
    encoder_data_dir_L = -encoder_get_count(ENCODER_DIR_1);
    encoder_data_dir_R = encoder_get_count(ENCODER_DIR_2);

    encoder_clear_count(ENCODER_DIR_1);
    encoder_clear_count(ENCODER_DIR_2);
}

void hundao(void)
{
	if(speed_pid==1)
	{
		hundao_run_state = -1;
		hundao_flag = 0;
		hundao_count01=0;
		hundao_count02=0;
		hundao_count011=0;
		hundao_count022=0;
		hundao_count1=0;
		hundao_count2=0;	
		return;
	}	
	
	if(hundao_cool_time > 0)
	{
		hundao_cool_time -= 10;
		return;
	}

	if(hundao_run_state == -1)
	{
//		if(norm_val[2]>=90&&norm_val[1]>=40&&norm_val[3]>=15&&norm_val[3]<=50&&norm_val[2]>norm_val[1]&&norm_val[3]>norm_val[0])
			if(norm_val[2]>=90&&norm_val[1]>=70&&norm_val[2]>norm_val[1])
		{
			hundao_count1++;
			hundao_count2=0;
			if(hundao_count1>=3)
			{	
				hundao_run_state = 0;
				hundao_flag=1;
				hundao_in=1;
				hundao_count1=0;
				hundao_timer=300;   
			}
		}
//		else if(norm_val[1]>=90&&norm_val[2]>=40&&norm_val[0]>=15&&norm_val[0]<=50&&norm_val[1]>norm_val[2]&&norm_val[0]>norm_val[3])
			else if(norm_val[1]>=90&&norm_val[2]>=70&&norm_val[1]>norm_val[2])
		{
			hundao_count2++;
			hundao_count1=0;
			if(hundao_count2>=3)
			{
				hundao_run_state = 0;  
				hundao_flag=2;
				hundao_in=2;
				hundao_count2=0;
				hundao_timer=300;   
			}
		}
		else
		{
			hundao_count1=0;
			hundao_count2=0;
		}
	}
	
	if(hundao_run_state == 0)
	{
		hundao_timer -= 10;
		if(hundao_timer <= 0)
		{
			hundao_run_state = 2;
			hundao_timer = 350; 
			hundao_count01=0;
			hundao_count02=0;
			return;
		}
	}

	if(hundao_run_state == 2)
	{
		hundao_timer -= 10;

		if(hundao_flag == 1) 
		{
			if(norm_val[2]>=80&&norm_val[3]>=80&&norm_val[1]>=80&&norm_val[0]>=80)
			{
				hundao_count01++;
				if(hundao_count01>=3)
				{
					hundao_run_state = 1;
					hundao_flag = 3;  
					hundao_timer = 100;  
					hundao_count01=0;
					hundao_count02=0;
					return;
				}
			}
			else
			{
				hundao_count01 = 0;
			}
		}
		else if(hundao_flag == 2) 
		{
			if(norm_val[1]>=80&&norm_val[0]>=80&&norm_val[2]>=80&&norm_val[3]>=80)
			{
				hundao_count02++;
				if(hundao_count02>=3)
				{
					hundao_run_state = 1;
					hundao_flag = 4; 
					hundao_timer = 100;
					hundao_count01=0;
					hundao_count02=0;
					return;
				}
			}
			else
			{
				hundao_count02 = 0;
			}
		}

		if(hundao_timer <= 0)
		{
			hundao_run_state = -1;
			hundao_flag = 0;
			hundao_cool_time = 3000;
			hundao_count01=0;
			hundao_count02=0;
			hundao_count011=0;
			hundao_count022=0;
			hundao_count1=0;
			hundao_count2=0;				
			return;
		}
	}
	
	if(hundao_run_state == 1 )
	{
		hundao_timer -= 10;
		if(hundao_timer <= 0)
		{
			hundao_run_state = -1;
			hundao_flag = 0;
			hundao_cool_time = 3000;
			hundao_count01=0;
			hundao_count02=0;
			hundao_count011=0;
			hundao_count022=0;
			hundao_count1=0;
			hundao_count2=0;	
		}	
	}
}
void car_stop(void)
{
	target_L = 0;
	target_R = 0;
	pwm_L = 0;
	pwm_R = 0;
	dir_out = 0;

	pid_L.error = 0;
	pid_L.last_error = 0;
	pid_L.output = 0;
	pid_R.error = 0;
	pid_R.last_error = 0;
	pid_R.output = 0;
	pid_dir.error = 0;
	pid_dir.last_error = 0;
	pid_dir.output = 0;

	drv8701_set_duty(0, 0);
}

void imu660rb(void)
{
	int32 yaw_normalized;

    KalmanGyro_UpdateFromIMU660RB(0);
    pitch_raw = KalmanGyro_GetPitchRawValue();
    pitch_value = KalmanGyro_GetPitchValue();
    pitch_360_value = KalmanGyro_GetPitch360Value();
	yaw_value = KalmanGyro_GetYawValue();
	yaw_normalized = yaw_value % 36000;
	if(yaw_normalized < 0)
	{
		yaw_normalized += 36000;
	}
	yaw_360_value = (int16)(yaw_normalized / 100);
}