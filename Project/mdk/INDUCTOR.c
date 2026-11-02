#include "INDUCTOR.h"

int32 adc_raw[4];

int32 adc_min[4] = {0, 0, 0, 0};
int32 adc_max[4] = {2500, 3000, 3000, 2500};	//rb r l lb

int32 norm_val[4];



static void sort_array(int32 *buf, uint8 len)
{
    uint8 i, j;
    int32 temp;

    for(i = 0; i < len - 1; i++)
    {
        for(j = 0; j < len - 1 - i; j++)
        {
            if(buf[j] > buf[j + 1])
            {
                temp = buf[j];
                buf[j] = buf[j + 1];
                buf[j + 1] = temp;
            }
        }
    }
}



static int32 median_filter(adc_channel_enum channel)
{
    int32 buf[FILTER_SIZE];
    uint8 i;

    
    for(i = 0; i < FILTER_SIZE; i++)
    {
        buf[i] = adc_convert(channel);

       
        system_delay_us(50);
    }

    
    sort_array(buf, FILTER_SIZE);

   
    return buf[FILTER_SIZE / 2];
}


void inductor_init(void)
{
    adc_init(INDUCTOR_1_PIN, ADC_12BIT);
    adc_init(INDUCTOR_2_PIN, ADC_12BIT);
    adc_init(INDUCTOR_3_PIN, ADC_12BIT);
    adc_init(INDUCTOR_4_PIN, ADC_12BIT);
}



void read_inductor(void)
{
		uint8 i;
	
    adc_raw[0] = median_filter(INDUCTOR_1_PIN);
    adc_raw[1] = median_filter(INDUCTOR_2_PIN);
    adc_raw[2] = median_filter(INDUCTOR_3_PIN);
    adc_raw[3] = median_filter(INDUCTOR_4_PIN);

   
    for(i = 0; i < 4; i++)
    {
        norm_val[i] = normalize(adc_raw[i], adc_min[i], adc_max[i]);
    }
}


int32 normalize(int32 raw_val, int32 min_val, int32 max_val)
{
    int32 temp;

    if(raw_val > max_val) raw_val = max_val;
    if(raw_val < min_val) raw_val = min_val;

    temp = (raw_val - min_val) * 100 / (max_val - min_val);

    return temp;
}


int32 deviation(int32 l, int32 lb, int32 rb, int32 r)
{
    int32 sub;
    int32 add;
    int32 deviation;

    sub = (a * (l - r) + b * (lb - rb)) * 100;

    add = (a * (l + r) + c * abs(lb - rb)) + 1;

    deviation = sub / add;

    return deviation;
}