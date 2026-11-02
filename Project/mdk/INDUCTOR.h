#ifndef INDUCTOR_H
#define INDUCTOR_H

#include "zf_common_headfile.h"

#define FILTER_SIZE 5

#define INDUCTOR_1_PIN    ADC_CH8_P00   // P0.0
#define INDUCTOR_2_PIN    ADC_CH9_P01   // P0.1
#define INDUCTOR_3_PIN    ADC_CH13_P05  // P0.5
#define INDUCTOR_4_PIN    ADC_CH14_P06  // P0.6

#define a 1  
#define b 1.0  
#define c 1.0

extern int32 adc_raw[4];      
extern int32 adc_min[4];       
extern int32 adc_max[4];      
extern int32 norm_val[4];     



void inductor_init(void);
void read_inductor(void);
int32 normalize(int32 raw_val, int32 min_val, int32 max_val);
int32 deviation(int32 l, int32 lb, int32 rb, int32 r);

#endif 