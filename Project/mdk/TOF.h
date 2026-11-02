#ifndef __TOF_H__
#define __TOF_H__

#include "zf_common_headfile.h"

#define TOF_SCL    P63
#define TOF_SDA    P65
#define TOF_ADDR   0x29

void TOF_Init(void);
uint8 TOF_Read_Distance(void);
uint8 TOF_Read_ID(void);
uint8 TOF_Read(uint16 reg); 

#endif