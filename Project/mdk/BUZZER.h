#ifndef BUZZER_H
#define BUZZER_H

#include "zf_common_headfile.h"

#define BUZZER_PIN              (IO_P67)

void buzzer_init(void);
void buzzer_on(void);
void buzzer_off(void);
void buzzer_set_enable(uint8 enable);
uint8 buzzer_get_enable(void);

#endif
