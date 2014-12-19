#ifndef __RTC_SET_H__
#define __RTC_SET_H__

#include "stm32f4xx_conf.h"
void RTC_setting();
void setting_time(uint8_t, uint8_t);
void set_alarm_time(uint8_t, uint8_t, char);
#endif
