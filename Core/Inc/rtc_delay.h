/*
 * rtc_delay.h
 *
 *  Created on: Jul 12, 2024
 *      Author: j
 */

#ifndef INC_RTC_DELAY_H_
#define INC_RTC_DELAY_H_

#include "main.h"


void rtc_set_alarm_seconds_it(RTC_HandleTypeDef* hrtc, uint16_t seconds);


#endif /* INC_RTC_DELAY_H_ */
