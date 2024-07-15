/*
 * rtc_delay.h
 *
 *  Created on: Jul 12, 2024
 *      Author: j
 */

#include "rtc_delay.h"



void rtc_set_alarm_seconds_it(RTC_HandleTypeDef* hrtc, uint16_t seconds){
	RTC_AlarmTypeDef sAlarm;

	    // Get the current time
	    RTC_TimeTypeDef sTime;
	    HAL_RTC_GetTime(hrtc, &sTime, RTC_FORMAT_BIN);

	    // Calculate alarm time (current time + period)
	    sAlarm.AlarmTime.Seconds = sTime.Seconds + seconds;
	    sAlarm.AlarmTime.Minutes = sTime.Minutes;
	    sAlarm.AlarmTime.Hours = sTime.Hours;

	    // Normalize alarm time
	    if (sAlarm.AlarmTime.Seconds >= 60) {
	        sAlarm.AlarmTime.Seconds -= 60;
	        sAlarm.AlarmTime.Minutes++;
	    }
	    if (sAlarm.AlarmTime.Minutes >= 60) {
	        sAlarm.AlarmTime.Minutes -= 60;
	        sAlarm.AlarmTime.Hours++;
	    }
	    if (sAlarm.AlarmTime.Hours >= 24) {
	        sAlarm.AlarmTime.Hours -= 24;
	    }

	    sAlarm.Alarm = RTC_ALARM_A;

	    if (HAL_RTC_SetAlarm_IT(hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK) {
	        Error_Handler();
	    }

}
