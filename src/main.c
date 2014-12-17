/**
 ******************************************************************************
 * @file    Template/main.c 
 * @author  MCD Application Team
 * @version V1.0.0
 * @date    20-September-2013
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
 *
 * Unless required by applicable law or agreed to in writing, software 
 * distributed under the License is distributed on an "AS IS" BASIS, 
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "RTC.h"
#include "main.h"
#include "stm32f4xx_conf.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static void LED_task(void *pvParameters);
static void LCD_display_task(void *pvParameters);
static void Gyroscope_Init(void);
static void Gyroscope_Update(void);
static void Gyroscope_Render(void);
static void GyroscopeTask(void *pvParameters);
static char* itoa(int value, char* result, int base);
void vTimerCallback( TimerHandle_t pxTimer );
void OnSysTick(void);
char * _8bitToStr(uint8_t time);
void prvInit();

char timeStr[];
xTaskHandle *pvLEDTask;
static float axes[3] = {0};
static float time = 0.0f, frametime = 0.0f;

int main(void)
{
	RTC_setting();
	prvInit();
	/* Turn OFF all LEDs */
	STM_EVAL_LEDOff(LED4);
	STM_EVAL_LEDOff(LED3);

	/* Create a task to display something in the LCD. */
	xTaskCreate(LCD_display_task,
			(signed portCHAR *) "Liquid Crystal Display",
			128 , NULL,
			tskIDLE_PRIORITY + 1, NULL);
	/* Create a task to flash the LED. */
	xTaskCreate(LED_task,
			(signed portCHAR *) "LED Flash",
			128 /* stack size */, NULL,
			tskIDLE_PRIORITY + 1, pvLEDTask );

	TimerHandle_t timer; // calculate the angle in gyroscope with angular velocity, it needs time
	timer = xTimerCreate("timer"/* Just a text name, not used by the RTOS kernel. */, 
			10/portTICK_PERIOD_MS/* 10ms, The timer period in ticks. */, 
			pdTRUE, 1, /* The timers will auto-reload themselves when they expire. */ 
			vTimerCallback /* Each timer calls the same callback when it expires. */);
	if(timer == NULL) {
	} else {
		 if(xTimerStart(timer, 0) != pdPASS) {
		 }
	}

	//SysTick_Config(SystemCoreClock / 100); // SysTick event each 10ms
	xTaskCreate(GyroscopeTask, 
			(char *) "Gyroscope", 
			256, NULL, 
			tskIDLE_PRIORITY + 1, NULL);
	/* Start running the tasks. */
	vTaskStartScheduler();
	return 0;
}

void vTimerCallback( TimerHandle_t pxTimer ){
	configASSERT( pxTimer );
	time += 0.01f;
}
void OnSysTick(void)
{
	time += 0.01f;
}

static void Gyroscope_Init(void)
{
	L3GD20_InitTypeDef L3GD20_InitStructure;
	L3GD20_InitStructure.Power_Mode = L3GD20_MODE_ACTIVE;
	L3GD20_InitStructure.Output_DataRate = L3GD20_OUTPUT_DATARATE_1;
	L3GD20_InitStructure.Axes_Enable = L3GD20_AXES_ENABLE;
	L3GD20_InitStructure.Band_Width = L3GD20_BANDWIDTH_4;
	L3GD20_InitStructure.BlockData_Update = L3GD20_BlockDataUpdate_Continous;
	L3GD20_InitStructure.Endianness = L3GD20_BLE_LSB;
	L3GD20_InitStructure.Full_Scale = L3GD20_FULLSCALE_250;
	L3GD20_Init(&L3GD20_InitStructure);
	L3GD20_FilterConfigTypeDef L3GD20_FilterStructure;
	L3GD20_FilterStructure.HighPassFilter_Mode_Selection = L3GD20_HPM_NORMAL_MODE_RES;
	L3GD20_FilterStructure.HighPassFilter_CutOff_Frequency = L3GD20_HPFCF_0;
	L3GD20_FilterConfig(&L3GD20_FilterStructure);
	L3GD20_FilterCmd(L3GD20_HIGHPASSFILTER_ENABLE);
}

static void Gyroscope_Update(void)
{
	uint8_t tmp[6] = {0};
	int16_t a[3] = {0};
	uint8_t tmpreg = 0;
	L3GD20_Read(&tmpreg, L3GD20_CTRL_REG4_ADDR, 1);
	L3GD20_Read(tmp, L3GD20_OUT_X_L_ADDR, 6);
	/* check in the control register 4 the data alignment (Big Endian or Little Endian)*/
	if (!(tmpreg & 0x40)) {
		for (int i = 0; i < 3; i++)
			a[i] = (int16_t)(((uint16_t)tmp[2 * i + 1] << 8) | (uint16_t)tmp[2 * i]);
	} else {
		for (int i = 0; i < 3; i++)
			a[i] = (int16_t)(((uint16_t)tmp[2 * i] << 8) | (uint16_t)tmp[2 * i + 1]);
	}
	float delta = frametime - time;
	frametime = time; // make frametime a consistent time value during the frames
	for (int i = 0; i < 3; i++){
		//axes[i] = a[i] / 114.285f;
		axes[i] += (float)a[i] * delta / 114.285f;
	}
	if (axes[0] < 0) axes[0] = 0;
	if (axes[0] > 180) axes[0] = 180;
}
static void Gyroscope_Render(void)
{
	LCD_ClearLine(LCD_LINE_1);
	LCD_ClearLine(LCD_LINE_2);
	LCD_ClearLine(LCD_LINE_3);
	LCD_SetTextColor( LCD_COLOR_RED );
	char str[16] = "X: ";
	itoa(axes[0], str + 3, 10);
	LCD_DisplayStringLine(LCD_LINE_1, str);
	str[0] = 'Y';
	itoa(axes[1], str + 3, 10);
	LCD_DisplayStringLine(LCD_LINE_2, str);
	str[0] = 'Z';
	itoa(axes[2], str + 3, 10);
	LCD_DisplayStringLine(LCD_LINE_3, str);
}
static void GyroscopeTask(void *pvParameters)
{
	Gyroscope_Init();
	while(1){
		Gyroscope_Update();
		Gyroscope_Render();
	}
}

static char* itoa(int value, char* result, int base)
{
	if (base < 2 || base > 36) {
		*result = '\0';
		return result;
	}
	char *ptr = result, *ptr1 = result, tmp_char;
	int tmp_value;
	do {
		tmp_value = value;
		value /= base;
		*ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
	} while (value);
	if (tmp_value < 0) *ptr++ = '-';
	*ptr-- = '\0';
	while (ptr1 < ptr) {
		tmp_char = *ptr;
		*ptr-- = *ptr1;
		*ptr1++ = tmp_char;
	}
	return result;
}

static void LCD_display_task(void *pvParameters)
{
	char *hello = "hello world";	
	char sec[3], min[3], hour[10], time[15];
	LCD_SetColors(LCD_COLOR_RED, LCD_COLOR_GREY);
	RTC_TimeTypeDef RTC_TimeStruct;
	RTC_DateTypeDef RTC_DateStruct;
	while(1){
		RTC_GetTime(RTC_Format_BIN, &RTC_TimeStruct);
		RTC_GetDate(RTC_Format_BIN, &RTC_DateStruct);
		strcpy(sec, _8bitToStr(RTC_TimeStruct.RTC_Seconds));
		strcpy(min, _8bitToStr(RTC_TimeStruct.RTC_Minutes));
		strcpy(hour, _8bitToStr(RTC_TimeStruct.RTC_Hours));
		strcpy(time,hour);
		strcat(time, ":");
		strcat(time, min);
		strcat(time, ":");
		strcat(time, sec);
		LCD_DisplayStringLine(LCD_LINE_8, time);
			
		/*showCalendar_time((uint8_t)RTC_TimeStruct.RTC_Hours,
		(uint8_t)RTC_TimeStruct.RTC_Minutes,
		(uint8_t)RTC_TimeStruct.RTC_Seconds);
		showCalendar_date((uint8_t)RTC_DateStruct.RTC_Year,
		(uint8_t)RTC_DateStruct.RTC_Month,
		(uint8_t)RTC_DateStruct.RTC_Date);
	*/}
}
static void LED_task(void *pvParameters)
{
	RCC_ClocksTypeDef RCC_Clocks;
	uint8_t togglecounter = 0x00;
	while(1)
	{
		/* Toggle LED3 */
		STM_EVAL_LEDToggle(LED3);
		vTaskDelay(200);
		/* Toggle LED4 */
		STM_EVAL_LEDToggle(LED4);
		vTaskDelay(200);
	}
}

void prvInit()
{
	//LCD init
	LCD_Init();
	IOE_Config();
	LTDC_Cmd( ENABLE );
	LCD_LayerInit();
	//Two LCD Layers setting initial
	LCD_SetLayer( LCD_BACKGROUND_LAYER );
	LCD_Clear( LCD_COLOR_BLACK );
	LCD_SetTransparency(0x00);
	LCD_SetLayer( LCD_FOREGROUND_LAYER );
	LCD_SetTransparency(0xFF);
	LCD_Clear( LCD_COLOR_BLACK );
	LCD_SetTextColor( LCD_COLOR_WHITE );

	STM_EVAL_LEDInit(LED4);
	STM_EVAL_LEDInit(LED3);
}

char * _8bitToStr(uint8_t time)	// Hexadecimal to Decimal'string 
{
	uint8_t cnt = 0;
	cnt += ((time & 0xF0) >> 4) * 16;
	cnt += (time & 0x0F);
	*timeStr = cnt/10 + '0';
	*(timeStr+1) = cnt%10 + '0';
	*(timeStr+2) = '\0';
	return timeStr;
}

