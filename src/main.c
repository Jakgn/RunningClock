/**
 ******************************************************************************
 * @file    RunningClock/src/main.c 
 * @author	embedded2014 Fall  
 * @version V1.0.0
 * @date	2014 Fall 
 * @brief   Main program body
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <string.h>
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
static void LCD_display_Time_task(void *pvParameters);
static void Gyroscope_Init(void);
static void Gyroscope_Update(void);
static void Gyroscope_Render(void);
static void GyroscopeTask(void *pvParameters);
static char* itoa(int value, char* result, int base);
void RCC_Configuration(void);
void GPIO_Configuration(void);
void USART1_Configuration(void);
void USART1_IRQHandler(void);
void vTimerCallback( TimerHandle_t pxTimer );
void OnSysTick(void);
char * _8bitToStr(uint8_t time);
void LCD_Configuration(void);
void USART1_puts(char* s);

char timeStr[10];
xTaskHandle *pvLEDTask;
static float axes[3] = {0};
static float time = 0.0f, frametime = 0.0f;
char CmdBuffer[100];
int num = 0;

int main(void)
{
	RTC_setting();
	LCD_Configuration();
	RCC_Configuration();
	GPIO_Configuration();
 	USART1_Configuration();

	STM_EVAL_LEDInit(LED4);
	STM_EVAL_LEDInit(LED3);
	/* Turn OFF all LEDs */
	STM_EVAL_LEDOff(LED4);
	STM_EVAL_LEDOff(LED3);

	/* Create a task to display Time in the LCD. */
	xTaskCreate(LCD_display_Time_task,
			(signed portCHAR *) "Liquid Crystal Display",
			128 , NULL,
			tskIDLE_PRIORITY + 1, NULL);
	/* Create a task to flash the LED. */
	xTaskCreate(LED_task,
			(signed portCHAR *) "LED Flash",
			128 /* stack size */, NULL,
			tskIDLE_PRIORITY + 1, pvLEDTask );
	
	TimerHandle_t timer; // calculate the angle in gyroscope with angular velocity, it needs time
	timer = xTimerCreate("Timer for getting angle"/* Just a text name, not used by the RTOS kernel. */, 
			10/portTICK_PERIOD_MS/* 10ms, The timer period in ticks. */, 
			pdTRUE, (void * const)1, /* The timers will auto-reload themselves when they expire. */ 
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

void USART1_puts(char* s)
{
    while(*s) {
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, *s);
        s++;
    }
}

void RCC_Configuration(void)
{
      /* --------------------------- System Clocks Configuration -----------------*/
      /* USART1 clock enable */
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
      /* GPIOA clock enable */
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
}

void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /*-------------------------- GPIO Configuration ----------------------------*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Connect USART pins to AF */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);   // USART1_TX
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);  // USART1_RX
}
 
void USART1_Configuration(void)
{
    USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate = 38400;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // enable USART1's receiver interrupt

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; // Configure USART1 interrupt
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // Set the priority group of USART1 interrupt
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // Set the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // Globally enable USART1 interrupt
	NVIC_Init(&NVIC_InitStructure);	

    USART_Cmd(USART1, ENABLE);
}

/* USART1 interrupt be used to receive the bluetooth device */
void USART1_IRQHandler(void)
{
	if( USART_GetITStatus(USART1, USART_IT_RXNE) ) {

		char t = USART_ReceiveData(USART1);
		if( t != '$')
			CmdBuffer[num++] = t;
		else
		{
			if(CmdBuffer[0]=='s' && CmdBuffer[1]=='e' && CmdBuffer[2]=='t')
			{
				uint8_t hour = ((CmdBuffer[3]-'0')<<4) | (CmdBuffer[4]-'0');
				uint8_t min = ((CmdBuffer[5]-'0')<<4) | (CmdBuffer[6]-'0');
				setting_time(hour,min);
				char msg[]="Set time success\0";
				USART1_puts(msg);
			}
			else if(CmdBuffer[0]=='a' && CmdBuffer[1]=='l' && CmdBuffer[2]=='a')
			{
				uint8_t hour = ((CmdBuffer[3]-'0')<<4) | (CmdBuffer[4]-'0');
				uint8_t min = ((CmdBuffer[5]-'0')<<4) | (CmdBuffer[6]-'0');
				set_alarm_time(hour,min,CmdBuffer[7]);
				char msg[]="Set alarm time success\0";
				USART1_puts(msg);
			}
			else
			{
				char msg[]="Error Command";
				USART1_puts(msg);
			}
			num = 0;
		}
        //while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
		//USART_SendData( USART1, t ); // send back
	}
}

void vTimerCallback( TimerHandle_t pxTimer ){
	configASSERT( pxTimer );
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
	while(1) {
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

static void LCD_display_Time_task(void *pvParameters)
{
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

void LCD_Configuration(void)
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

