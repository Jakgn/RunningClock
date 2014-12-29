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
void TIM_Configuration(void);
void USART1_Configuration(void);
void USART1_IRQHandler(void);
void vTimerCallback( TimerHandle_t pxTimer );
void OnSysTick(void);
char * _8bitToStr(uint8_t time);
void LCD_Configuration(void);
void USART1_puts(char* s);
static void Motor_PWM(void *pvParameters);
static void angle_print_on_PC(void *pvParameters);
void PID_controller( TimerHandle_t pxTimer );
void PID_init(void);
void PWM_calculate(void);

char timeStr[10];
xTaskHandle *pvLEDTask;
static float axes[3] = {0};
static float time = 0.0f, frametime = 0.0f;
char CmdBuffer[100];
int num = 0;
static float actual_error, previous_error, P, I, D, Kp, Ki, Kd, target, output;
int main(void)
{
	RTC_setting();
	LCD_Configuration();
	RCC_Configuration();
	GPIO_Configuration();
 	USART1_Configuration();
	TIM_Configuration();
	PID_init();
	STM_EVAL_LEDInit(LED4);
	STM_EVAL_LEDInit(LED3);
	/* Turn OFF all LEDs */
	STM_EVAL_LEDOff(LED4);
	STM_EVAL_LEDOff(LED3);

	/* Create a task to display Time in the LCD. */
/*	xTaskCreate(LCD_display_Time_task,
			(signed portCHAR *) "Liquid Crystal Display",
			128 , NULL,
			tskIDLE_PRIORITY + 1, NULL);
*/	/* Create a task to flash the LED. */
/*	xTaskCreate(LED_task,
			(signed portCHAR *) "LED Flash",
			128*/ /* stack size */ /*, NULL,
			tskIDLE_PRIORITY + 1, pvLEDTask );
*/
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

	TimerHandle_t PID_timer; // exec the pid every times 
	PID_timer = xTimerCreate("Timer for PID"/* Just a text name, not used by the RTOS kernel. */, 
			50/portTICK_PERIOD_MS/* X ms, The timer period in ticks. */, 
			pdTRUE, (void * const)1, /* The timers will auto-reload themselves when they expire. */ 
			PID_controller /* Each timer calls the function when it expires. */);
	if(PID_timer == NULL) {
	} else {
		 if(xTimerStart(PID_timer, 0) != pdPASS) {
		 }
	}
	
	xTaskCreate(GyroscopeTask,
			(char *) "Gyroscope", 
			256, NULL, 
			tskIDLE_PRIORITY + 1, NULL);

	xTaskCreate(angle_print_on_PC,
			(char *) "angle_to_PC", 
			256, NULL, 
			tskIDLE_PRIORITY + 1, NULL);

	/*xTaskCreate(Motor_PWM,
			(char *) "Motor_PWM",
			256, NULL,
			tskIDLE_PRIORITY + 1, NULL);*/
	/* Start running the tasks. */

	vTaskStartScheduler();
	return 0;
}

void PID_init(void)
{
	target = 0;
	actual_error = 0;
	previous_error = 0;
	P = 0;
	I = 0;
	D = 0;
	Kp = 50;
	Ki = 10;
	Kd = 1;

}
void PID_controller( TimerHandle_t pxTimer )
{
	previous_error = actual_error;
	actual_error = target - axes[1];
	P = actual_error;
	I += previous_error;
	D = actual_error - previous_error;
	output = Kp * P + Ki * I + Kd * D;
	PWM_calculate();

}

void USART1_puts(char* s)
{
    while(*s) {
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, *s);
        s++;
    }
}

static void angle_print_on_PC(void *pvParameters)
{
	char str[50];
	while(1) {
		itoa(axes[1]*1000, str, 10);
		//itoa(output, str, 10);
		strcat(str, "\n");	
		
		USART1_puts(str);
    }
}
void RCC_Configuration(void)
{
    /* --------------------------- System Clocks Configuration -----------------*/
    /* USART1 clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    /* GPIOA clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* For Motor */
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB , ENABLE );//Enalbe AHB for GPIOB
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM4, ENABLE );//Enable APB for TIM4
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

	/* For Motor PWM */
	GPIO_StructInit(&GPIO_InitStructure); // Reset GPIO_structure
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4); // set GPIOB_Pin6 to AF_TIM4
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // Alt Function - Push Pull
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init( GPIOB, &GPIO_InitStructure );
	/* For Motor PWM */
	GPIO_StructInit(&GPIO_InitStructure); // Reset GPIO_structure
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4); // set GPIOB_Pin6 to AF_TIM4
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // Alt Function - Push Pull
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init( GPIOB, &GPIO_InitStructure );
	/* For Motor PWM */
	GPIO_StructInit(&GPIO_InitStructure); // Reset GPIO_structure
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4); // set GPIOB_Pin6 to AF_TIM4
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // Alt Function - Push Pull
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init( GPIOB, &GPIO_InitStructure );
	/* For Motor PWM */
	GPIO_StructInit(&GPIO_InitStructure); // Reset GPIO_structure
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM4); // set GPIOB_Pin6 to AF_TIM4
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // Alt Function - Push Pull
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init( GPIOB, &GPIO_InitStructure );
}

void TIM_Configuration(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	// Let PWM frequency equal XX Hz.
	TIM_TimeBaseStructInit( &TIM_TimeBaseInitStruct );
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV4;
	TIM_TimeBaseInitStruct.TIM_Period = 200 - 1; //84000000/x*1000= XX hz ex:20ms for cycle
	TIM_TimeBaseInitStruct.TIM_Prescaler = 1000 - 1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit( TIM4, &TIM_TimeBaseInitStruct );
	TIM_OCStructInit( &TIM_OCInitStruct );
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	// Initial duty cycle equals 100%. Value can range from zero to 65535.
	//TIM_Pulse = TIM4_CCR1 register (16 bits)
	TIM_OCInitStruct.TIM_Pulse = 65535; //(0=Always Off, 65535=Always On)
	TIM_OC1Init( TIM4, &TIM_OCInitStruct ); // Channel 1
	TIM_OC2Init( TIM4, &TIM_OCInitStruct ); // Channel 2
	TIM_OC3Init( TIM4, &TIM_OCInitStruct ); // Channel 3
	TIM_OC4Init( TIM4, &TIM_OCInitStruct ); // Channel 4
	TIM_Cmd( TIM4, ENABLE );
}

void PWM_calculate(void)
{
	if( output > 0 )
	{
		TIM4->CCR3 = 0;
		TIM4->CCR4 = 0;
		TIM4->CCR1 = output;
		TIM4->CCR2 = output;
	}
	else if( output <= 0 )
	{
		TIM4->CCR1 = 0;
		TIM4->CCR2 = 0;
		TIM4->CCR3 = -output;
		TIM4->CCR4 = -output;
	}
}
static void Motor_PWM(void *pvParameters)
{
	volatile int i;

	while(1) // Do not exit
	{
		STM_EVAL_LEDToggle(LED3);
		TIM4->CCR1 = 32;
		TIM4->CCR2 = 32;
		TIM4->CCR3 = 0;
		TIM4->CCR4 = 0;
		for(i=0;i<10000000;i++);
		STM_EVAL_LEDToggle(LED3);
		TIM4->CCR1 = 0;
		TIM4->CCR2 = 0;
		TIM4->CCR3 = 0;
		TIM4->CCR4 = 0;
		for(i=0;i<10000000;i++); // delay
	}

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

