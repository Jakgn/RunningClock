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
static char* itoa(int value, char* result, int base);
void RCC_Configuration(void);
void GPIO_Configuration(void);
void TIM_Configuration(void);
void USART1_Configuration(void);
void USART2_Configuration(void);
void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void OnSysTick(void);
char * _8bitToStr(uint8_t time);
void LCD_Configuration(void);
void USART1_puts(char* s);
static void Motor_PWM(void *pvParameters);
static void angle_print_on_PC(void *pvParameters);
static void user_button_press(void *pvParameters);
static void buzzer(void *pvParameters);
void PID_controller( TimerHandle_t pxTimer );
void Integral_to_Zero( TimerHandle_t pxTimer );
void cmd_check( TimerHandle_t pxTimer );
void toggle_PID_state( TimerHandle_t pxTimer );
void PID_init(float,float,float,float,float);
void PWM_calculate(void);

char timeStr[10];
xTaskHandle *pvLEDTask;
static float axes[3] = {0};
static float time = 0.0f, frametime = 0.0f;
char CmdBuffer[100];
int num = 0;
static float actual_error, previous_error, P, I, D, Kp, Ki, Kd, target, output, jserv, last_time = 0.0f;
uint8_t angleData[11];
uint8_t angleX[2];
uint8_t angleCnt = 0;
uint8_t angleFlag = 0;
int AT_mode = 0;
int PID_state = 1;
volatile int AT_cnt = 0;

int main(void)
{
	RTC_setting();
	LCD_Configuration();
	RCC_Configuration();
	GPIO_Configuration();
 	USART1_Configuration();
 	USART2_Configuration();
	TIM_Configuration();
	//PID_init(-4,40,6,1,2);
	//PID_init(0,0,0,0,0);
	I = 0;
	PID_init(0,0,7,0.8,2);
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

	TimerHandle_t PID_timer; // exec the pid every times 
	PID_timer = xTimerCreate("Timer for PID"/* Just a text name, not used by the RTOS kernel. */, 
			7/portTICK_PERIOD_MS/* X ms, The timer period in ticks. */, 
			pdTRUE, (void * const)1, /* The timers will auto-reload themselves when they expire. */ 
			PID_controller /* Each timer calls the function when it expires. */);
	if(PID_timer == NULL) {
	} else {
		 if(xTimerStart(PID_timer, 0) != pdPASS) {
		 }
	}
	
	TimerHandle_t I_zero_timer; // exec the pid every times 
	I_zero_timer = xTimerCreate("Timer for I to Zero"/* Just a text name, not used by the RTOS kernel. */, 
			3000/portTICK_PERIOD_MS/* X ms, The timer period in ticks. */, 
			pdTRUE, (void * const)1, /* The timers will auto-reload themselves when they expire. */ 
			Integral_to_Zero/* Each timer calls the function when it expires. */);
	if(I_zero_timer == NULL) {
	} else {
		 if(xTimerStart(I_zero_timer, 0) != pdPASS) {
		 }
	}
	
	TimerHandle_t cmd_check_timer; // exec the pid every times 
	cmd_check_timer = xTimerCreate("Timer for check cmd"/* Just a text name, not used by the RTOS kernel. */, 
			100/portTICK_PERIOD_MS/* X ms, The timer period in ticks. almost 0.3s */, 
			pdTRUE, (void * const)1, /* The timers will auto-reload themselves when they expire. */ 
			cmd_check/* Each timer calls the function when it expires. */);
	if(cmd_check_timer == NULL) {
	} else {
		 if(xTimerStart(cmd_check_timer, 0) != pdPASS) {
		 }
	}

	TimerHandle_t toggle_PID_state_timer; // exec the pid every times 
	toggle_PID_state_timer = xTimerCreate("Timer for toggle PID"/* Just a text name, not used by the RTOS kernel. */, 
			660/portTICK_PERIOD_MS/* X ms, The timer period in ticks. almost 0.3s */, 
			pdTRUE, (void * const)1, /* The timers will auto-reload themselves when they expire. */ 
			toggle_PID_state/* Each timer calls the function when it expires. */);
	if(toggle_PID_state_timer == NULL) {
	} else {
		 if(/*xTimerStart(toggle_PID_state_timer, 0) != pdPASS*/1) {
		 }
	}
	/*xTaskCreate(angle_print_on_PC,
			(char *) "angle_to_PC", 
			256, NULL, 
			tskIDLE_PRIORITY + 1, NULL);
*/
	xTaskCreate(user_button_press,
			(char *) "user btn pressed", 
			256, NULL, 
			tskIDLE_PRIORITY + 1, NULL);

	/*xTaskCreate(buzzer,
			(char *) "buzzer", 
			256, NULL, 
			tskIDLE_PRIORITY + 1, NULL);*/
	/*xTaskCreate(Motor_PWM,
			(char *) "Motor_PWM",
			256, NULL,
			tskIDLE_PRIORITY + 1, NULL);*/
	/* Start running the tasks. */
	
	//GPIO_SetBits(GPIOA, GPIO_Pin_5);
	vTaskStartScheduler();
	return 0;
}

void PID_init(float Param_t,float Param_j,float Param_p, float Param_i, float Param_d)
{
	jserv = Param_j;
	target = Param_t;
	actual_error = 0;
	previous_error = 0;
	P = 0;
	//I = 0;
	D = 0;
	Kp = Param_p;//7 //7
	Ki = Param_i;//1 //0.8
	Kd = Param_d;//0 //2
}
void PID_controller( TimerHandle_t pxTimer )
{
	//float now = time;
	//float delta = now - last_time;
	int16_t a[3] = {0};
	a[0] = (int16_t)(((uint16_t)angleX[1] << 8) | (uint16_t)angleX[0]);
	axes[0] = (float)a[0] / 131.0f;	
	float delta = 0.015;
	previous_error = actual_error;
	actual_error = target - axes[0];
	P = actual_error;
	I += actual_error;
	D = -(actual_error - previous_error);
	if( I > 197 ) I = 197;
	else if( I < -197) I = -197;
	output = Kp * P + Ki * I + Kd * D + jserv;
	PWM_calculate();
	//last_time = now;
}

void Integral_to_Zero( TimerHandle_t pxTimer )
{
	I = 0;
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
		//itoa((frametime - time)*1000, str, 10);
		//itoa(axes[1]*1000, str, 10);
		itoa(axes[0]*100, str, 10);
		//itoa(output, str, 10);
		//itoa(I*100, str, 10);
		strcat(str, "\r\n");	
		
		USART1_puts(str);
    }
}

static void buzzer(void *pvParameters)
{
	GPIO_SetBits(GPIOA, GPIO_Pin_6);
	GPIO_ResetBits(GPIOA, GPIO_Pin_7);
}

static void user_button_press(void *pvParameters) // when pressed, buzzer close and bluetooth disconnect
{
	volatile int i;
	while(1)
	{
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) == Bit_SET) //User Button Pressed
		{
			GPIO_SetBits(GPIOA, GPIO_Pin_5); // bluetooth's KEY pin
			while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) == Bit_SET);	
			AT_mode = 1;	
			AT_cnt = 0;
			USART1_puts("AT+DISC\r\n");
			while(AT_cnt < 2);
			USART1_puts("AT+ROLE=0\r\n");
			while(AT_cnt < 4);
			STM_EVAL_LEDToggle(LED3);
			GPIO_SetBits(GPIOA, GPIO_Pin_7); // close buzzer
			GPIO_ResetBits(GPIOA, GPIO_Pin_5); // bluetooth's KEY pin
			PID_init(0,0,0,0,0);
			AT_mode = 0;
			AT_cnt = 0;
		}
	}
}

void RCC_Configuration(void)
{
    /* --------------------------- System Clocks Configuration -----------------*/
    /* USART1 clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    /* USART2 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    /* GPIOA clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	/* For Motor */
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB , ENABLE );//Enalbe AHB for GPIOB
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM4, ENABLE );//Enable APB for TIM4
}

void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    /* For USART1 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /* Connect USART1 pins to AF */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);   // USART1_TX
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);  // USART1_RX
	
	/* For USART2 */
	GPIO_StructInit(&GPIO_InitStructure); // Reset GPIO_structure
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /* Connect USART2 pins to AF */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);   // USART1_TX
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);  // USART1_RX

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
	/* buzzer's use (6,7) and bluetooth's KEY pin (5) */	
	GPIO_StructInit(&GPIO_InitStructure); // Reset GPIO_structure
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA, GPIO_Pin_5); // bluetooth's KEY pin
	GPIO_SetBits(GPIOA, GPIO_Pin_6); // for one motor's enable
	GPIO_SetBits(GPIOA, GPIO_Pin_7); // for buzzer's black line
	/* user button */
	GPIO_StructInit(&GPIO_InitStructure); // Reset GPIO_structure
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
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
		output += 8;
		if(output > 197)
			output = 197;
		TIM4->CCR3 = 0;
		TIM4->CCR4 = 0;
		TIM4->CCR1 = output;
		TIM4->CCR2 = output;
	}
	else if( output < 0 )
	{
		output -= 8;
		if(output < -197)
			output = -197;
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
		TIM4->CCR1 = 100;
		TIM4->CCR2 = 100;
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

void USART2_Configuration(void)
{
    USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // enable USART2's receiver interrupt
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; // Configure USART2 interrupt
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // Set the priority group of USART2 interrupt
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // Set the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // Globally enable USART2 interrupt
	NVIC_Init(&NVIC_InitStructure);	

    USART_Cmd(USART2, ENABLE);
}
void USART1_IRQHandler(void)
{
	
	if( USART_GetITStatus(USART1, USART_IT_RXNE) ) {

		char t = USART_ReceiveData(USART1);
		if(AT_mode == 1)
		{
			if(t == 'O' || t == 'K')
				AT_cnt++;
			return;
		}
		CmdBuffer[num++] = t;
			//STM_EVAL_LEDToggle(LED4);
        //while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
		//USART_SendData( USART1, t ); // send back
	}
}

void USART2_IRQHandler(void) // use to receive the angle from MPU-6050
{
	angleCnt ++;
	if( USART_GetITStatus(USART2, USART_IT_RXNE) ) {
		char t = USART_ReceiveData(USART2);
		if(t == 0x55 && angleFlag == 0) {
			angleFlag = 1;
			angleCnt = 0;
		}
		else if(t == 0x55 && angleCnt == 11) {
			angleCnt = 0;
			if(angleData[1] == 0x53) {
				angleX[0] = angleData[4];
				angleX[1] = angleData[5];
			}
		}
		else if(angleFlag == 1){
			angleData[angleCnt] = t;		
		}
	}

}

void cmd_check( TimerHandle_t pxTimer )
{
	volatile int i;
		if(CmdBuffer[num - 1] == '$') 
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
				set_alarm_time(hour,min);
				char msg[]="Set alarm time success\0";
				USART1_puts(msg);
			}
			else if(CmdBuffer[0]=='o' && CmdBuffer[1]=='k')
			{
				GPIO_SetBits(GPIOA, GPIO_Pin_5); // bluetooth's KEY pin	
				for(i = 0; i < 1000000; i++);
				AT_mode = 1;
				AT_cnt = 0;
				USART1_puts("AT+DISC\r\n"); 
				//for(i = 0; i < 5000000; i++);
				while(AT_cnt < 2);
				STM_EVAL_LEDOn(LED3);
				USART1_puts("AT+ROLE=1\r\n");
				//for(i = 0; i < 5000000; i++);
				while(AT_cnt < 4);
				STM_EVAL_LEDOn(LED4);
				USART1_puts("AT+LINK=98D3,31,801F21\r\n");
				while(AT_cnt < 6);
				STM_EVAL_LEDOn(LED4);
				AT_cnt = 0;
				AT_mode = 0;
				GPIO_ResetBits(GPIOA, GPIO_Pin_5); // bluetooth's KEY pin	
			}
			else
			{
				char msg[]="Error Command";
				USART1_puts(msg);
			}
			num = 0;
		}
}

	
void toggle_PID_state( TimerHandle_t pxTimer )
{
	PID_state ^= 1;

	if(PID_state == 0)	
		PID_init(0,0,7,0.8,2);
	else
		PID_init(-3,30,6,1,2);
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

