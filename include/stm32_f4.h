#ifndef __STM32_F4_H
#define __STM32_F4_H

/* This library contains routines for interfacing with the STM32 F4 board. */

/* USART1 setting */
void RCC_Configuration(void);
void GPIO_Configuration(void);
void USART1_Configuration(void);

/* Configures the RS232 serial port */
void init_rs232(void);

void enable_rs232_interrupts(void);

void enable_rs232(void);

#endif /* __STM32_F4_H */
