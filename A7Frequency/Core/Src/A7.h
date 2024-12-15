/*
 * A7.h
 *
 *  Created on: Nov 9, 2024
 *      Author: camil
 */

#ifndef SRC_A7_H_
#define SRC_A7_H_

#define BRR_num 694
#define MAX_TIME 0xFFFFFFFF
#define MICROSECOND 1000000
#define NVIC_REG_MASK 0x1F
#define SAMPLE_SIZE 10000
#define ONEHUNDREDUS 100
#define FREQUENCY_CALIBRATION 0.99199
#define FREQUENCY_OFFSET 1

uint16_t get_frequency(void);
uint16_t get_average(uint16_t []);
void print_freq(uint16_t);
void TIM2_Init(void);
void ADC_init(void);
void USART_ESC_Code(char*);
void UART_print(char*);
void UART_init(void);

#endif /* SRC_A7_H_ */
