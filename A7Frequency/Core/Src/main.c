/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "A7.h"
void SystemClock_Config(void);

// global variables
uint16_t samples[SAMPLE_SIZE];
uint32_t idx = 0;
uint8_t get_avg_flag = 0;



int main(void)
{

  HAL_Init();
  SystemClock_Config();
  ADC_init();
  TIM2_Init();
  UART_init();
  __enable_irq();

  // clear screen and reset cursor
 	USART_ESC_Code("[2J");
 	USART_ESC_Code("[H");


  while (1)
  {
	  /*
	   * @brief Calculates frequency of a pure sine wave from 1Hz to 1000Hz.
	   */
	  static uint16_t freq = 0;
	  // if done sampling wave, calculate frequency and print to terminal
	  if (get_avg_flag){
		   	freq = get_frequency();
		   	UART_print("Sine frequency: ");
		   	print_freq(freq);
		   	get_avg_flag = 0;
	  }
  }

}

//---------------------------------- Helper functions --------------------------------------

//Calculates frequency by counting how many times the sine wave is at the average value
uint16_t get_frequency(void){
	uint16_t avg_point = 0;
	uint16_t average = get_average(samples);
	for (uint16_t i = 1; i < SAMPLE_SIZE; i++) {
		if ((samples[i - 1] > average) && (samples[i] <= average)){
			avg_point++;
		}
	}
	// calibrate frequency since its a little off
	uint16_t frequency = avg_point;
	return frequency;
}

// Gets average value from array of samples
uint16_t get_average(uint16_t samples[]) {
	uint32_t mean = 0;
	for (uint32_t i = 0; i < SAMPLE_SIZE; i++) {
		mean += samples[i];
	}
	mean /= SAMPLE_SIZE;
	return mean;
}

//Prints the sine wave frequency
void print_freq(uint16_t frequency) {
	 // clear screen and reset cursor
	USART_ESC_Code("[2J");
	USART_ESC_Code("[H");

	char freq_buf[7];
	uint8_t idx = 0;

	if (frequency >= 1 && frequency <= 1000) {
		int temp = frequency;
		// Extract each digit
		while (temp > 0) {
			freq_buf[idx++] = '0' + (temp % 10);
			temp /= 10;
		}

		// Reverse the string
		for (int j = 0; j < idx / 2; j++) {
			char tempChar = freq_buf[j];
			freq_buf[j] = freq_buf[idx - j - 1];
			freq_buf[idx - j - 1] = tempChar;
		}
		freq_buf[idx++] = 'H';
		freq_buf[idx++] = 'z';
		freq_buf[idx] = '\0';
	}
	    // Error message for out of range
	    else {
	        UART_print("Frequency out of range!");
	        return;
	    }
	UART_print("Frequency is: ");
	UART_print(freq_buf);
	return;
}

//------------------------------------------------------------------------------------

//---------------------------------- TIM2 Stuff --------------------------------------

/*
 * @brief Prints frequency every two seconds or resets counts
 */
void TIM2_IRQHandler(void){
	// ARR max reached
	if (TIM2->SR & TIM_SR_UIF) {
		// start ADC
		ADC1->CR |= (ADC_CR_ADSTART);
		// clear flag
		TIM2->SR &= ~TIM_SR_UIF;
	}
}

/*
 * @brief Initializes TIM2
 */
void TIM2_Init(void){
	// enable TIM2 clock
	RCC->APB1ENR1 |= (RCC_APB1ENR1_TIM2EN);
	// set TIM2 as an up counter
	TIM2->CR1 &= ~TIM_CR1_DIR;
	// Pre-scaler value to 100kHz
    TIM2->PSC = 80 - 1;
    // sample at 10kHz
    TIM2->ARR = ONEHUNDREDUS;
	// enable update event interrupt in TIM2 and CCR1 interrupts
	TIM2->DIER |=(TIM_DIER_UIE);
	// clear interrupt status register for update event and CCR1
	TIM2->SR &= ~(TIM_SR_UIF);
	// start timer
	TIM2->CR1 |= TIM_CR1_CEN;
	// enable TIM2 interrupts
	NVIC->ISER[0] = (1 <<(TIM2_IRQn & NVIC_REG_MASK));
	// Enable timer
	TIM2->CR1 |= TIM_CR1_CEN;
}

//------------------------------------------------------------------------------------

//----------------------------------- ADC STUFF --------------------------------------

/*
 * @brief gets result from ADC
 */
void ADC1_2_IRQHandler(void) {
	if (ADC1->ISR & ADC_ISR_EOC){
		// Pull value from ADC and store it
		samples[idx++] = ADC1->DR;
		// reset when full
		if (idx == SAMPLE_SIZE) {
			idx = 0;
			get_avg_flag = 1;
		}
		ADC1->ISR &= ~ADC_ISR_EOC;
	}

}

/*
 * @brief Initializes ADC
 */
void ADC_init(void) {
	// Configure ADC clock
	  RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;
	  // run on system clock
	  ADC123_COMMON->CCR = (1<< ADC_CCR_CKMODE_Pos);

	  // Power up ADC and voltage regulator
	  ADC1->CR &= ~(ADC_CR_DEEPPWD);
	  ADC1->CR |= ADC_CR_ADVREGEN;
	  // wait 20 us
	  for (uint32_t i = 0; i<50000; i++);

	  // configure channel 5 for single ended mode
	  // Using channel 5 (PA0)
	  ADC1->DIFSEL &= ~ADC_DIFSEL_DIFSEL_5;

	  // Calibrate ADC
	  // ensure ADC disabled and single ended calibration
	  ADC1->CR &= ~(ADC_CR_ADEN | ADC_CR_ADCALDIF);
	  // start calibration
	  ADC1->CR |= ADC_CR_ADCAL;
	  // wait for calibration to finish
	  while (ADC1->CR & ADC_CR_ADCAL);

	  // enable ADC
	  // clear ADC Ready with a 1
	  ADC1->ISR |= (ADC_ISR_ADRDY);
	  ADC1->CR |= ADC_CR_ADEN;
	  while (!(ADC1->ISR & ADC_ISR_ADRDY));

	  // Configure sequence
	  // single channel 5, conversion size of 1
	  ADC1->SQR1 = (5 << ADC_SQR1_SQ1_Pos);

	  // Set ADC sample time for channel 5 to 2.5 ADC clock cycles
	  ADC1->SMPR1 &= ~ADC_SMPR1_SMP5;
	  ADC1->SMPR1 |= (7 << ADC_SMPR1_SMP5_Pos);

	  // configure 12 bit resolution, right align, single conversion mode
	  ADC1->CFGR = 0;

	  // Configure interrupts
	  ADC1->IER |= ADC_IER_EOCIE; //enable EOC interrupts
	  NVIC->ISER[0] |= (1 << (ADC1_2_IRQn & 0x1F));

	  // Configure GPIO for PA0
	  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; // enable pin clock
	  GPIOA->MODER &= ~(GPIO_MODER_MODE0);
	  GPIOA->MODER |= (GPIO_MODER_MODE0);
	  GPIOA->ASCR |= GPIO_ASCR_ASC0;

	  // start a conversion
	  ADC1->CR |= ADC_CR_ADSTART;
}
//-------------------------------------------------------------------------------------


//----------------------------------- UART STUFF---------------------------------------

/*
 * @brief Auto prints ESC along with custom code
 */
void USART_ESC_Code(char* data) {
   while (!(USART2->ISR & USART_ISR_TXE));
   USART2->TDR = '\x1B';  // esc char
   while (*data) {
          while (!(USART2->ISR & USART_ISR_TXE));
          USART2->TDR = *data++;  // Send character and get the next one too
      }
}

/*
 * @brief Prints custom message to terminal
 */
void UART_print(char *data) {
	// wait for TXE then write character and advanve till null
    while (*data) {
        while (!(USART2->ISR & USART_ISR_TXE));
        USART2->TDR = *data++;
    }
}


 //initializing USART2
void UART_init(void) {
	// enable GPIOA and USART clock
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	// AF7 configuration for PA2 and PA3
	GPIOA->AFR[0] &= ~((0xF << GPIO_AFRL_AFSEL2_Pos)|(0xF << GPIO_AFRL_AFSEL3_Pos)); // clear
	GPIOA->AFR[0] |= ((0x7 << GPIO_AFRL_AFSEL2_Pos)|(0x7 << GPIO_AFRL_AFSEL3_Pos)); // set
	//turn PA2 and PA3 on
	GPIOA->MODER &= ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE3);
	GPIOA->MODER |= (2 << GPIO_MODER_MODE2_Pos) | (2 << GPIO_MODER_MODE3_Pos); // Set to AF mode (0b10)
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT2 | GPIO_OTYPER_OT3);
	GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED2 | GPIO_OSPEEDR_OSPEED3);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD2 | GPIO_PUPDR_PUPD3);

	// define word length, 8 bits
	USART2->CR1 &= ~(USART_CR1_M0 | USART_CR1_M1);
	// set baud rate
	USART2->BRR = BRR_num;
	// 1 stop bit
	USART2->CR2 &= ~(USART_CR2_STOP_1 | USART_CR2_STOP_0);
	// enable transmit and receive
	USART2->CR1 |= (USART_CR1_RE | USART_CR1_TE);
	// enable RX interrupts
	USART2->CR1 |= (USART_CR1_RXNEIE);
	//enable USART and interrupt
	USART2->CR1 |= USART_CR1_UE;
	NVIC->ISER[1] |= (1 << (USART2_IRQn & 0x1F));
}
//---------------------------------------------------------------------------------------

/**
  * @brief System Clock Configuration
  * @retval None
  * 80Mhz Clock
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
