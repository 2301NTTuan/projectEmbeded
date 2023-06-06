#include "main.h"
#include "fonts.h"
#include "ssd1306.h"
#include "math.h"
#include "mpu6050.h"
#include <stdio.h>

//KHAI BAO CAC BIEN TOAN CUC
I2C_HandleTypeDef hi2c1;
MPU6050_t MPU6050;
float accelMag, prev, deltaMag;
char BUFF[16];
volatile uint32_t systickDelay;
uint16_t countStep;
uint8_t mark, temp;
//

//KHAI BAO CAC HAM CON
void SysTick_Init(void);
void SystemClock_Config(void);
void GPIO_Init(void);
void I2C1_Init(void);
void EXTI_Init(void);
uint16_t calcData(MPU6050_t accelData , float accelMag, float prev, uint16_t countStep);
void delayMs(uint32_t ms);
void printfScreen(void);
//

//HAM MAIN
int main(void){
	
  HAL_Init(); 
	SystemClock_Config();
	SysTick_Init();
  GPIO_Init();
  I2C1_Init();
	MPU6050_Init(&hi2c1);
	SSD1306_Init();
	__enable_irq();
	mark = 1;
	countStep = 0;
	temp = 0;
  while (1){
		MPU6050_Read_Gyro(&hi2c1, &MPU6050);
		countStep = calcData(MPU6050, accelMag, prev, countStep);
		
		//Sw STOP
		while(mark == 0){
			GPIOC->BSRR |= GPIO_PIN_13;
			GPIOA->BSRR = GPIO_BSRR_BS2;
			SSD1306_GotoXY(47,32);
			sprintf(BUFF, "STOP");
			SSD1306_Puts(BUFF, &Font_7x10, SSD1306_COLOR_WHITE);
			SSD1306_UpdateScreen();
			delayMs(1000);
			if(temp == 1){
				mark = 1;
			}
		}
		GPIOA->BRR = GPIO_BRR_BR2;
		//Sw RESET
		if(temp == 1){
			countStep = 0;
			SSD1306_GotoXY(46,32);
			sprintf(BUFF, "RESET");
			SSD1306_Puts(BUFF, &Font_7x10, SSD1306_COLOR_WHITE);
			SSD1306_UpdateScreen();
			delayMs(1000);
			SSD1306_Clear();
			temp = 0;
			}
		//CAP NHAT BUOC CHAN LEN MAN HINH
		printfScreen();
		//NHAY DEN LED XANH 
		GPIOC->ODR ^= (1 << 13);
		delayMs(1000);
  }
}
//

//HAM CON
void SysTick_Handler(void){
    if (systickDelay > 0){
        systickDelay--;
    }
}

void delayMs(uint32_t ms){
    systickDelay = ms;
    // Thiet lap Systick timer voi chu ki ms
    SysTick_Config(SystemCoreClock / 1000);
    while (systickDelay > 0){}
    // Tat Systick timer
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

uint16_t calcData(MPU6050_t accelData , float accelMag, float prev, uint16_t countStep){
	
		accelMag = sqrt(accelData.Gx*accelData.Gx + accelData.Gy*accelData.Gy + accelData.Gz*accelData.Gz);
		deltaMag = accelMag - prev;
		deltaMag = sqrt(deltaMag*deltaMag);
		delayMs(100);
		if( deltaMag >= 20 && deltaMag < 80 ){
				countStep ++;
		}
		prev = accelMag;
		return countStep;
	}
void SysTick_Init(void){
  // Cau hinh gia tri Reload
  SysTick->LOAD = SystemCoreClock / 2 - 1; // Tan so SysTick 1Hz
  // Cau hình thanh ghi Control
  SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk; 
  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk; // Cho phep ngat SysTick
  SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; // Bat SysTick Timer
}

//Read GPIO
uint16_t GPIO_ReadPin(GPIO_TypeDef *GPIO, uint16_t GPIO_Pin){
  uint16_t Status;
  if ((GPIO->IDR & GPIO_Pin) != 0){
    Status = 1;
  }
  else{
    Status = 0;
  }
  return Status;
}
void EXTI_Init(void){
  // Enable AFIO Clock
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
  
  // Cau hinh ngat cho sw1, sw2
  EXTI->IMR |= EXTI_IMR_MR0 | EXTI_IMR_MR1; // Enable interrupt cho sw1, sw2
  EXTI->FTSR |= EXTI_FTSR_TR0 | EXTI_FTSR_TR1; // Set falling edge trigger cho sw1, sw2
  EXTI->PR |= EXTI_PR_PR0 | EXTI_PR_PR1; // Clear pending interrupt cho sw1, sw2
  
  // Configure NVIC cho sw1, sw2
  NVIC_SetPriority(EXTI0_IRQn, 0); // Set priority cho sw1
  NVIC_EnableIRQ(EXTI0_IRQn); // Enable IRQ cho sw1
  NVIC_SetPriority(EXTI1_IRQn, 0); // Set priority cho sw2
  NVIC_EnableIRQ(EXTI1_IRQn); // Enable IRQ cho sw2
  
  // Connect sw1, sw2 to GPIOA
  AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI0_PA; // Connect sw1 to GPIOA
  AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI1_PA; // Connect sw2 to GPIOA
}

void EXTI0_IRQHandler(void){
  if (EXTI->PR & EXTI_PR_PR0){// Check if triggered the interrupt
		if(GPIO_ReadPin(GPIOA,GPIO_PIN_0) == 0 && mark == 1){
			mark = 0;
			SSD1306_Clear();
		}
		else if(GPIO_ReadPin(GPIOA,GPIO_PIN_0) == 0 && mark == 0){
			mark = 1;
			SSD1306_Clear();
		}
		//delayMs(100);
		EXTI->PR |= EXTI_PR_PR0; // Clear pending interrupt 
  }
}

void EXTI1_IRQHandler(void){
  if (EXTI->PR & EXTI_PR_PR1){ // Check if triggered the interrupt
    if(GPIO_ReadPin(GPIOA,GPIO_PIN_1) == 0){
			temp = 1;
			SSD1306_Clear();
		}
    EXTI->PR |= EXTI_PR_PR1; // Clear pending interrupt 
  }
}

void GPIO_Init(void){

	//PORT C
  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;// Cap xung clock cho port C
  GPIOC->CRH &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13); // Clear chan PC13
  GPIOC->CRH |= GPIO_CRH_MODE13_0; // Dat chan PC13 la dau ra

	//PORT A
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; // Cap xung clock cho port A
  GPIOA->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0 | GPIO_CRL_MODE1 | GPIO_CRL_CNF1 | GPIO_CRL_MODE2 | GPIO_CRL_CNF2); // Clear chan A0, A1, A2
	GPIOA->CRL |= GPIO_CRL_MODE2_0 ; // Dat chan A2 la dau ra
	GPIOA->CRL |= GPIO_CRL_CNF0_1 | GPIO_CRL_CNF1_1; // Dat chan A0, A1 la dau vao
  // Set initial output state for PA0 and PA1
  GPIOA->BSRR = GPIO_BSRR_BS0 | GPIO_BSRR_BS1; // Set pins PA0 and PA1 high

	EXTI_Init();
}
//I2C_INIT
void I2C1_Init(void){
	hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK){
    Error_Handler();
  }
}
void printfScreen(void){
		SSD1306_GotoXY(37,32);
		sprintf(BUFF, "STEPS:= %d", countStep);
		SSD1306_Puts(BUFF, &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_UpdateScreen();
}
void SystemClock_Config(){
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
    Error_Handler();
  }
	
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK){
    Error_Handler();
  }
}
void Error_Handler(void){
  __disable_irq();
  while (1)
  {
  }
}
//