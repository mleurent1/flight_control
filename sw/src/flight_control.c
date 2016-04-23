#include "stm32f103xb.h"

volatile uint32_t msTicks;

void SysTick_Handler(void)
{
	msTicks++;
}

/*
// System Interrupts
void NMI_Handler       (void) {}
void HardFault_Handler (void) {}
void MemManage_Handler (void) {}
void BusFault_Handler  (void) {}
void UsageFault_Handler(void) {}
void SVC_Handler       (void) {}
void DebugMon_Handler  (void) {}
void PendSV_Handler    (void) {}

// External Interrupts
void WWDG_IRQHandler           (void) {}
void PVD_IRQHandler            (void) {}
void TAMPER_IRQHandler         (void) {}
void RTC_IRQHandler            (void) {}
void FLASH_IRQHandler          (void) {}
void RCC_IRQHandler            (void) {}
void EXTI0_IRQHandler          (void) {}
void EXTI1_IRQHandler          (void) {}
void EXTI2_IRQHandler          (void) {}
void EXTI3_IRQHandler          (void) {}
void EXTI4_IRQHandler          (void) {}
void DMA1_Channel1_IRQHandler  (void) {}
void DMA1_Channel2_IRQHandler  (void) {}
void DMA1_Channel3_IRQHandler  (void) {}
void DMA1_Channel4_IRQHandler  (void) {}
void DMA1_Channel5_IRQHandler  (void) {}
void DMA1_Channel6_IRQHandler  (void) {}
void DMA1_Channel7_IRQHandler  (void) {}
void ADC1_2_IRQHandler         (void) {}
void USB_HP_CAN1_TX_IRQHandler (void) {}
void USB_LP_CAN1_RX0_IRQHandler(void) {}
void CAN1_RX1_IRQHandler       (void) {}
void CAN1_SCE_IRQHandler       (void) {}
void EXTI9_5_IRQHandler        (void) {}
void TIM1_BRK_IRQHandler       (void) {}
void TIM1_UP_IRQHandler        (void) {}
void TIM1_TRG_COM_IRQHandler   (void) {}
void TIM1_CC_IRQHandler        (void) {}
void TIM2_IRQHandler           (void) {}
void TIM3_IRQHandler           (void) {}
void TIM4_IRQHandler           (void) {}
void I2C1_EV_IRQHandler        (void) {}
void I2C1_ER_IRQHandler        (void) {}
void I2C2_EV_IRQHandler        (void) {}
void I2C2_ER_IRQHandler        (void) {}
void SPI1_IRQHandler           (void) {}
void SPI2_IRQHandler           (void) {}
void USART1_IRQHandler         (void) {}
void USART2_IRQHandler         (void) {}
void USART3_IRQHandler         (void) {}
void EXTI15_10_IRQHandler      (void) {}
void RTC_Alarm_IRQHandler      (void) {}
void USBWakeUp_IRQHandler      (void) {}
*/

void SpiWrite(uint8_t Addr, uint8_t Data)
{
	SPI1->CR1 |= SPI_CR1_SPE;
	SPI1->DR = (uint32_t)Addr;
	while ((SPI1->SR & SPI_SR_TXE) == 0) {}
	SPI1->DR = (uint32_t)Data;
	while ((SPI1->SR & SPI_SR_TXE) == 0) {}
	while ((SPI1->SR & SPI_SR_BSY) == 1) {}
	SPI1->CR1 &= ~SPI_CR1_SPE;
}

void SpiRead(uint8_t Addr, uint8_t * Data)
{
	SPI1->CR1 |= SPI_CR1_SPE;
	SPI1->DR = (uint32_t)(Addr | 0x80);
	while ((SPI1->SR & SPI_SR_TXE) == 0) {}
	SPI1->DR = 0;
	while ((SPI1->SR & SPI_SR_RXNE) == 0) {}
	*Data = (uint8_t)SPI1->DR;
	while ((SPI1->SR & SPI_SR_RXNE) == 0) {}
	*Data = (uint8_t)SPI1->DR;
	while ((SPI1->SR & SPI_SR_BSY) == 1) {}
	SPI1->CR1 &= ~SPI_CR1_SPE;
}

void WaitForTick(void)
{
	uint32_t curTicks;
	curTicks = msTicks; // Save Current SysTick Value
	while (msTicks == curTicks) // Wait for next SysTick Interrupt
		__WFE (); // Power-Down until next Event/Interrupt
}

int main(void)
{
	uint8_t SpiData;
	
	//####### INTERRUPT #######
	
	NVIC_SetPriority(TIM1_UP_IRQn,1); // Set Timer priority
	NVIC_EnableIRQ(TIM1_UP_IRQn); // Enable Timer Interrupt
	
	//######## CLOCK ##########
	
	RCC->APB2ENR = RCC_APB2ENR_SPI1EN | RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;
	
	SystemCoreClockUpdate();
	
	//####### GPIO ##########
	
	/***************
	INPUT:
	MODE = 0 | CNF_1: pull up(ODR=1)/down(ODR=0) | CNF_1 = 0 +  CNF_0: floating (otherwise analog)
	OUTPUT:
	CNF_1: Alt. func. | CNF_0: Open-drain (otherwise push-pull) | MODE: 50MHz | MODE_0: 10MHz | MODE_1: 2MHz
  *****************/
	
	// PB3 = blue LED, PB2 = inv UART Rx
	AFIO->MAPR = AFIO_MAPR_SWJ_CFG_JTAGDISABLE; // Free PB3
	GPIOB->CRL &= 0xFFFF00FF;
	GPIOB->CRL |= GPIO_CRL_CNF3_0 | GPIO_CRL_MODE3_1 | 
	                                GPIO_CRL_MODE2_1;
	GPIOB->BSRR = GPIO_BSRR_BR2; // Set PB2 to 0 => no inv UART Rx
	
	// PA9 = UART1 Tx, PA10 = UART1 Rx
	GPIOA->CRH &= 0xFFFFF00F;
	GPIOA->CRH |= GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9_0 | 
	              GPIO_CRH_CNF10_0;
	
	// PA4/5/6/7 = SPI CSN/SCK/MISO/MOSI
	GPIOA->CRL &= 0x0000FFFF;
	GPIOA->CRL |= GPIO_CRL_CNF4   | GPIO_CRL_MODE4 | 
	              GPIO_CRL_CNF5_1 | GPIO_CRL_MODE5 | 
	              GPIO_CRL_CNF6_0 | 
	              GPIO_CRL_CNF7_1 | GPIO_CRL_MODE7;
	
	// PB9/8/7, PA8 = SERVO 1/2/3/4
	GPIOB->CRH &= 0xFFFFFF00;
	GPIOB->CRH |= GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9 |
	              GPIO_CRH_CNF8_1 | GPIO_CRH_MODE8;
	GPIOB->CRL &= 0x0FFFFFFF;
	GPIOB->CRL |= GPIO_CRL_CNF7_1 | GPIO_CRL_MODE7;
	GPIOA->CRH &= 0xFFFFFFF0;
	GPIOA->CRH |= GPIO_CRH_CNF8_1 | GPIO_CRH_MODE8;
	
	// PB6/5/0/1 = COMMAND 1/2/3/4
	GPIOB->CRL &= 0xF00FFF00;
	GPIOB->CRL |= GPIO_CRL_CNF6_0 |
	              GPIO_CRL_CNF5_0 |
								GPIO_CRL_CNF0_0 |
								GPIO_CRL_CNF1_0;
	
	//######## UART ##########
	
	USART1->BRR = (1 & USART_BRR_DIV_Fraction) | ((52<<4) & USART_BRR_DIV_Mantissa); // 9600 bps
	USART1->CR1 = USART_CR1_UE | USART_CR1_TE;
	
	//####### SPI #########
	
	SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_BR_0 | SPI_CR1_BR_1;
	SPI1->CR2 = SPI_CR2_SSOE;
	
	//####### TIM ########
	
	
	
	//####### MPU6000 INIT #########
	/*
	SpiWrite(0x6B, 0x80); // Reset MPU
	WaitForTick();
	SpiWrite(0x68, 0x07); // Reset signal path
	*/
	//######## MAIN LOOP #########
	
	SysTick_Config(SystemCoreClock/4); // 250 ms
	
	while (1)
	{
		WaitForTick();
		GPIOB->BSRR = GPIO_BSRR_BR3;
		WaitForTick();
		GPIOB->BSRR = GPIO_BSRR_BS3;
		
		SpiRead(0x75, &SpiData);
		
		USART1->DR = (uint32_t)SpiData;
	}
}

void Reset_Handler(void)
{
	main();
}
