#include <string.h>
#include "stm32f103xb.h"
#include "flight_control.h"

volatile uint32_t tick;
volatile uint8_t sensor_sample_count;
volatile uint8_t uart_rx_buffer[16];
volatile uint8_t uart_tx_buffer[16];
volatile uint8_t spi_rx_buffer[16];
volatile uint8_t spi_tx_buffer[16];
volatile int16_t gyro_x;
volatile int16_t gyro_y;
volatile int16_t gyro_z;
volatile int16_t accel_x;
volatile int16_t accel_y;
volatile int16_t accel_z;

void SetDmaForUartRx(uint8_t size)
{
	DMA1_Channel5->CCR = DMA_CCR_TCIE | DMA_CCR_MINC;
	DMA1_Channel5->CMAR = (uint32_t)uart_rx_buffer;
	DMA1_Channel5->CPAR = (uint32_t)&(USART1->DR);
	DMA1_Channel5->CNDTR = size;
	DMA1_Channel5->CCR |= DMA_CCR_EN;
}

void SetDmaForUartTx(uint8_t size)
{
	USART1->CR3 |= USART_CR3_DMAT;
	
	DMA1_Channel4->CCR = DMA_CCR_TCIE | DMA_CCR_MINC | DMA_CCR_DIR;
	DMA1_Channel4->CMAR = (uint32_t)uart_tx_buffer;
	DMA1_Channel4->CPAR = (uint32_t)&(USART1->DR);
	DMA1_Channel4->CNDTR = size;
	DMA1_Channel4->CCR |= DMA_CCR_EN;
}

void SetDmaForSpi(uint8_t size)
{
	DMA1_Channel2->CCR = DMA_CCR_TCIE | DMA_CCR_MINC;
	DMA1_Channel2->CMAR = (uint32_t)spi_rx_buffer;
	DMA1_Channel2->CPAR = (uint32_t)&(SPI1->DR);
	DMA1_Channel2->CNDTR = size;
	
	DMA1_Channel3->CCR = DMA_CCR_MINC | DMA_CCR_DIR;
	DMA1_Channel3->CMAR = (uint32_t)spi_tx_buffer;
	DMA1_Channel3->CPAR = (uint32_t)&(SPI1->DR);
	DMA1_Channel3->CNDTR = size;
	
	SPI1->CR2 |= SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;
	DMA1_Channel2->CCR |= DMA_CCR_EN;
	DMA1_Channel3->CCR |= DMA_CCR_EN;
	SPI1->CR1 |= SPI_CR1_SPE;
}

void UartSend(uint8_t * data, uint8_t size)
{
	int i;
	while ((USART1->SR & USART_SR_TC) == 0) {} // Wait for end of current transaction
	for(i=0; i<size; i++)
	{
		while ((USART1->SR & USART_SR_TXE) == 0) {} 
		USART1->DR = data[i];
	}
	while ((USART1->SR & USART_SR_TC) == 0) {}
}

void Spi(uint8_t addr, uint8_t din, uint8_t * dout)
{
	// Abort current transaction
	SPI1->CR1 &= ~SPI_CR1_SPE;
	SPI1->CR2 &= ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
	
	// SPI transaction
	SPI1->CR1 |= SPI_CR1_SPE;
	SPI1->DR = addr;
	while ((SPI1->SR & SPI_SR_TXE) == 0) {}
	SPI1->DR = din;
	while ((SPI1->SR & SPI_SR_RXNE) == 0) {}
	*dout = SPI1->DR;
	while ((SPI1->SR & SPI_SR_RXNE) == 0) {}
	*dout = SPI1->DR;
	while ((SPI1->SR & SPI_SR_TXE) == 0) {}
	while (SPI1->SR & SPI_SR_BSY) {}
	SPI1->CR1 &= ~SPI_CR1_SPE;
}

void SpiWrite(uint8_t addr, uint8_t data)
{
	uint8_t datar;
	Spi(addr, data, &datar);
}

void SpiRead(uint8_t addr, uint8_t * data)
{
	Spi(addr|0x80, 0, data);
}

void Wait(uint32_t ticks)
{
	uint32_t current_tick;
	uint32_t next_tick;
	current_tick = tick;
	next_tick = current_tick + ticks;
	while (tick != next_tick)
		__WFE();
}

//######## System Interrupts ########
/*
void NMI_Handler       (void) {}
void HardFault_Handler (void) {}
void MemManage_Handler (void) {}
void BusFault_Handler  (void) {}
void UsageFault_Handler(void) {}
void SVC_Handler       (void) {}
void DebugMon_Handler  (void) {}
void PendSV_Handler    (void) {}
*/
void SysTick_Handler()
{
	tick++;
}

//####### External Interrupts ########
/*
void WWDG_IRQHandler           (void) {}
void PVD_IRQHandler            (void) {}
void TAMPER_IRQHandler         (void) {}
void RTC_IRQHandler            (void) {}
void FLASH_IRQHandler          (void) {}
void RCC_IRQHandler            (void) {}
void EXTI0_IRQHandler          (void) {}
void EXTI1_IRQHandler          (void) {}
void EXTI2_IRQHandler          (void) {}
*/
void EXTI3_IRQHandler()
{
	int i;
	
	sensor_sample_count++;
	if (sensor_sample_count == 0)
		GPIOB->ODR ^= GPIO_ODR_ODR3;
	
	accel_x = ((int16_t)spi_rx_buffer[0]  << 8) + (int16_t)spi_rx_buffer[1];
	accel_y = ((int16_t)spi_rx_buffer[2]  << 8) + (int16_t)spi_rx_buffer[3];
	accel_z = ((int16_t)spi_rx_buffer[4]  << 8) + (int16_t)spi_rx_buffer[5];
	gyro_x  = ((int16_t)spi_rx_buffer[8]  << 8) + (int16_t)spi_rx_buffer[9];
	gyro_y  = ((int16_t)spi_rx_buffer[10] << 8) + (int16_t)spi_rx_buffer[11];
	gyro_z  = ((int16_t)spi_rx_buffer[12] << 8) + (int16_t)spi_rx_buffer[13];
	
	if (reg[DEBUG_MUX] == 1)
	{
		uart_tx_buffer[0] = sensor_sample_count;
		for(i=0; i<6; i++)
		{
			uart_tx_buffer[1+i] = spi_rx_buffer[1+i];
			uart_tx_buffer[7+i] = spi_rx_buffer[9+i];
		}
		
		spi_tx_buffer[0] = 59 | 0x80;
		SetDmaForSpi(15);
	}
	
	EXTI->PR = EXTI_PR_PIF3; // Clear pending request
}
/*
void EXTI4_IRQHandler          (void) {}
void DMA1_Channel1_IRQHandler  (void) {}
*/
void DMA1_Channel2_IRQHandler()
{
	while ((SPI1->SR & SPI_SR_TXE) == 0) {}
	while (SPI1->SR & SPI_SR_BSY) {}
	SPI1->CR1 &= ~SPI_CR1_SPE; // Disable SPI
	SPI1->CR2 &= ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
	
	DMA1->IFCR = DMA_IFCR_CTCIF2 | DMA_IFCR_CTCIF3; // clear flag
	DMA1->IFCR = 0;
}
/*
void DMA1_Channel3_IRQHandler  (void) {}
*/
void DMA1_Channel4_IRQHandler()
{
	USART1->CR3 &= ~USART_CR3_DMAT;
	
	DMA1->IFCR = DMA_IFCR_CTCIF4; // clear flag
	DMA1->IFCR = 0;
}

void DMA1_Channel5_IRQHandler()
{
	uint8_t addr;
	uint8_t spi_data;
	uint8_t uart_data[2];
	
	addr = uart_rx_buffer[1];
	switch(uart_rx_buffer[0])
	{
		case 0:
			uart_data[0] = (uint8_t)(reg[addr] >> 8);
			uart_data[1] = (uint8_t)(reg[addr] & 0xFF);
			UartSend(uart_data, 2);
			break;
		case 1:
			reg[addr] = ((uint16_t)uart_rx_buffer[2] << 8) + (uint16_t)uart_rx_buffer[3];
			break;
		case 2:
			SpiRead(addr, &spi_data);
			UartSend(&spi_data, 1);
			break;
		case 3:
			SpiWrite(addr, uart_rx_buffer[3]);
			break;
		case 4:
			if (reg[DEBUG_MUX] == 1)
				SetDmaForUartTx(13);
			break;
	}
	
	SetDmaForUartRx(4);
	
	DMA1->IFCR = DMA_IFCR_CTCIF5; // clear flag
	DMA1->IFCR = 0;
}
/*
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

//######## MAIN #########

int main()
{
	int i;
	
	//######## CLOCK ##########
	
	RCC->APB2ENR = RCC_APB2ENR_SPI1EN | RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/1000); // 1 ms
	
	//####### REG INIT #######
	
	for(i=0; i<REG_NB; i++)
		reg[i] = reg_init[i];
	
	//####### GPIO ##########
	
	// PB3 = blue LED, PB2 = inv UART Rx
	AFIO->MAPR = AFIO_MAPR_SWJ_CFG_JTAGDISABLE; // Free PB3
	GPIOB->CRL &= 0xFFFF00FF;
	GPIOB->CRL |= GPIO_CRL_CNF3_0 | GPIO_CRL_MODE3_1 | 
	                                GPIO_CRL_MODE2_1;
	GPIOB->BSRR = GPIO_BSRR_BR2; // Set PB2 to 0 => no inv UART Rx
	
	// PA9 = UART1 Tx, PA10 = UART1 Rx
	GPIOA->CRH &= 0xFFFFF00F;
	GPIOA->CRH |= GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9 | 
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
	
	//####### INTERRUPT #######
	
	AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI3_PA;
	EXTI->RTSR |= EXTI_RTSR_RT3;
	EXTI->IMR = EXTI_IMR_IM3;
	
	NVIC_EnableIRQ(EXTI3_IRQn);
	NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	NVIC_EnableIRQ(DMA1_Channel5_IRQn);
	
	NVIC_SetPriority(EXTI3_IRQn,1);
	NVIC_SetPriority(DMA1_Channel2_IRQn,2);
	NVIC_SetPriority(DMA1_Channel5_IRQn,3);
	NVIC_SetPriority(DMA1_Channel4_IRQn,4);
	
	//######## UART ##########
	
	//USART1->BRR = (1 & USART_BRR_DIV_Fraction) | ((52<<4) & USART_BRR_DIV_Mantissa); // 9600 bps
	USART1->BRR = (5 & USART_BRR_DIV_Fraction) | ((4<<4) & USART_BRR_DIV_Mantissa); // 115200 bps
	USART1->CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;
	USART1->CR3 = USART_CR3_DMAR;
	SetDmaForUartRx(4);
	
	//####### SPI #########
	
	SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_BR_0 | SPI_CR1_BR_1;
	SPI1->CR2 = SPI_CR2_SSOE;
	
	//####### TIM ########
	
	//####### MPU INIT #########
	
	SpiWrite(107, 0x80); // Reset MPU
	Wait(100);
	SpiWrite(107, 0); //Get MPU out of sleep
	Wait(100);
	SpiWrite(106, 16); // Disable I2C
	//SpiWrite(104, 7); // Reset signal path
	//Wait(100);
	SpiWrite(26, 1); // Filter ON (=> Fs=1kHz)
	SpiWrite(56, 1); // Enable interrupt
	
	//######## MAIN LOOP #########
	
	while (1)
	{
		/*
		if (reg[LED])
			GPIOB->BSRR = GPIO_BSRR_BR3;
		else
			GPIOB->BSRR = GPIO_BSRR_BS3;
		*/
		__WFE();
	}
}

void Reset_Handler()
{
	main();
}
