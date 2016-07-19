#include "stm32f103xb.h"
#include "fc_reg.h"
#include "mpu_reg.h"

#define SERVO_MIN 1806
#define FLAG_SENSOR 0x01
#define FLAG_COMMAND 0x02
#define FLAG_SERVO 0x04

volatile uint8_t FLAG;

volatile uint32_t tick;

volatile uint8_t uart_rx_buffer[16];
volatile uint8_t uart_tx_buffer[16];
volatile uint8_t spi_rx_buffer[16];
volatile uint8_t spi_tx_buffer[16];
volatile uint8_t spi_byte_count;
volatile uint8_t uart3_rx_buffer[16];

volatile uint16_t sensor_sample_count;
volatile int16_t gyro_x_raw;
volatile int16_t gyro_y_raw;
volatile int16_t gyro_z_raw;
volatile int16_t accel_x_raw;
volatile int16_t accel_y_raw;
volatile int16_t accel_z_raw;
//volatile int16_t temp_raw;

volatile uint8_t command_frame_count;
//volatile uint8_t command_fades;
//volatile uint8_t command_system;
volatile uint16_t throttle_raw;
volatile uint16_t aileron_raw;
volatile uint16_t elevator_raw;
volatile uint16_t rudder_raw;
volatile uint16_t armed_raw;

volatile uint8_t servo_count;

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
	DMA1_Channel2->CCR |= DMA_CCR_EN;
	
	SPI1->CR2 |= SPI_CR2_TXEIE | SPI_CR2_RXDMAEN;
	SPI1->CR1 |= SPI_CR1_SPE;
}

void SetDmaForUart3Rx(uint8_t size)
{
	DMA1_Channel3->CCR = DMA_CCR_TCIE | DMA_CCR_MINC;
	DMA1_Channel3->CMAR = (uint32_t)uart3_rx_buffer;
	DMA1_Channel3->CPAR = (uint32_t)&(USART3->DR);
	DMA1_Channel3->CNDTR = size;
	DMA1_Channel3->CCR |= DMA_CCR_EN;
}

void UartSend(uint8_t * data, uint8_t size)
{
	int i;
	while ((USART1->SR & USART_SR_TC) == 0) {} // Wait for end of current transaction
	for (i=0; i<size; i++)
	{
		while ((USART1->SR & USART_SR_TXE) == 0) {} 
		USART1->DR = data[i];
	}
	while ((USART1->SR & USART_SR_TC) == 0) {}
}

void Spi(uint8_t addr, uint8_t din, uint8_t * dout)
{
	spi_byte_count = 2;
	spi_tx_buffer[1] = addr;
	spi_tx_buffer[0] = din;
	SPI1->CR2 |= SPI_CR2_TXEIE;
	SPI1->CR1 |= SPI_CR1_SPE;
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
		__WFI();
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
	sensor_sample_count++;
	
	if (REG_SPI_HOST_CTRL == 0)
	{
		accel_x_raw = ((int16_t)spi_rx_buffer[0]  << 8) | (int16_t)spi_rx_buffer[1];
		accel_y_raw = ((int16_t)spi_rx_buffer[2]  << 8) | (int16_t)spi_rx_buffer[3];
		accel_z_raw = ((int16_t)spi_rx_buffer[4]  << 8) | (int16_t)spi_rx_buffer[5];
	  //temp_raw    = ((int16_t)spi_rx_buffer[6]  << 8) | (int16_t)spi_rx_buffer[7];
		gyro_x_raw  = ((int16_t)spi_rx_buffer[8]  << 8) | (int16_t)spi_rx_buffer[9];
		gyro_y_raw  = ((int16_t)spi_rx_buffer[10] << 8) | (int16_t)spi_rx_buffer[11];
		gyro_z_raw  = ((int16_t)spi_rx_buffer[12] << 8) | (int16_t)spi_rx_buffer[13];
		
		FLAG |= FLAG_SENSOR;
		
		spi_byte_count = 15;
		spi_tx_buffer[14] = 59 | 0x80;
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
	SPI1->CR2 &= ~SPI_CR2_RXDMAEN;
	
	DMA1->IFCR = DMA_IFCR_CTCIF2; // clear flag
	DMA1->IFCR = 0;
}

void DMA1_Channel3_IRQHandler()
{
	int i;
	uint8_t chan_id;
	uint16_t servo;
	
	command_frame_count++;
	
	//command_fades = uart3_rx_buffer[0];
	//command_system = uart3_rx_buffer[1];
	for (i=0; i<7; i++)
	{
		servo = ((uint16_t)uart3_rx_buffer[2+2*i] << 8) | (uint16_t)uart3_rx_buffer[3+2*i];
		chan_id = (uint8_t)((servo & 0x7800) >> 11);
		switch(chan_id)
		{
			case 0:
				throttle_raw = servo & 0x07FF;
				break;
			case 1:
				aileron_raw = servo & 0x07FF;
				break;
			case 2:
				elevator_raw = servo & 0x07FF;
				break;
			case 3:
				rudder_raw = servo & 0x07FF;
				break;
			case 4:
				armed_raw = servo & 0x07FF;
				break;
		}
	}
	
	FLAG |= FLAG_COMMAND;
	
	DMA1->IFCR = DMA_IFCR_CTCIF3; // clear flag
	DMA1->IFCR = 0;
	
	SetDmaForUart3Rx(16);
}

void DMA1_Channel4_IRQHandler()
{
	USART1->CR3 &= ~USART_CR3_DMAT;
	
	DMA1->IFCR = DMA_IFCR_CTCIF4; // clear flag
	DMA1->IFCR = 0;
}

void DMA1_Channel5_IRQHandler()
{
	uint8_t cmd;
	uint8_t addr;
	uint8_t spi_data;
	uint8_t uart_data[2];
	
	cmd = uart_rx_buffer[0];
	addr = uart_rx_buffer[1];
	
	switch(cmd)
	{
		case 0:
			uart_data[0] = (uint8_t)(reg[addr] >> 8);
			uart_data[1] = (uint8_t)(reg[addr] & 0xFF);
			UartSend(uart_data, 2);
			break;
		case 1:
			reg[addr] = ((uint16_t)uart_rx_buffer[2] << 8) | (uint16_t)uart_rx_buffer[3];
			break;
		case 2:
			SpiRead(addr, &spi_data);
			UartSend(&spi_data, 1);
			break;
		case 3:
			SpiWrite(addr, uart_rx_buffer[3]);
			break;
		case 4:
			if (REG_DEBUG_MUX == 1)
				SetDmaForUartTx(13);
			else if (REG_DEBUG_MUX == 2)
				SetDmaForUartTx(11);
			else if (REG_DEBUG_MUX == 3)
				SetDmaForUartTx(7);
			else if (REG_DEBUG_MUX == 4)
				SetDmaForUartTx(9);
			break;
	}
	
	DMA1->IFCR = DMA_IFCR_CTCIF5; // clear flag
	DMA1->IFCR = 0;
	
	SetDmaForUartRx(4);
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
*/
void TIM4_IRQHandler()
{
	servo_count++;
	
	FLAG |= FLAG_SERVO;
	
	TIM4->SR &= ~TIM_SR_CC4IF;
}
/*
void I2C1_EV_IRQHandler        (void) {}
void I2C1_ER_IRQHandler        (void) {}
void I2C2_EV_IRQHandler        (void) {}
void I2C2_ER_IRQHandler        (void) {}
*/
void SPI1_IRQHandler()
{
	if (spi_byte_count)
		SPI1->DR = spi_tx_buffer[spi_byte_count-1];
	else
		SPI1->CR2 &= ~SPI_CR2_TXEIE;
	spi_byte_count--;
}
/*
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
	
	int32_t gyro_x;
	int32_t gyro_y;
	int32_t gyro_z;
	int32_t gyro_x_dc;
	int32_t gyro_y_dc;
	int32_t gyro_z_dc;
	
	int32_t throttle;
	int32_t aileron;
	int32_t elevator;
	int32_t rudder;
	
	int32_t error_pitch;
	int32_t error_roll;
	int32_t error_yaw;
	int32_t error_pitch_z;
	int32_t error_roll_z;
	int32_t error_yaw_z;
	int32_t error_pitch_int;
	int32_t error_roll_int;
	int32_t error_yaw_int;
	int32_t pitch;
	int32_t roll;
	int32_t yaw;
	
	int32_t motor_1_sign;
	int32_t motor_2_sign;
	int32_t motor_3_sign;
	int32_t motor_4_sign;
	uint16_t motor_1;
	uint16_t motor_2;
	uint16_t motor_3;
	uint16_t motor_4;
	
	//######## CLOCK ##########
	
	RCC->APB1ENR = RCC_APB1ENR_USART3EN | RCC_APB1ENR_TIM4EN;
	RCC->APB2ENR = RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_SPI1EN | RCC_APB2ENR_USART1EN | RCC_APB2ENR_TIM1EN;
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/1000); // 1 ms
	
	//####### VAR AND REG INIT #######
	
	FLAG = 0;
	for(i=0; i<REG_NB_ADDR; i++)
		reg[i] = reg_init[i];
	command_frame_count = 0;
	throttle = 0;
	aileron = 0;
	elevator = 0;
	rudder = 0;
	sensor_sample_count = 0;
	error_pitch_int = 0;
	error_roll_int = 0;
	error_yaw_int = 0;
	pitch = 0;
	roll = 0;
	yaw = 0;
	
	//####### BIND ########
	
	Wait(100);
	
	GPIOB->BSRR = GPIO_BSRR_BS11;
	GPIOB->CRH &= 0xFFFF0FFF;
	GPIOB->CRH |= GPIO_CRH_CNF11_0 | GPIO_CRH_MODE11_1;
	for (i=0; i<9; i++)
	{
		GPIOB->BSRR = GPIO_BSRR_BR11;
		Wait(1);
		GPIOB->BSRR = GPIO_BSRR_BS11;
		Wait(1);
	}
		
	//####### GPIO ##########
	
	AFIO->MAPR = AFIO_MAPR_SWJ_CFG_JTAGDISABLE; // Free PB3
	
	// PA4 : SPI CSN
	// PA5 : SPI SCK
	// PA6 : SPI MISO
	// PA7 : SPI MOSI
	GPIOA->CRL &= 0x0000FFFF;
	GPIOA->CRL |= GPIO_CRL_CNF4   | GPIO_CRL_MODE4 | 
	              GPIO_CRL_CNF5_1 | GPIO_CRL_MODE5 | 
	              GPIO_CRL_CNF6_0 | 
	              GPIO_CRL_CNF7_1 | GPIO_CRL_MODE7;
	
	// PA8 : servo 4
	// PA9 : UART1 Tx
	// PA10: UART1 Rx
	GPIOA->CRH &= 0xFFFFF000;
	GPIOA->CRH |= GPIO_CRH_CNF8_1 | GPIO_CRH_MODE8 |
	              GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9 | 
	              GPIO_CRH_CNF10_0;
	
	// PB2 : inv UART Rx
	// PB3 : LED
	// PB7 : servo 3
	GPIOB->CRL &= 0x0FFF00FF;
	GPIOB->CRL |=                   GPIO_CRL_MODE2_1 |
	              GPIO_CRL_CNF3_0 | GPIO_CRL_MODE3_1 | 
	              GPIO_CRL_CNF7_1 | GPIO_CRL_MODE7;
	GPIOB->BSRR = GPIO_BSRR_BR2; // Set PB2 to 0 => no inv UART Rx
	
	// PB8 : servo 2
	// PB9 : servo 1
	// PB10: UART3 Tx
	// PB11: UART3 Rx
	GPIOB->CRH &= 0xFFFF0000;
	GPIOB->CRH |= GPIO_CRH_CNF8_1  | GPIO_CRH_MODE8  |
	              GPIO_CRH_CNF9_1  | GPIO_CRH_MODE9  |
	              GPIO_CRH_CNF10_1 | GPIO_CRH_MODE10 | 
	              GPIO_CRH_CNF11_0;
	
	//####### INTERRUPT #######
	
	AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI3_PA;
	EXTI->RTSR |= EXTI_RTSR_RT3;
	EXTI->IMR = EXTI_IMR_IM3;
	
	NVIC_EnableIRQ(EXTI3_IRQn);
	NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	NVIC_EnableIRQ(DMA1_Channel3_IRQn);
	NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	NVIC_EnableIRQ(DMA1_Channel5_IRQn);
	NVIC_EnableIRQ(SPI1_IRQn);
	NVIC_EnableIRQ(TIM4_IRQn);
	
	NVIC_SetPriority(SPI1_IRQn,1);
	NVIC_SetPriority(EXTI3_IRQn,2);
	NVIC_SetPriority(DMA1_Channel2_IRQn,3);
	NVIC_SetPriority(DMA1_Channel3_IRQn,4);
	NVIC_SetPriority(DMA1_Channel5_IRQn,5);
	NVIC_SetPriority(DMA1_Channel4_IRQn,6);
	NVIC_SetPriority(TIM4_IRQn,7);
	
	//######## UART ##########
	
	//USART1->BRR = (1 & USART_BRR_DIV_Fraction) | ((52<<4) & USART_BRR_DIV_Mantissa); // 9600 bps
	USART1->BRR = (5 & USART_BRR_DIV_Fraction) | ((4<<4) & USART_BRR_DIV_Mantissa); // 115200 bps
	USART1->CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;
	USART1->CR3 = USART_CR3_DMAR;
	SetDmaForUartRx(4);
	
	USART3->BRR = (5 & USART_BRR_DIV_Fraction) | ((4<<4) & USART_BRR_DIV_Mantissa); // 115200 bps
	USART3->CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;
	USART3->CR3 = USART_CR3_DMAR;
	SetDmaForUart3Rx(16);
	
	//####### SPI #########
	
	SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_BR_0 | SPI_CR1_BR_1;
	SPI1->CR2 = SPI_CR2_SSOE;
	
	//####### TIM ########
	
	TIM4->CR1 = TIM_CR1_ARPE;
	TIM4->DIER = TIM_DIER_CC4IE;
	TIM4->PSC = 3; // 8MHz/4 = 2MHz = 0.5 us
	TIM4->ARR = 44000; // 0.5us * 44000 = 22ms
	TIM4->CCR2 = SERVO_MIN; // 0.5us * 1806 = 903us
	TIM4->CCR3 = SERVO_MIN;
	TIM4->CCR4 = SERVO_MIN;
	TIM4->CCMR1 = TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
	TIM4->CCMR2 = TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1;
	TIM4->CCER = TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
	
	TIM1->CR1 = TIM_CR1_ARPE;
	TIM1->PSC = 3;
	TIM1->ARR = 44000;
	TIM1->CCR1 = SERVO_MIN;
	TIM1->CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
	TIM1->CCER = TIM_CCER_CC1E;
	TIM1->BDTR = TIM_BDTR_MOE;
	
	TIM4->EGR = TIM_EGR_UG;
	TIM1->EGR = TIM_EGR_UG;
	TIM4->CR1 |= TIM_CR1_CEN;
	TIM1->CR1 |= TIM_CR1_CEN;
	
	//####### MPU INIT #########
	
	SpiWrite(MPU_PWR_MGMT_1, MPU_PWR_MGMT_1__DEVICE_RST);
	Wait(100);
	SpiWrite(MPU_PWR_MGMT_1, 0); //Get MPU out of sleep
	Wait(100);
	SpiWrite(MPU_USER_CTRL, MPU_USER_CTRL__I2C_IF_DIS);
	SpiWrite(MPU_SIGNAL_PATH_RST, MPU_SIGNAL_PATH_RST__ACCEL_RST | MPU_SIGNAL_PATH_RST__GYRO_RST | MPU_SIGNAL_PATH_RST__TEMP_RST);
	Wait(100);
	SpiWrite(MPU_CFG, MPU_CFG__DLPF_CFG(1)); // Filter ON(=> Fs=1kHz
	Wait(100); // wait for filter to settle
	Wait(100); // wait for filter to settle
	SpiWrite(MPU_INT_EN, MPU_INT_EN__DATA_RDY_EN);
	
	// Calibrate gyro DC
	gyro_x_dc = 0;
	gyro_y_dc = 0;
	gyro_z_dc = 0;
	while (sensor_sample_count < 1024)
	{
		if (FLAG & FLAG_SENSOR)
		{
			gyro_x_dc += (int32_t)gyro_x_raw;
			gyro_y_dc += (int32_t)gyro_y_raw;
			gyro_z_dc += (int32_t)gyro_z_raw;
			
			// Blink LED during calibration
			if ((sensor_sample_count & 0x3F) == 0)
				GPIOB->BSRR = GPIO_BSRR_BR3;
			else if ((sensor_sample_count & 0x3F) == 32)
				GPIOB->BSRR = GPIO_BSRR_BS3;
			
			FLAG &= ~FLAG_SENSOR;
		}
		__WFI();
	}
	gyro_x_dc = gyro_x_dc >> 10;
	gyro_y_dc = gyro_y_dc >> 10;
	gyro_z_dc = gyro_z_dc >> 10;
	
	//######## MAIN LOOP #########
	
	while (1)
	{
		// Process commands
		if (FLAG & FLAG_COMMAND)
		{
			throttle = ((int32_t)throttle_raw - (int32_t)REG_THROTTLE_OFFSET) * (int32_t)REG_THROTTLE_SCALE;
			aileron = ((int32_t)aileron_raw - (int32_t)REG_AILERON_OFFSET) * (int32_t)REG_AILERON_SCALE;
			elevator = ((int32_t)elevator_raw - (int32_t)REG_ELEVATOR_OFFSET) * (int32_t)REG_ELEVATOR_SCALE;
			rudder = ((int32_t)rudder_raw - (int32_t)REG_RUDDER_OFFSET) * (int32_t)REG_RUDDER_SCALE;
			
			// Debug
			if (REG_DEBUG_MUX == 2)
			{
				uart_tx_buffer[0] = command_frame_count;
				uart_tx_buffer[1] = (uint8_t)(throttle & 0xFF);
				uart_tx_buffer[2] = (uint8_t)((throttle & 0xFF00) >> 8);
				uart_tx_buffer[3] = (uint8_t)(aileron & 0xFF);
				uart_tx_buffer[4] = (uint8_t)((aileron & 0xFF00) >> 8);
				uart_tx_buffer[5] = (uint8_t)(elevator & 0xFF);
				uart_tx_buffer[6] = (uint8_t)((elevator & 0xFF00) >> 8);
				uart_tx_buffer[7] = (uint8_t)(rudder & 0xFF);
				uart_tx_buffer[8] = (uint8_t)((rudder & 0xFF00) >> 8);
				uart_tx_buffer[9] = (uint8_t)(armed_raw & 0xFF);
				uart_tx_buffer[10] = (uint8_t)((armed_raw & 0xFF00) >> 8);
			}
			
			FLAG &= ~FLAG_COMMAND;
		}
		
		// Process sensors
		if (FLAG & FLAG_SENSOR)
		{
			// Remove DC
			gyro_x = (int32_t)gyro_x_raw - gyro_x_dc;
			gyro_y = (int32_t)gyro_y_raw - gyro_y_dc;
			gyro_z = (int32_t)gyro_z_raw - gyro_z_dc;
			
			if (REG_DEBUG_MUX == 1)
			{
				uart_tx_buffer[0] = (uint8_t)sensor_sample_count;
				uart_tx_buffer[1] = (uint8_t)(gyro_x & 0xFF);
				uart_tx_buffer[2] = (uint8_t)((gyro_x & 0xFF00) >> 8);
				uart_tx_buffer[3] = (uint8_t)(gyro_y & 0xFF);
				uart_tx_buffer[4] = (uint8_t)((gyro_y & 0xFF00) >> 8);
				uart_tx_buffer[5] = (uint8_t)(gyro_z & 0xFF);
				uart_tx_buffer[6] = (uint8_t)((gyro_z & 0xFF00) >> 8);
				uart_tx_buffer[7] = (uint8_t)(accel_x_raw & 0xFF);
				uart_tx_buffer[8] = (uint8_t)((accel_x_raw & 0xFF00) >> 8);
				uart_tx_buffer[9] = (uint8_t)(accel_y_raw & 0xFF);
				uart_tx_buffer[10] = (uint8_t)((accel_y_raw & 0xFF00) >> 8);
				uart_tx_buffer[11] = (uint8_t)(accel_z_raw & 0xFF);
				uart_tx_buffer[12] = (uint8_t)((accel_z_raw & 0xFF00) >> 8);
			}
			
			error_pitch_z = error_pitch;
			error_roll_z = error_roll;
			error_yaw_z = error_yaw;
			
			error_pitch = elevator - gyro_x;
			error_roll = aileron - gyro_y;
			error_yaw = rudder - gyro_z;
			
			error_pitch_int += error_pitch;
			error_roll_int += error_pitch;
			error_pitch_int += error_pitch;
			
			pitch = (error_pitch * REG_PITCH_P + error_pitch_int * REG_PITCH_I + (error_pitch - error_pitch_z) * REG_PITCH_D) >> 16;
			roll = (error_roll * REG_ROLL_P + error_roll_int * REG_ROLL_I + (error_roll - error_roll_z) * REG_ROLL_D) >> 16;
			yaw = (error_yaw * REG_YAW_P + error_yaw_int * REG_YAW_I + (error_yaw - error_yaw_z) * REG_YAW_D) >> 16;
			
			if (REG_DEBUG_MUX == 3)
			{
				uart_tx_buffer[0] = (uint8_t)sensor_sample_count;
				uart_tx_buffer[1] = (uint8_t)(pitch & 0xFF);
				uart_tx_buffer[2] = (uint8_t)((pitch & 0xFF00) >> 8);
				uart_tx_buffer[3] = (uint8_t)(roll & 0xFF);
				uart_tx_buffer[4] = (uint8_t)((roll & 0xFF00) >> 8);
				uart_tx_buffer[5] = (uint8_t)(yaw & 0xFF);
				uart_tx_buffer[6] = (uint8_t)((yaw & 0xFF00) >> 8);
			}
			
			FLAG &= ~FLAG_SENSOR;
		}
		
		// 4 1
		// 3 2
		if (FLAG & FLAG_SERVO)
		{
			if (REG_THROTTLE_TEST)
				throttle = REG_THROTTLE_TEST;
			motor_1_sign = throttle + roll + pitch + yaw;
			motor_2_sign = throttle + roll - pitch - yaw;
			motor_3_sign = throttle - roll - pitch + yaw;
			motor_4_sign = throttle - roll + pitch - yaw;
			
			motor_1 = ((uint16_t)motor_1_sign  + REG_SERVO_OFFSET);
			motor_2 = ((uint16_t)motor_2_sign  + REG_SERVO_OFFSET);
			motor_3 = ((uint16_t)motor_3_sign  + REG_SERVO_OFFSET);
			motor_4 = ((uint16_t)motor_4_sign  + REG_SERVO_OFFSET);
			
			TIM4->CCR4 = (REG_MOTOR_SELECT & 1) ? REG_MOTOR_TEST : motor_1;
			TIM4->CCR3 = (REG_MOTOR_SELECT & 2) ? REG_MOTOR_TEST : motor_2;
			TIM4->CCR2 = (REG_MOTOR_SELECT & 4) ? REG_MOTOR_TEST : motor_3;
			TIM1->CCR1 = (REG_MOTOR_SELECT & 8) ? REG_MOTOR_TEST : motor_4;
			
			if (REG_DEBUG_MUX == 4)
			{
				uart_tx_buffer[0] = servo_count;
				uart_tx_buffer[1] = (uint8_t)(motor_1 & 0xFF);
				uart_tx_buffer[2] = (uint8_t)((motor_1 & 0xFF00) >> 8);
				uart_tx_buffer[3] = (uint8_t)(motor_2 & 0xFF);
				uart_tx_buffer[4] = (uint8_t)((motor_2 & 0xFF00) >> 8);
				uart_tx_buffer[5] = (uint8_t)(motor_3 & 0xFF);
				uart_tx_buffer[6] = (uint8_t)((motor_3 & 0xFF00) >> 8);
				uart_tx_buffer[7] = (uint8_t)(motor_4 & 0xFF);
				uart_tx_buffer[8] = (uint8_t)((motor_4 & 0xFF00) >> 8);
			}
			
			FLAG &= ~FLAG_SERVO;
		}
		
		// LED
		switch(REG_LED_MUX)
		{
			case 0:
				GPIOB->BSRR = GPIO_BSRR_BR3;
				break;
			case 1:
				GPIOB->BSRR = GPIO_BSRR_BS3;
				break;
			case 2:
				if ((sensor_sample_count & 0xFF) == 0)
					GPIOB->BSRR = GPIO_BSRR_BR3;
				else if ((sensor_sample_count & 0xFF) == 128)
					GPIOB->BSRR = GPIO_BSRR_BS3;
				break;
			case 3:
				if ((command_frame_count & 0x3F) == 0)
					GPIOB->BSRR = GPIO_BSRR_BR3;
				else if ((command_frame_count & 0x3F) == 32)
					GPIOB->BSRR = GPIO_BSRR_BS3;
				break;
			case 4:
				if ((servo_count & 0x1F) == 0)
					GPIOB->BSRR = GPIO_BSRR_BR3;
				else if ((servo_count & 0x1F) == 16)
					GPIOB->BSRR = GPIO_BSRR_BS3;
				break;
		}
		
		__WFI();
	}
}

void Reset_Handler()
{
	main();
}
