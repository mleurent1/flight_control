#include "stm32f303xc.h"
#include "fc_reg.h"
#include "mpu_reg.h"

#define MOTOR_PULSE_MAX 2100
#define FLAG_SENSOR 0x01
#define FLAG_COMMAND 0x02
#define FLAG_SPI_TRANSACTION_COMPLETE 0x04
#define FLAG_SPI_RETURN_ON_UART 0x08
#define FLAG_UPDATE_REG 0x10

#define FPU_CPACR ((uint32_t *) 0xE000ED88)

volatile uint8_t FLAG;

volatile uint32_t tick;

volatile uint8_t uart3_rx_buffer[16];
volatile uint8_t uart3_tx_buffer[16];
volatile uint8_t uart2_rx_buffer[16];
volatile uint8_t spi2_rx_buffer[16];
volatile uint8_t spi2_tx_buffer[16];

volatile uint16_t sensor_sample_count;
volatile int16_t gyro_x_raw;
volatile int16_t gyro_y_raw;
volatile int16_t gyro_z_raw;
volatile int16_t accel_x_raw;
volatile int16_t accel_y_raw;
volatile int16_t accel_z_raw;
//volatile int16_t temp_raw;
volatile uint16_t sensor_error_count;

volatile uint16_t command_frame_count;
volatile uint8_t command_fades;
//volatile uint8_t command_system;
volatile uint16_t throttle_raw;
volatile uint16_t aileron_raw;
volatile uint16_t elevator_raw;
volatile uint16_t rudder_raw;
volatile uint16_t armed_raw;
volatile uint16_t command_error_count;

void SetDmaUart3Tx(uint8_t size)
{
	DMA1_Channel2->CCR = DMA_CCR_MINC | DMA_CCR_DIR;
	DMA1_Channel2->CMAR = (uint32_t)uart3_tx_buffer;
	DMA1_Channel2->CPAR = (uint32_t)&(USART3->TDR);
	DMA1_Channel2->CNDTR = size;
	DMA1_Channel2->CCR |= DMA_CCR_EN;
}

void SetDmaUart3Rx()
{
	DMA1_Channel3->CCR = DMA_CCR_TCIE | DMA_CCR_MINC;
	DMA1_Channel3->CMAR = (uint32_t)uart3_rx_buffer;
	DMA1_Channel3->CPAR = (uint32_t)&(USART3->RDR);
	DMA1_Channel3->CNDTR = 6;
	DMA1_Channel3->CCR |= DMA_CCR_EN;
}

void SetDmaSpi2Rx(uint8_t size)
{
	DMA1_Channel4->CCR = DMA_CCR_TCIE | DMA_CCR_MINC | DMA_CCR_PL;
	DMA1_Channel4->CMAR = (uint32_t)spi2_rx_buffer;
	DMA1_Channel4->CPAR = (uint32_t)&(SPI2->DR);
	DMA1_Channel4->CNDTR = size;
	DMA1_Channel4->CCR |= DMA_CCR_EN;
}

void SetDmaSpi2Tx(uint8_t size)
{
	DMA1_Channel5->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_PL_1;
	DMA1_Channel5->CMAR = (uint32_t)spi2_tx_buffer;
	DMA1_Channel5->CPAR = (uint32_t)&(SPI2->DR);
	DMA1_Channel5->CNDTR = size;
	DMA1_Channel5->CCR |= DMA_CCR_EN;
	
	FLAG &= ~FLAG_SPI_TRANSACTION_COMPLETE;
	SPI2->CR1 |= SPI_CR1_SPE;
}

void SetDmaUart2Rx()
{
	DMA1_Channel6->CCR = DMA_CCR_TCIE | DMA_CCR_MINC | DMA_CCR_PL_0;
	DMA1_Channel6->CMAR = (uint32_t)uart2_rx_buffer;
	DMA1_Channel6->CPAR = (uint32_t)&(USART2->RDR);
	DMA1_Channel6->CNDTR = 16;
	DMA1_Channel6->CCR |= DMA_CCR_EN;
}

void Wait(uint32_t ticks)
{
	uint32_t next_tick;
	next_tick = tick + ticks;
	while (tick != next_tick)
		__WFI();
}

void SpiWrite(uint8_t addr, uint8_t data)
{
	spi2_tx_buffer[0] = addr & 0x7F;
	spi2_tx_buffer[1] = data;
	SetDmaSpi2Rx(2);
	SetDmaSpi2Tx(2);
	Wait(1);
}

uint8_t SpiRead(uint8_t addr)
{
	spi2_tx_buffer[0] = 0x80 | (addr & 0x7F);
	SetDmaSpi2Rx(2);
	SetDmaSpi2Tx(2);
	Wait(1);
	return spi2_rx_buffer[1];
}

void bytes_to_float(volatile uint32_t * b, float * f)
{
	union {
		float f;
		uint32_t b;
	} f2b;
	f2b.b = *b;
	*f = f2b.f;
}

void float_to_bytes(float * f, volatile uint8_t * b)
{
	union {
		float f;
		uint8_t b[4];
	} f2b;
	f2b.f = *f;
	b[0] = f2b.b[0];
	b[1] = f2b.b[1];
	b[2] = f2b.b[2];
	b[3] = f2b.b[3];
}

//######## System Interrupts ########

void SysTick_Handler()
{
	tick++;
}

//####### External Interrupts ########

void EXTI15_10_IRQHandler()
{
	EXTI->PR = EXTI_PR_PIF15; // Clear pending request
	
	sensor_sample_count++;
	
	if (REG_CTRL__READ_SENSOR)
	{
		if (FLAG & FLAG_SPI_TRANSACTION_COMPLETE)
		{
			accel_x_raw = ((int16_t)spi2_rx_buffer[1]  << 8) | (int16_t)spi2_rx_buffer[2];
			accel_y_raw = ((int16_t)spi2_rx_buffer[3]  << 8) | (int16_t)spi2_rx_buffer[4];
			accel_z_raw = ((int16_t)spi2_rx_buffer[5]  << 8) | (int16_t)spi2_rx_buffer[6];
			//temp_raw    = ((int16_t)spi2_rx_buffer[7]  << 8) | (int16_t)spi2_rx_buffer[8];
			gyro_x_raw  = ((int16_t)spi2_rx_buffer[9]  << 8) | (int16_t)spi2_rx_buffer[10];
			gyro_y_raw  = ((int16_t)spi2_rx_buffer[11] << 8) | (int16_t)spi2_rx_buffer[12];
			gyro_z_raw  = ((int16_t)spi2_rx_buffer[13] << 8) | (int16_t)spi2_rx_buffer[14];
			
			FLAG |= FLAG_SENSOR;
		}
		else
		{
			sensor_error_count++;
		}
		
		spi2_tx_buffer[0] = 0x80 | 59;
		SetDmaSpi2Rx(15);
		SetDmaSpi2Tx(15);
	}
}

void DMA1_Channel3_IRQHandler()
{
	uint8_t cmd;
	uint8_t addr;
	
	DMA1->IFCR = DMA_IFCR_CTCIF3; // clear flag
	
	cmd = uart3_rx_buffer[0];
	addr = uart3_rx_buffer[1];
	
	if (cmd == 0)
	{
		uart3_tx_buffer[0] = (uint8_t)((reg[addr] >> 24) & 0xFF);
		uart3_tx_buffer[1] = (uint8_t)((reg[addr] >> 16) & 0xFF);
		uart3_tx_buffer[2] = (uint8_t)((reg[addr] >>  8) & 0xFF);
		uart3_tx_buffer[3] = (uint8_t)((reg[addr] >>  0) & 0xFF);
		SetDmaUart3Tx(4);
	}
	else if (cmd == 1)
	{
		reg[addr] = ((uint32_t)uart3_rx_buffer[2] << 24) | ((uint32_t)uart3_rx_buffer[3] << 16) | ((uint32_t)uart3_rx_buffer[4] << 8) | (uint32_t)uart3_rx_buffer[5];
		FLAG |= FLAG_UPDATE_REG;
	}
	else if (cmd == 2)
	{
		FLAG |= FLAG_SPI_RETURN_ON_UART;
		spi2_tx_buffer[0] = 0x80 | (addr & 0x7F);
		SetDmaSpi2Rx(2);
		SetDmaSpi2Tx(2);
	}
	else if (cmd == 3)
	{
		spi2_tx_buffer[0] = addr & 0x7F;
		spi2_tx_buffer[1] = uart3_rx_buffer[5];
		SetDmaSpi2Rx(2);
		SetDmaSpi2Tx(2);
	}
	
	SetDmaUart3Rx();
}

void DMA1_Channel4_IRQHandler()
{
	uint8_t error = 0;
	
	DMA1->IFCR = DMA_IFCR_CTCIF4; // clear flag
	
	if (DMA1->ISR & DMA_ISR_TEIF4)
		error = 1;
	
	while (SPI2->SR & SPI_SR_FTLVL) {}
	while (SPI2->SR & SPI_SR_BSY) 
		
	if (SPI2->SR & SPI_SR_OVR)
	{
		error = 1;
		SPI2->DR;
	}
	if (SPI2->SR & SPI_SR_MODF)
		error = 1;
	
	SPI2->CR1 &= ~SPI_CR1_SPE; // disable SPI
	
	while (SPI2->SR & SPI_SR_FRLVL) // empty FIFO
		SPI2->DR;
	
	if (error == 0)
		FLAG |= FLAG_SPI_TRANSACTION_COMPLETE;
	
	if (FLAG & FLAG_SPI_RETURN_ON_UART)
	{
		FLAG &= ~FLAG_SPI_RETURN_ON_UART;
		uart3_tx_buffer[0] = spi2_rx_buffer[1];
		SetDmaUart3Tx(1);
	}
}

void DMA1_Channel6_IRQHandler()
{
	int i;
	uint8_t chan_id;
	uint16_t servo;
	uint8_t error = 0;
	
	DMA1->IFCR = DMA_IFCR_CTCIF6; // clear flag
	
	if (DMA1->ISR & DMA_ISR_TEIF6)
		error = 1;
	if (USART2->ISR & (USART_ISR_ORE | USART_ISR_NE))
	{
		error = 1;
		USART2->ICR = USART_ICR_ORECF | USART_ICR_NCF;
	}
	
	TIM6->EGR = TIM_EGR_UG; // Reset timer
	
	command_frame_count++;
	
	if (error == 0)
	{
		command_fades = uart2_rx_buffer[0];
		//command_system = uart2_rx_buffer[1];
		for (i=0; i<7; i++)
		{
			servo = ((uint16_t)uart2_rx_buffer[2+2*i] << 8) | (uint16_t)uart2_rx_buffer[3+2*i];
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
	}
	else
		command_error_count++;
	
	SetDmaUart2Rx();
}

//######## MAIN #########

int main()
{
	int i;
	
	float gyro_x;
	float gyro_y;
	float gyro_z;
	
	float gyro_x_dc;
	float gyro_y_dc;
	float gyro_z_dc;
	
	float gyro_x_scale;
	float gyro_y_scale;
	float gyro_z_scale;
	
	float throttle;
	float aileron;
	float elevator;
	float rudder;
	
	float throttle_scale;
	float aileron_scale;
	float elevator_scale;
	float rudder_scale;
	
	float error_pitch;
	float error_roll;
	float error_yaw;
	float error_pitch_z;
	float error_roll_z;
	float error_yaw_z;
	float error_pitch_int;
	float error_roll_int;
	float error_yaw_int;
	
	float pitch;
	float roll;
	float yaw;
	
	float pitch_p;
	float pitch_i;
	float pitch_d;
	float roll_p;
	float roll_i;
	float roll_d;
	float yaw_p;
	float yaw_i;
	float yaw_d;
	
	float motor[4];
	int32_t motor_clip[4];
	uint16_t motor_raw[4];
	
	//######## CLOCK ##########
	
	RCC->APB1ENR = RCC_APB1ENR_USART2EN | RCC_APB1ENR_USART3EN | RCC_APB1ENR_SPI2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM6EN | RCC_APB1ENR_TIM7EN;
	RCC->APB2ENR = RCC_APB2ENR_SYSCFGEN;
	RCC->AHBENR |= RCC_AHBENR_DMA1EN | RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/1000); // 1 ms
	
	//####### VAR AND REG INIT #######
	
	for(i=0; i<REG_NB_ADDR; i++)
		reg[i] = reg_init[i];
	
	FLAG = FLAG_UPDATE_REG;
	
	command_frame_count = 0;
	throttle_raw = 0;
	aileron_raw = 0;
	elevator_raw = 0;
	rudder_raw = 0;
	armed_raw = 0;
	
	sensor_sample_count = 0;
	sensor_error_count = 0;
	gyro_x_raw = 0;
	gyro_y_raw = 0;
	gyro_z_raw = 0;
	accel_x_raw = 0;
	accel_y_raw = 0;
	accel_z_raw = 0;
	
	throttle = 0;
	aileron = 0;
	elevator = 0;
	rudder = 0;
	
	error_pitch_int = 0;
	error_roll_int = 0;
	error_yaw_int = 0;
	
	pitch = 0;
	roll = 0;
	yaw = 0;
	
	//####### BIND ########
	
	// A2 : Binding ON bar
	GPIOA->MODER &= ~GPIO_MODER_MODER2_Msk; //Reset MODER
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR2_Msk; // Reset PUPDR
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR2_0; // PU
	
	Wait(1);
	
	if ((GPIOA->IDR & GPIO_IDR_2) == 0)
	{
		Wait(100);
		
		GPIOB->BSRR = GPIO_BSRR_BS_4;
		GPIOB->MODER &= ~GPIO_MODER_MODER4_Msk;
		GPIOB->MODER |= GPIO_MODER_MODER4_0; // Output GP
		GPIOB->OTYPER &= ~GPIO_OTYPER_OT_4; // PP
		GPIOB->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR4_Msk; // Low speed
		GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR4_Msk; // No PU/PD
		for (i=0; i<9; i++)
		{
			GPIOB->BSRR = GPIO_BSRR_BR_4;
			Wait(1);
			GPIOB->BSRR = GPIO_BSRR_BS_4;
			Wait(1);
		}
	}
	
	//####### GPIO ##########
	
	// A0 : Beep
	// A4 : Motor 2
	// A6 : Motor 1
	// A15: MPU interrupt
	GPIOA->MODER &= ~(GPIO_MODER_MODER0_Msk | GPIO_MODER_MODER4_Msk | GPIO_MODER_MODER6_Msk | GPIO_MODER_MODER15_Msk); //Reset MODER
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0_Msk | GPIO_PUPDR_PUPDR4_Msk | GPIO_PUPDR_PUPDR6_Msk | GPIO_PUPDR_PUPDR15_Msk); // Reset PUPDR
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR4_Msk | GPIO_OSPEEDER_OSPEEDR6_Msk); // Reset OSPEEDR for output only
	GPIOA->MODER |= GPIO_MODER_MODER4_1 | GPIO_MODER_MODER6_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4 | GPIO_OSPEEDER_OSPEEDR6; // Full-speed
	GPIOA->AFR[0] = (GPIO_AFRL_AFRL4_Msk & (2 << GPIO_AFRL_AFRL4_Pos)) | (GPIO_AFRL_AFRL6_Msk & (2 << GPIO_AFRL_AFRL6_Pos)); // AF2 = TIM3
	
	// B0 : Motor 3
	// B1 : Motor 4
	// B3 : UART2 Tx NOT USED
	// B4 : UART2 Rx
	// B5 : Red LED
	// B6 : UART1 Tx NOT USED
	// B7 : UART1 Rx NOT USED
	// B9 : LED ?
	// B10: UART3 Tx
	// B11: UART3 Rx
	// B12: SPI2 CS
	// B13: SPI2 CLK
	// B14: SPI2 MISO
	// B15: SPI2 MOSI
	GPIOB->MODER &= ~(GPIO_MODER_MODER0_Msk | GPIO_MODER_MODER1_Msk | GPIO_MODER_MODER4_Msk | GPIO_MODER_MODER5_Msk | GPIO_MODER_MODER10_Msk | GPIO_MODER_MODER11_Msk | GPIO_MODER_MODER12_Msk | GPIO_MODER_MODER13_Msk | GPIO_MODER_MODER14_Msk | GPIO_MODER_MODER15_Msk); // Reset MODER
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR0_Msk | GPIO_PUPDR_PUPDR1_Msk | GPIO_PUPDR_PUPDR4_Msk | GPIO_PUPDR_PUPDR5_Msk | GPIO_PUPDR_PUPDR10_Msk | GPIO_PUPDR_PUPDR11_Msk | GPIO_PUPDR_PUPDR12_Msk | GPIO_PUPDR_PUPDR13_Msk | GPIO_PUPDR_PUPDR14_Msk | GPIO_PUPDR_PUPDR15_Msk); // Reset PUPDR
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0_Msk | GPIO_OSPEEDER_OSPEEDR1_Msk | GPIO_OSPEEDER_OSPEEDR5_Msk | GPIO_OSPEEDER_OSPEEDR10_Msk | GPIO_OSPEEDER_OSPEEDR12_Msk | GPIO_OSPEEDER_OSPEEDR13_Msk | GPIO_OSPEEDER_OSPEEDR15_Msk); // Reset OSPEEDR for output only
	GPIOB->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1 | GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_0 | GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1 | GPIO_MODER_MODER12_1 | GPIO_MODER_MODER13_1 | GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR12_0; // PU
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0 | GPIO_OSPEEDER_OSPEEDR1 | GPIO_OSPEEDER_OSPEEDR10 | GPIO_OSPEEDER_OSPEEDR12 | GPIO_OSPEEDER_OSPEEDR13 | GPIO_OSPEEDER_OSPEEDR15; // Full-speed
	GPIOB->OTYPER = GPIO_OTYPER_OT_5 | GPIO_OTYPER_OT_12; // OD
	GPIOB->AFR[0] = (GPIO_AFRL_AFRL0_Msk & (2 << GPIO_AFRL_AFRL0_Pos)) | (GPIO_AFRL_AFRL1_Msk & (2 << GPIO_AFRL_AFRL1_Pos)) | (GPIO_AFRL_AFRL4_Msk & (7 << GPIO_AFRL_AFRL4_Pos)); // AF2 = TIM3, AF7 = USART2
	GPIOB->AFR[1] = (GPIO_AFRH_AFRH2_Msk & (7 << GPIO_AFRH_AFRH2_Pos)) | (GPIO_AFRH_AFRH3_Msk & (7 << GPIO_AFRH_AFRH3_Pos)) | (GPIO_AFRH_AFRH4_Msk & (5 << GPIO_AFRH_AFRH4_Pos)) | (GPIO_AFRH_AFRH5_Msk & (5 << GPIO_AFRH_AFRH5_Pos)) | (GPIO_AFRH_AFRH6_Msk & (5 << GPIO_AFRH_AFRH6_Pos)) | (GPIO_AFRH_AFRH7_Msk & (5 << GPIO_AFRH_AFRH7_Pos)); // AF7 = USART3, AF5 = SPI2
	
	//######## UART ##########
	
	USART3->BRR = 69; // 115200 bps
	USART3->CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;
	USART3->CR3 = USART_CR3_DMAR | USART_CR3_DMAT;
	SetDmaUart3Rx();
	
	USART2->BRR = 69; // 115200 bps
	USART2->CR1 = USART_CR1_UE | USART_CR1_RE;
	USART2->CR3 = USART_CR3_DMAR;
	SetDmaUart2Rx();
	
	//####### SPI #########
	
	SPI2->CR1 = SPI_CR1_MSTR | SPI_CR1_BR_0 | SPI_CR1_BR_1;
	SPI2->CR2 = SPI_CR2_SSOE | SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN | SPI_CR2_FRXTH;
	
	//####### TIM ########
	
	TIM3->CR1 = TIM_CR1_OPM;
	TIM3->PSC = 0; // 8MHz/(x+1)
	TIM3->ARR = MOTOR_PULSE_MAX;
	TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
	TIM3->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_0;
	TIM3->CCMR2 = TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_0 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_0;
	
	TIM6->PSC = 7999; // ms
	TIM6->ARR = 65535;
	TIM6->CR1 = TIM_CR1_CEN;
	TIM7->PSC = 7; // us
	TIM7->ARR = 65535;
	TIM7->CR1 = TIM_CR1_CEN;
	
	//####### INTERRUPT #######
	
	SYSCFG->EXTICR[3] = SYSCFG_EXTICR4_EXTI15_PA;
	EXTI->RTSR |= EXTI_RTSR_TR15;
	//EXTI->IMR = EXTI_IMR_MR15; // To be enabled after MPU init
	
	NVIC_EnableIRQ(EXTI15_10_IRQn);
	NVIC_EnableIRQ(DMA1_Channel3_IRQn);
	NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	NVIC_EnableIRQ(DMA1_Channel6_IRQn);
	
	NVIC_SetPriority(EXTI15_10_IRQn,2);
	NVIC_SetPriority(DMA1_Channel3_IRQn,4);
	NVIC_SetPriority(DMA1_Channel4_IRQn,1);
	NVIC_SetPriority(DMA1_Channel6_IRQn,3);
	
	//####### MPU INIT #########
	
	SpiWrite(MPU_PWR_MGMT_1, MPU_PWR_MGMT_1__DEVICE_RST);
	Wait(100);
	SpiWrite(MPU_SIGNAL_PATH_RST, MPU_SIGNAL_PATH_RST__ACCEL_RST | MPU_SIGNAL_PATH_RST__GYRO_RST | MPU_SIGNAL_PATH_RST__TEMP_RST);
	Wait(100);
	SpiWrite(MPU_USER_CTRL, MPU_USER_CTRL__I2C_IF_DIS);
	SpiWrite(MPU_PWR_MGMT_1, 0); // Get MPU out of sleep
	Wait(100);
	SpiWrite(MPU_PWR_MGMT_1, MPU_PWR_MGMT_1__CLKSEL(1)); // Set CLK = gyro X clock
	Wait(100);
	//I2cWrite(MPU_SMPLRT_DIV, 0); // Sample rate = Fs/(x+1)
	SpiWrite(MPU_CFG, MPU_CFG__DLPF_CFG(2)); // Filter ON => Fs=1kHz
	Wait(100); // wait for filter to settle
	SpiWrite(MPU_INT_EN, MPU_INT_EN__DATA_RDY_EN);
	
	EXTI->IMR = EXTI_IMR_MR15;
	
	// Gyro calibration
	gyro_x_dc = 0.0f;
	gyro_y_dc = 0.0f;
	gyro_z_dc = 0.0f;
	
	while (sensor_sample_count < 1000)
	{
		if (FLAG & FLAG_SENSOR)
		{
			FLAG &= ~FLAG_SENSOR;
			
			gyro_x_dc += (float)gyro_x_raw;
			gyro_y_dc += (float)gyro_y_raw;
			gyro_z_dc += (float)gyro_z_raw;
			
			// Blink LED during calibration
			if ((sensor_sample_count & 0x3F) == 0)
				GPIOB->BSRR = GPIO_BSRR_BS_5;
			else if ((sensor_sample_count & 0x3F) == 32)
				GPIOB->BSRR = GPIO_BSRR_BR_5;
		}
		__WFI();
	}
	gyro_x_dc = gyro_x_dc / 1000.0f;
	gyro_y_dc = gyro_y_dc / 1000.0f;
	gyro_z_dc = gyro_z_dc / 1000.0f;
	
	gyro_x_scale = 0.0038f;
	gyro_y_scale = 0.0038f;
	gyro_z_scale = 0.0038f;
	
	//######## MAIN LOOP #########
	
	while (1)
	{
		// Update float registers
		if (FLAG & FLAG_UPDATE_REG)
		{
			FLAG &= ~FLAG_UPDATE_REG;
			
			if (REG_CTRL__LED == 0)
				GPIOB->BSRR = GPIO_BSRR_BS_5;
			else if (REG_CTRL__LED == 1)
				GPIOB->BSRR = GPIO_BSRR_BR_5;
			
			bytes_to_float(&REG_THROTTLE_SCALE, &throttle_scale);
			bytes_to_float(&REG_AILERON_SCALE, &aileron_scale);
			bytes_to_float(&REG_ELEVATOR_SCALE, &elevator_scale);
			bytes_to_float(&REG_RUDDER_SCALE, &rudder_scale);
			
			bytes_to_float(&REG_PITCH_P, &pitch_p);
			bytes_to_float(&REG_PITCH_I, &pitch_i);
			bytes_to_float(&REG_PITCH_D, &pitch_d);
			bytes_to_float(&REG_ROLL_P, &roll_p);
			bytes_to_float(&REG_ROLL_I, &roll_i);
			bytes_to_float(&REG_ROLL_D, &roll_d);
			bytes_to_float(&REG_YAW_P, &yaw_p);
			bytes_to_float(&REG_YAW_I, &yaw_i);
			bytes_to_float(&REG_YAW_D, &yaw_d);
		}
		
		TIM7->EGR = TIM_EGR_UG;
		
		// Process commands
		if (FLAG & FLAG_COMMAND)
		{
			FLAG &= ~FLAG_COMMAND;
			
			if (REG_CTRL__LED == 3)
			{
				if ((command_frame_count & 0x3F) == 0)
					GPIOB->BSRR = GPIO_BSRR_BR_5;
				else if ((command_frame_count & 0x3F) == 32)
					GPIOB->BSRR = GPIO_BSRR_BS_5;
			}
			
			REG_ERROR &= 0x0000FFFF;
			REG_ERROR |= (uint32_t)command_fades << 16; 
			
			if (REG_DEBUG == 3)
			{
				REG_DEBUG = 0;
				uart3_tx_buffer[ 0] = (uint8_t) command_frame_count;
				uart3_tx_buffer[ 1] = (uint8_t) throttle_raw;
				uart3_tx_buffer[ 2] = (uint8_t)(throttle_raw >> 8);
				uart3_tx_buffer[ 3] = (uint8_t) aileron_raw;
				uart3_tx_buffer[ 4] = (uint8_t)(aileron_raw>> 8);
				uart3_tx_buffer[ 5] = (uint8_t) elevator_raw;
				uart3_tx_buffer[ 6] = (uint8_t)(elevator_raw>> 8);
				uart3_tx_buffer[ 7] = (uint8_t) rudder_raw;
				uart3_tx_buffer[ 8] = (uint8_t)(rudder_raw >> 8);
				uart3_tx_buffer[ 9] = (uint8_t) armed_raw;
				uart3_tx_buffer[10] = (uint8_t)(armed_raw >> 8);
				SetDmaUart3Tx(11);
			}
			
			throttle = (float)((int32_t)throttle_raw - (int32_t)REG_THROTTLE__OFFSET) * throttle_scale;
			aileron = (float)((int32_t)aileron_raw - 1024) * aileron_scale;
			elevator = (float)((int32_t)elevator_raw - 1024) * elevator_scale;
			rudder = (float)((int32_t)rudder_raw - 1024) * rudder_scale;
			
			if (REG_DEBUG == 4)
			{
				REG_DEBUG = 0;
				uart3_tx_buffer[0] = (uint8_t)command_frame_count;
				float_to_bytes(&throttle, &uart3_tx_buffer[1]);
				float_to_bytes(&aileron, &uart3_tx_buffer[5]);
				float_to_bytes(&elevator, &uart3_tx_buffer[9]);
				float_to_bytes(&rudder, &uart3_tx_buffer[13]);
				SetDmaUart3Tx(17);
			}
		}
		
		if (TIM6->CNT > 200)
			armed_raw = 0;
		
		// Process sensors
		if (FLAG & FLAG_SENSOR)
		{
			FLAG &= ~FLAG_SENSOR;
			
			if (REG_CTRL__LED == 2)
			{
				if ((sensor_sample_count & 0xFF) == 0)
					GPIOB->BSRR = GPIO_BSRR_BR_5;
				else if ((sensor_sample_count & 0xFF) == 128)
					GPIOB->BSRR = GPIO_BSRR_BS_5;
			}
			
			REG_ERROR &= 0xFFFF0000;
			REG_ERROR |= (uint32_t)sensor_error_count & 0x0000FFFF;; 
			
			if (REG_DEBUG == 1)
			{
				REG_DEBUG = 0;
				uart3_tx_buffer[ 0] = (uint8_t) sensor_sample_count;
				uart3_tx_buffer[ 1] = (uint8_t) gyro_x_raw;
				uart3_tx_buffer[ 2] = (uint8_t)(gyro_x_raw >> 8);
				uart3_tx_buffer[ 3] = (uint8_t) gyro_y_raw;
				uart3_tx_buffer[ 4] = (uint8_t)(gyro_y_raw >> 8);
				uart3_tx_buffer[ 5] = (uint8_t) gyro_z_raw;
				uart3_tx_buffer[ 6] = (uint8_t)(gyro_z_raw >> 8);
				uart3_tx_buffer[ 7] = (uint8_t) accel_x_raw;
				uart3_tx_buffer[ 8] = (uint8_t)(accel_x_raw >> 8);
				uart3_tx_buffer[ 9] = (uint8_t) accel_y_raw;
				uart3_tx_buffer[10] = (uint8_t)(accel_y_raw >> 8);
				uart3_tx_buffer[11] = (uint8_t) accel_z_raw;
				uart3_tx_buffer[12] = (uint8_t)(accel_z_raw >> 8);
				SetDmaUart3Tx(13);
			}
			
			// Remove DC
			gyro_x = ((float)gyro_x_raw - gyro_x_dc) * gyro_x_scale;
			gyro_y = ((float)gyro_y_raw - gyro_y_dc) * gyro_y_scale;
			gyro_z = ((float)gyro_z_raw - gyro_z_dc) * gyro_z_scale;
			
			if ((gyro_x < 0.5f) && (gyro_x > -0.5f))
				gyro_x = 0.0f;
			if ((gyro_y < 0.5f) && (gyro_y > -0.5f))
				gyro_y = 0.0f;
			if ((gyro_z < 0.5f) && (gyro_z > -0.5f))
				gyro_z = 0.0f;
			
			if (REG_DEBUG == 2)
			{
				REG_DEBUG = 0;
				uart3_tx_buffer[0] = (uint8_t)sensor_sample_count;
				float_to_bytes(&gyro_x, &uart3_tx_buffer[1]);
				float_to_bytes(&gyro_y, &uart3_tx_buffer[5]);
				float_to_bytes(&gyro_z, &uart3_tx_buffer[9]);
				SetDmaUart3Tx(13);
			}
			
			error_pitch_z = error_pitch;
			error_roll_z = error_roll;
			error_yaw_z = error_yaw;
			
			error_pitch = elevator - gyro_y;
			error_roll = aileron - gyro_x;
			error_yaw = rudder - gyro_z;
			
			if ((armed_raw < 1024) && REG_CTRL__RESET_INT_ON_ARMED)
			{
				error_pitch_int = 0.0f;
				error_roll_int = 0.0f;
				error_yaw_int = 0.0f;
			}
			else
			{
				error_pitch_int += error_pitch;
				error_roll_int += error_roll;
				error_yaw_int += error_yaw;
			}
			
			pitch = error_pitch * pitch_p + error_pitch_int * pitch_i + (error_pitch - error_pitch_z) * pitch_d;
			roll = error_roll * roll_p + error_roll_int * roll_i + (error_roll - error_roll_z) * roll_d;
			yaw = error_yaw * yaw_p + error_yaw_int * yaw_i + (error_yaw - error_yaw_z) * yaw_d;
			
			if (REG_DEBUG == 5)
			{
				REG_DEBUG = 0;
				uart3_tx_buffer[0] = (uint8_t)sensor_sample_count;
				float_to_bytes(&pitch, &uart3_tx_buffer[1]);
				float_to_bytes(&roll, &uart3_tx_buffer[5]);
				float_to_bytes(&yaw, &uart3_tx_buffer[9]);
				SetDmaUart3Tx(13);
			}
			
			motor[0] = throttle + roll + pitch - yaw;
			motor[1] = throttle + roll - pitch + yaw;
			motor[2] = throttle - roll - pitch - yaw;
			motor[3] = throttle - roll + pitch + yaw;
			
			if (REG_DEBUG == 6)
			{
				REG_DEBUG = 0;
				uart3_tx_buffer[0] = (uint8_t)sensor_sample_count;
				for (i=0; i<4; i++)
					float_to_bytes(&motor[i], &uart3_tx_buffer[i*4+1]);
				SetDmaUart3Tx(17);
			}
			
			for (i=0; i<4; i++)
			{
				motor_clip[i] = (int32_t)motor[i] + (int32_t)REG_THROTTLE__ARMED;
				
				if (motor_clip[i] < (int32_t)REG_MOTOR__MIN)
					motor_clip[i] = (int32_t)REG_MOTOR__MIN;
				else if (motor_clip[i] > (int32_t)REG_MOTOR__MAX)
					motor_clip[i] = (int32_t)REG_MOTOR__MAX;
				
				motor_raw[i] = (uint16_t)motor_clip[i];
			}
			
			if (REG_DEBUG == 7)
			{
				REG_DEBUG = 0;
				uart3_tx_buffer[0] = (uint8_t)sensor_sample_count;
				for (i=0; i<4; i++)
				{
					uart3_tx_buffer[i*2+1] = (uint8_t) motor_raw[i];
					uart3_tx_buffer[i*2+2] = (uint8_t)(motor_raw[i] >> 8);
				}
				SetDmaUart3Tx(9);
			}
			
			if (armed_raw < 1024)
			{
				TIM3->CCR1 = (REG_CTRL__MOTOR_SEL == 2) ? (MOTOR_PULSE_MAX - (uint16_t)REG_CTRL__MOTOR_TEST) : (MOTOR_PULSE_MAX - REG_MOTOR__MIN);
				TIM3->CCR2 = (REG_CTRL__MOTOR_SEL == 1) ? (MOTOR_PULSE_MAX - (uint16_t)REG_CTRL__MOTOR_TEST) : (MOTOR_PULSE_MAX - REG_MOTOR__MIN);
				TIM3->CCR3 = (REG_CTRL__MOTOR_SEL == 3) ? (MOTOR_PULSE_MAX - (uint16_t)REG_CTRL__MOTOR_TEST) : (MOTOR_PULSE_MAX - REG_MOTOR__MIN);
				TIM3->CCR4 = (REG_CTRL__MOTOR_SEL == 4) ? (MOTOR_PULSE_MAX - (uint16_t)REG_CTRL__MOTOR_TEST) : (MOTOR_PULSE_MAX - REG_MOTOR__MIN);
			}
			else
			{
				TIM3->CCR1 = MOTOR_PULSE_MAX - motor_raw[1];
				TIM3->CCR2 = MOTOR_PULSE_MAX - motor_raw[0];
				TIM3->CCR3 = MOTOR_PULSE_MAX - motor_raw[2];
				TIM3->CCR4 = MOTOR_PULSE_MAX - motor_raw[3];
			}
			TIM3->CR1 |= TIM_CR1_CEN;
		}
		
		REG_TIME = TIM7->CNT;
		
		__WFI();
	}
}

void Reset_Handler()
{
	// Enable FPU
	*FPU_CPACR = (0xF << 20); 
	
	// Reset pipeline
	__DSB();
	__ISB();
	
	main();
}
