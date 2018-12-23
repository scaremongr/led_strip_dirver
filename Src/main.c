
#include "main.h"

#include "bit_ops.h"

LedStripRxPacket packetLedStrip;

WsOperationsStatus wsTxStatus[2];


static void LL_Init(void);
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);



/**
  * @brief  The application entry point.
  * @retval None
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM16_Init();
	MX_TIM17_Init();

	/* Start waiting for data form uart */
	StartUartRxTransfers();	

  /* Infinite loop */
  while (1)
  {

  }
}

static void LL_Init(void)
{
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  /* SVC_IRQn interrupt configuration */
  NVIC_SetPriority(SVC_IRQn, 0);
  /* PendSV_IRQn interrupt configuration */
  NVIC_SetPriority(PendSV_IRQn, 0);
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, 0);
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
  {
  Error_Handler();  
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {
    
  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_6);

  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
    
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  
  }
  LL_Init1msTick(48000000);

  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);

  LL_SetSystemCoreClock(48000000);

  LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK1);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, 0);
}

/* TIM16 init function */
static void MX_TIM16_Init(void)
{
  LL_TIM_InitTypeDef TIM_InitStruct;
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct;
  LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct;

  LL_GPIO_InitTypeDef GPIO_InitStruct;

  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM16);

  /* TIM16 DMA Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PRIORITY_HIGH);
  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MODE_CIRCULAR);
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PDATAALIGN_HALFWORD);
  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MDATAALIGN_BYTE);
	
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_3,
                         (uint32_t)&wsTxStatus[0].dmaBuffer,
                         (uint32_t)&TIM16->CCR1,
                         LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	
	LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_3);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_3);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_3);
	

	/* TIM16 Init */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 59;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM16, &TIM_InitStruct);

  LL_TIM_DisableARRPreload(TIM16);

  LL_TIM_OC_EnablePreload(TIM16, LL_TIM_CHANNEL_CH1);

  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  LL_TIM_OC_Init(TIM16, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);

  LL_TIM_OC_DisableFast(TIM16, LL_TIM_CHANNEL_CH1);

//  LL_TIM_SetTriggerOutput(TIM16, LL_TIM_TRGO_RESET);

//  LL_TIM_DisableMasterSlaveMode(TIM16);

  TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
  TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
  TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
  TIM_BDTRInitStruct.DeadTime = 0;
  TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
  TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
  TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
  LL_TIM_BDTR_Init(TIM16, &TIM_BDTRInitStruct);

  /**TIM16 GPIO Configuration  
  PA6   ------> TIM16_CH1 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
	
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* TIM17 init function */
static void MX_TIM17_Init(void)
{
  LL_TIM_InitTypeDef TIM_InitStruct;
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct;
  LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct;

  LL_GPIO_InitTypeDef GPIO_InitStruct;

  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM17);

  /* TIM17 DMA Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_HIGH);
  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_CIRCULAR);
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_HALFWORD);
  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE);
	
	LL_SYSCFG_SetRemapDMA_TIM(LL_SYSCFG_TIM17_RMP_DMA1_CH2);
	
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_2,
                         (uint32_t)&wsTxStatus[1].dmaBuffer,
                         (uint32_t)&TIM17->CCR1,
                         LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	
	LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_2);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_2);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_2);
	

	/* TIM17 Init */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 59;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM17, &TIM_InitStruct);

  LL_TIM_DisableARRPreload(TIM17);

  LL_TIM_OC_EnablePreload(TIM17, LL_TIM_CHANNEL_CH1);

  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  LL_TIM_OC_Init(TIM17, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);

  LL_TIM_OC_DisableFast(TIM17, LL_TIM_CHANNEL_CH1);

  LL_TIM_SetTriggerOutput(TIM17, LL_TIM_TRGO_RESET);

  LL_TIM_DisableMasterSlaveMode(TIM17);

  TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
  TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
  TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
  TIM_BDTRInitStruct.DeadTime = 0;
  TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
  TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
  TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
  LL_TIM_BDTR_Init(TIM17, &TIM_BDTRInitStruct);

  /**TIM17 GPIO Configuration  
  PA7   ------> TIM17_CH1 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
	
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{
  LL_USART_InitTypeDef USART_InitStruct;

  LL_GPIO_InitTypeDef GPIO_InitStruct;

  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1);
  
  /**USART1 GPIO Configuration  
  PA2   ------> USART1_TX
  PA3   ------> USART1_RX 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART1 DMA Init */
  
  /* USART1_RX Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_5, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PRIORITY_HIGH);
  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MDATAALIGN_BYTE);
  LL_SYSCFG_SetRemapDMA_USART(LL_SYSCFG_USART1RX_RMP_DMA1CH5);
	
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_5,
                         LL_USART_DMA_GetRegAddr(USART1, LL_USART_DMA_REG_DATA_RECEIVE),
                         (uint32_t)&packetLedStrip,
                         LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_5));
												 
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, sizeof(LedStripRxPacket));

  /* USART1_TX Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_4, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PRIORITY_HIGH);
  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MDATAALIGN_BYTE);
  LL_SYSCFG_SetRemapDMA_USART(LL_SYSCFG_USART1TX_RMP_DMA1CH4);
	
  USART_InitStruct.BaudRate = 921600;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);

  LL_USART_DisableIT_CTS(USART1);

  LL_USART_DisableOverrunDetect(USART1);

  LL_USART_ConfigAsyncMode(USART1);
	
  /* (5) Enable DMA transfer complete/error interrupts  */
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_4);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_4);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_5);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_5);

  LL_USART_Enable(USART1);
}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0);
  NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0);
  NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

}

static void MX_GPIO_Init(void)
{
  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

}

/**
  * @brief  This function initiates TX and RX DMA transfers by enabling DMA channels
  * @param  None
  * @retval None
  */
void StartUartRxTransfers(void)
{
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, sizeof(LedStripRxPacket));
	
  /* Enable DMA RX Interrupt */
  LL_USART_EnableDMAReq_RX(USART1);

  /* Enable DMA Channel Rx */
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5);
}

void StartUartTxTransfers(void)
{
	/* Enable DMA TX Interrupt */  
	LL_USART_EnableDMAReq_TX(USART1);
	
	/* Enable DMA Channel Tx */
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);
}

void StopUartRxTransfer(void)
{
	LL_USART_DisableDMAReq_TX(USART1);
	
	/* Disable DMA Channel Tx */
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_5);
}

void Timer16DmaStart(void)
{
	/* Set data transfer length */
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, RGB_BUFFER_SIZE * 24);
	
  /* Enable TIM1 channel 3 */
  LL_TIM_CC_EnableChannel(TIM16, LL_TIM_CHANNEL_CH1);
  
  /* Enable TIM1 outputs */
  LL_TIM_EnableAllOutputs(TIM16);
  
  /* Enable counter */
  LL_TIM_EnableCounter(TIM16);
  
  /* Force update generation */
  LL_TIM_GenerateEvent_UPDATE(TIM16);
		
	LL_TIM_EnableDMAReq_CC1(TIM16);
	
	LL_TIM_EnableARRPreload(TIM16);
	
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
}

void Timer17DmaStart(void)
{
		/* Set data transfer length */
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, RGB_BUFFER_SIZE * 24);
	
  /* Enable TIM1 channel 3 */
	LL_TIM_CC_EnableChannel(TIM17, LL_TIM_CHANNEL_CH1);
  
  /* Enable TIM1 outputs */
	LL_TIM_EnableAllOutputs(TIM17);
  
  /* Enable counter */
	LL_TIM_EnableCounter(TIM17);
  
  /* Force update generation */
	LL_TIM_GenerateEvent_UPDATE(TIM17);
		
	LL_TIM_EnableDMAReq_CC1(TIM17);
	
	LL_TIM_EnableARRPreload(TIM17);
	
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
}

void Timer16DmaStop(void)
{
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
	
	LL_TIM_OC_SetCompareCH2(TIM16, WS2812B_NO_PULSE);
}

void Timer17DmaStop(void)
{
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
	
	LL_TIM_OC_SetCompareCH2(TIM17, WS2812B_NO_PULSE);
}

//---------------------------------------------------
//-------DMA Buffer prepare functions----------------
//---------------------------------------------------

void WS2812InitFirstTransaction(void)
{
	wsTxStatus[0].stripLedCount = packetLedStrip.ch0_size;
	wsTxStatus[1].stripLedCount = packetLedStrip.ch1_size;
	
	wsTxStatus[0].conversionRemain = packetLedStrip.ch0_size;
	wsTxStatus[1].conversionRemain = packetLedStrip.ch1_size;
	
	wsTxStatus[0].transmissionRemain = packetLedStrip.ch0_size;
	wsTxStatus[1].transmissionRemain = packetLedStrip.ch1_size;
	
	wsTxStatus[0].firstPartPwmBuf = (PWM_t*)&wsTxStatus[0].dmaBuffer;
	wsTxStatus[0].secondPartPwmBuf = (PWM_t*)&wsTxStatus[0].dmaBuffer[RGB_BUFFER_HALF_SIZE];	
	wsTxStatus[0].rgbBufferPtr = (RGB_t*)packetLedStrip.ch0_data;
	
	wsTxStatus[1].firstPartPwmBuf = (PWM_t*)&wsTxStatus[1].dmaBuffer;
	wsTxStatus[1].secondPartPwmBuf = (PWM_t*)&wsTxStatus[1].dmaBuffer[RGB_BUFFER_HALF_SIZE];	
	wsTxStatus[1].rgbBufferPtr = (RGB_t*)packetLedStrip.ch1_data;
		
	DoConversionRgbToDmaFirstPart(&wsTxStatus[0]);
	DoConversionRgbToDmaSecondPart(&wsTxStatus[0]);
	
	DoConversionRgbToDmaFirstPart(&wsTxStatus[1]);
	DoConversionRgbToDmaSecondPart(&wsTxStatus[1]);
	
	Timer16DmaStart();
	Timer17DmaStart();
}

inline void DoConversionRgbToDmaFirstPart(WsOperationsStatus *opStatus)
{
	uint32_t convert_size = opStatus->conversionRemain >= RGB_BUFFER_HALF_SIZE ? RGB_BUFFER_HALF_SIZE : opStatus->conversionRemain;
	
	uint32_t n;
	uint32_t i;
	
	for(n = 0; n < convert_size; n++)
	{
		uint8_t mask = 0x80;
    
    for (i = 0; i < 8; i++)
    {
				opStatus->firstPartPwmBuf[n].r[i] = opStatus->rgbBufferPtr[n].r & mask ? WS2812B_PULSE_HIGH : WS2812B_PULSE_LOW;
        opStatus->firstPartPwmBuf[n].g[i] = opStatus->rgbBufferPtr[n].g & mask ? WS2812B_PULSE_HIGH : WS2812B_PULSE_LOW;
        opStatus->firstPartPwmBuf[n].b[i] = opStatus->rgbBufferPtr[n].b & mask ? WS2812B_PULSE_HIGH : WS2812B_PULSE_LOW;

        mask >>= 1;
    }
	}
	
	//Fill dma buffer to the end with low level signal if the rgb buffer is over
	for(; n < RGB_BUFFER_HALF_SIZE; n++)
	{
		for (i = 0; i < 8; i++)
    {
        opStatus->firstPartPwmBuf[n].r[i] = WS2812B_NO_PULSE;
        opStatus->firstPartPwmBuf[n].g[i] = WS2812B_NO_PULSE;
        opStatus->firstPartPwmBuf[n].b[i] = WS2812B_NO_PULSE;
    }
	}
	
	opStatus->conversionRemain -= convert_size;
	opStatus->rgbBufferPtr += convert_size;
	
}

inline void DoConversionRgbToDmaSecondPart(WsOperationsStatus *opStatus)
{
	uint32_t convert_size = opStatus->conversionRemain >= RGB_BUFFER_HALF_SIZE ? RGB_BUFFER_HALF_SIZE : opStatus->conversionRemain;
	
	uint32_t n;
	uint32_t i;
	
	for(n = 0; n < convert_size; n++)
	{
		uint8_t mask = 0x80;
    
    for (i = 0; i < 8; i++)
    {
        opStatus->secondPartPwmBuf[n].r[i] = opStatus->rgbBufferPtr[n].r & mask ? WS2812B_PULSE_HIGH : WS2812B_PULSE_LOW;
        opStatus->secondPartPwmBuf[n].g[i] = opStatus->rgbBufferPtr[n].g & mask ? WS2812B_PULSE_HIGH : WS2812B_PULSE_LOW;
        opStatus->secondPartPwmBuf[n].b[i] = opStatus->rgbBufferPtr[n].b & mask ? WS2812B_PULSE_HIGH : WS2812B_PULSE_LOW;

        mask >>= 1;
    }
	}
	
	//Fill dma buffer for the end with low level signal if the rgb buffer is over
	for(; n < RGB_BUFFER_HALF_SIZE; n++)
	{
		for (i = 0; i < 8; i++)
    {
        opStatus->secondPartPwmBuf[n].r[i] = WS2812B_NO_PULSE;
        opStatus->secondPartPwmBuf[n].g[i] = WS2812B_NO_PULSE;
        opStatus->secondPartPwmBuf[n].b[i] = WS2812B_NO_PULSE;
    }
	}
	
	opStatus->conversionRemain -= convert_size;
	opStatus->rgbBufferPtr += convert_size;
}

//If all data has been transferred return 1 else 0
inline uint8_t DidTransmission(WsOperationsStatus *opStatus)
{
	opStatus->transmissionRemain -= opStatus->transmissionRemain >= RGB_BUFFER_HALF_SIZE ? 
																			RGB_BUFFER_HALF_SIZE : opStatus->transmissionRemain;
	if(opStatus->transmissionRemain == 0)
		return 1;
		
	return 0;
}

//---------------------------------------------------
//---------------------Calbacks----------------------
//---------------------------------------------------
/**
  * @brief  Function called from DMA1 IRQ Handler when Tx transfer is completed (USART1)
  * @param  None
  * @retval None
  */
void USART1_DMA1_TransmitComplete_Callback(void)
{

}

static int uart_count = 0;
void USART1_DMA1_ReceiveComplete_Callback(void)
{
	/* DMA uart Tx transfer completed */

	uart_count++;
	StopUartRxTransfer();
	WS2812InitFirstTransaction();
}

void USART_TransferError_Callback(void)
{
	
}

//Callbacks for Timer1
void TIM1_DMA1_HalfTransmit_Callback(void)
{
	
}

void TIM1_DMA1_TransmitComplete_Callback(void)
{

}

void TIM1_TransferError_Callback(void)
{
	
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  while(1)
  {
		
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */

