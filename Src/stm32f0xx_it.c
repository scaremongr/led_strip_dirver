

#include "stm32f0xx.h"
#include "stm32f0xx_it.h"


/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  while (1)
  {

  }
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  
}

/**
* @brief This function handles DMA1 channel 2 and 3 interrupts.
*/
void DMA1_Channel2_3_IRQHandler(void)
{
  if(LL_DMA_IsActiveFlag_HT3(DMA1))
  {
		LL_DMA_ClearFlag_GI3(DMA1);

		DoConversionRgbToDmaFirstPart(&wsTxStatus[0]);
  }
	else if(LL_DMA_IsActiveFlag_HT2(DMA1))
  {
		LL_DMA_ClearFlag_GI2(DMA1);

		DoConversionRgbToDmaFirstPart(&wsTxStatus[1]);
  }
	
	else if(LL_DMA_IsActiveFlag_TC3(DMA1))
	{
		LL_DMA_ClearFlag_GI3(DMA1);
	
		DoConversionRgbToDmaSecondPart(&wsTxStatus[0]);
	}
	
	else if(LL_DMA_IsActiveFlag_TC2(DMA1))
	{
		LL_DMA_ClearFlag_GI2(DMA1);
	
		DoConversionRgbToDmaSecondPart(&wsTxStatus[1]);
	}
	
  else if(LL_DMA_IsActiveFlag_TE3(DMA1))
  {
    /* Call Error function */
    TIM1_TransferError_Callback();
  }
	
	/* If first and second cannels transfer complete get new data from uart */
	if(DidTransmission(&wsTxStatus[0]) == 1 && DidTransmission(&wsTxStatus[1]) == 1)
	{
		Timer16DmaStop();
		Timer17DmaStop();
		
		StartUartRxTransfers();
	}
}

/**
* @brief This function handles DMA1 channel 4 and 5 interrupts.
*/
void DMA1_Channel4_5_IRQHandler(void)
{
  if(LL_DMA_IsActiveFlag_TC4(DMA1))
  {
    LL_DMA_ClearFlag_GI4(DMA1);
    USART1_DMA1_TransmitComplete_Callback();
  }
  else if(LL_DMA_IsActiveFlag_TE4(DMA1))
  {
    USART_TransferError_Callback();
  }
  if(LL_DMA_IsActiveFlag_TC5(DMA1))
  {
    LL_DMA_ClearFlag_GI5(DMA1);
    USART1_DMA1_ReceiveComplete_Callback();
  }
  else if(LL_DMA_IsActiveFlag_TE5(DMA1))
  {
    USART_TransferError_Callback();
  }
}

