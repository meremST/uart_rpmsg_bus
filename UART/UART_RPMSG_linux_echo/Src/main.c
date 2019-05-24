/**
  ******************************************************************************
  * @file    UART/UART_Printf/Src/main.c 
  * @author  MCD Application Team
  * @brief   This example shows how to retarget the C library printf function 
  *          to the UART.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup UART_Printf
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
#define RPMSG_NAME_SIZE	(32)

struct serdev_rproc_hdr {
	uint16_t magic_number; //0xbe57 #beST
	uint16_t len;
	uint8_t msg_type;
} __attribute__((__packed__));

struct rpmsg_hdr {
	uint32_t src;
	uint32_t dst;
	uint32_t reserved;
	uint16_t len;
	uint16_t flags;
	uint8_t data[0];
} __attribute__((__packed__));

struct rpmsg_ns_msg {
	char name[RPMSG_NAME_SIZE];
	uint32_t addr;
	uint32_t flags;
} __attribute__((__packed__));

enum rpmsg_ns_flags {
	RPMSG_NS_CREATE		= 0,
	RPMSG_NS_DESTROY	= 1,
};

enum serdev_rproc_type {
	SERDEV_RPROC_RPMSG	= 0,
	SERDEV_RPROC_ANNOUNCE	= 1,
};


/* Private define ------------------------------------------------------------*/
/* Error macros. */
#define RPMSG_SUCCESS		0
#define RPMSG_ERROR_BASE	-2000
#define RPMSG_ERR_NO_MEM	(RPMSG_ERROR_BASE - 1)
#define RPMSG_ERR_NO_BUFF	(RPMSG_ERROR_BASE - 2)
#define RPMSG_ERR_PARAM		(RPMSG_ERROR_BASE - 3)
#define RPMSG_ERR_DEV_STATE	(RPMSG_ERROR_BASE - 4)
#define RPMSG_ERR_BUFF_SIZE	(RPMSG_ERROR_BASE - 5)
#define RPMSG_ERR_INIT		(RPMSG_ERROR_BASE - 6)
#define RPMSG_ERR_ADDR		(RPMSG_ERROR_BASE - 7)
#define RPMSG_ERR_TRANSMIT	(RPMSG_ERROR_BASE - 8)


#define	SERDEV_RPMSG_MAGIC_NUMBER	(0xbe57)
#define RPMSG_MAX_BUF_SIZE	(512)
#define RPMSG_ADDR_ANY	(0xFFFFFFFF)
#define RPMSG_NS_ADDR			(0x35)
#define RPMSG_SERVICE_NAME	"rpmsg-misc-channel"//"st,rpmsg-i2c"

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
struct serdev_rproc_hdr *serdev_hdr = NULL;
struct rpmsg_hdr *msg = NULL;

/* UART handler declaration */
UART_HandleTypeDef UartHandle;
__IO ITStatus UartReady = RESET;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);

/* Private functions ---------------------------------------------------------*/
static inline int uart_rpmsg_send(UART_HandleTypeDef *UartHandle,uint32_t src, uint32_t dst,const void *data,int len)
{
	struct serdev_rproc_hdr s_hdr;
	struct rpmsg_hdr *msg_container = malloc(RPMSG_MAX_BUF_SIZE);
	if(!msg_container)
		return RPMSG_ERR_NO_MEM;

	s_hdr.magic_number = SERDEV_RPMSG_MAGIC_NUMBER;
	s_hdr.len = len + sizeof(struct rpmsg_hdr);
	s_hdr.msg_type = SERDEV_RPROC_RPMSG;

	msg_container->src = src;
	msg_container->dst = dst;
	msg_container->flags = 0;
	msg_container->len = len;
	msg_container->reserved = 0;

	memcpy(msg_container->data,data,len);

	if(HAL_UART_Transmit(UartHandle, (uint8_t *)&s_hdr, sizeof(struct serdev_rproc_hdr), 0xFFFF) != HAL_OK){
	  	  free(msg_container);
	  	  return RPMSG_ERR_TRANSMIT;
	  }

	  if(HAL_UART_Transmit(UartHandle, (uint8_t *)msg_container, s_hdr.len, 0xFFFF) != HAL_OK){
		  free(msg_container);
		  return RPMSG_ERR_TRANSMIT;
	  }

	free(msg_container);
	return RPMSG_SUCCESS;
}

int uart_rpmsg_send_ns_message(UART_HandleTypeDef *UartHandle, char *name, uint32_t addr, unsigned long flags)
{
	struct rpmsg_ns_msg ns_msg;
	int ret;

	ns_msg.flags = flags;
	ns_msg.addr = addr;
	strncpy(ns_msg.name, name, sizeof(ns_msg.name));
	ret = uart_rpmsg_send(UartHandle, addr,
					RPMSG_NS_ADDR,
					&ns_msg, sizeof(ns_msg));
	if (ret < 0)
		return ret;
	else
		return RPMSG_SUCCESS;
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  uint8_t serdev_hdr_size = sizeof(struct serdev_rproc_hdr);
  struct rpmsg_ns_msg ns_msg;
  struct serdev_rproc_hdr s_hdr;
  uint32_t temp_src;
  HAL_Init();
  
  /* Configure the system clock to 100 MHz */
  SystemClock_Config();

  BSP_LED_Init(LED2);
   
  /*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART1 configured as follow:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = ODD parity
      - BaudRate = 9600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  UartHandle.Instance          = USARTx;
  UartHandle.Init.BaudRate     = 57600; //57600 max-speed
  UartHandle.Init.WordLength   = UART_WORDLENGTH_9B;
  UartHandle.Init.StopBits     = UART_STOPBITS_1;
  UartHandle.Init.Parity       = UART_PARITY_ODD;
  UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode         = UART_MODE_TX_RX;
  UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
    
  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  }
  
  /*corresspond to ept_init*/
  msg = malloc(RPMSG_MAX_BUF_SIZE);
  serdev_hdr = malloc(sizeof(struct serdev_rproc_hdr));

  /*ept creationj*/
  if(uart_rpmsg_send_ns_message(&UartHandle, RPMSG_SERVICE_NAME, 0x1, RPMSG_NS_CREATE)!= RPMSG_SUCCESS){
	  Error_Handler();
  }

  /*launch answer reception*/
  BSP_LED_On(LED2);
  if(HAL_UART_Receive_IT(&UartHandle, (uint8_t *)serdev_hdr, serdev_hdr_size) != HAL_OK){
	  /* Initialization Error */
	  Error_Handler();
  }

  /*end of ept_init*/

  while(1){

	  /*Waiting util uart an message is received*/
	  while (UartReady != SET);
	  UartReady = RESET;


	  if(uart_rpmsg_send(&UartHandle, msg->dst, msg->src,msg->data,msg->len) != RPMSG_SUCCESS){
		  Error_Handler();
	  }

	  if(HAL_UART_Receive_IT(&UartHandle, (uint8_t *)serdev_hdr, serdev_hdr_size) != HAL_OK){
	  	  /* Initialization Error */
		  Error_Handler();
	  }

  }

  free(msg);
  free(serdev_hdr);

}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of IT Rx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	const uint8_t serdev_hdr_size = sizeof(struct serdev_rproc_hdr);
	/* Set transmission flag: transfer complete */
	if(serdev_hdr->magic_number == SERDEV_RPMSG_MAGIC_NUMBER){
		if(HAL_UART_Receive(UartHandle,(uint8_t *)msg,serdev_hdr->len,0xFFFF)){
			Error_Handler();
		}
		BSP_LED_Toggle(LED2);
		UartReady = SET;
	}

	/*if(HAL_UART_Receive_IT(&UartHandle, (uint8_t *)&serdev_hdr, serdev_hdr_size) != HAL_OK){
		Error_Handler();
	}*/
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 100000000
  *            HCLK(Hz)                       = 100000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 16000000
  *            PLL_M                          = 16
  *            PLL_N                          = 400
  *            PLL_P                          = 4
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale2 mode
  *            Flash Latency(WS)              = 3
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  
  /* Enable HSI Oscillator and activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED2 on */
  while(1)
  {
	  BSP_LED_Toggle(LED2);
	  /* Wait for 75ms */
	  HAL_Delay(75);
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

