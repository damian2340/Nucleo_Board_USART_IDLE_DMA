
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f3xx_hal.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
//extern uint8_t dma_uart_tx_buffer[DMA_TX_BUFFER_SIZE];
extern uint8_t sdin_buffer[SDIN_BUFFER_SIZE] ;
//extern UART_HandleTypeDef huart2;
//extern volatile size_t dataToSend;
extern volatile BufferStateTypeDef SdInBufferState;
extern cBufferHandler_t TxbufferHandler;
extern cBufferHandler_t DataBufferHandler;
extern volatile uint32_t uwTick;
extern elapsedTime_t pulseTimeBuffer[PULSE_BUFFER_SIZE];
volatile uint32_t pulseWithBuffer[32];
volatile uint32_t pulseDelayBuffer[32];

uint8_t bufferFlag;
uint8_t newConfigFlag;


typedef enum TASK_STATE{
	Stopped =   0x0,
	Idle 	=	0x1,
	Busy 	=	0x2
}taskState;

typedef struct TASK_CONTEXT{
	taskState state;
	taskState (*getState)(struct TASK_CONTEXT * context);
	void (*setState)(struct TASK_CONTEXT * context, taskState state);
	void (*task)( struct TASK_CONTEXT * context );
}taskContext_t;

typedef enum PARSE_STATE{
	StateParseSTOPPED = 0x00,
	StateParseIDLE    = 0x01,
	StateParseBUSY0   = 0x02,
	StateParseERROR   = 0x04,
}parseState_t;

typedef struct PARSE_CONTEXT{
	parseState_t ParseState;

}parseContext_t;

typedef enum COMM_STATE{
	StateCommSTOPPED 	= 0x00,
	StateCommIDLE     	= 0x01,
	StateCommBUSY0   	= 0x02,
	StateSetDataBuffer	= 0x04,
	StateSetSdouBuffer	= 0x08,
	StateResetDataBuffer= 0x0F,
	StateResetSdoBuffer = 0x10,
	StateCommERROR   	= 0x40,
}commState_t;

typedef struct COMM_CONTEXT{
	commState_t CommState;
	cBufferHandler_t * CurrentBuffer;
}commContext_t;

commState_t requestState = StateCommIDLE;
extern bufferType_t CurrentBufferType;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
taskState getTaskState(taskContext_t * context);
void setTaskState(taskContext_t * context, taskState state);

void parse_task();
void parseInitialize(parseContext_t * parseContext);

void comm_task();
void commInitialize(commContext_t * commContext);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uwTick = 0;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  //volatile uint32_t newData = 0 ;
  while(uwTick<100){}

//  TransmitionStart(&TxbufferHandler);
  ReceptionStart();


  taskContext_t task[3];
  task[0].getState = getTaskState;
  task[0].setState = setTaskState;
  task[1].getState = getTaskState;
  task[1].setState = setTaskState;
  task[2].getState = getTaskState;
  task[2].setState = setTaskState;

  //taskContext_t* parseTask = &(task[0]);
  task[0].state = Stopped;
  task[0].task = parse_task;
  task[1].state = Stopped;
  task[1].task = comm_task;


  uint32_t i = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	  switch( task[0].getState(&task[0]) ) {
		  case Stopped:
			  task[0].setState(&task[0], Idle);
			  break;
		  case Idle:
			  task[0].task(&task[0]);
			  break;
		  default: break;
	  }

	  switch( task[1].getState(&task[1]) ) {
	  case Stopped:
		  task[1].setState(&task[1], Idle);
		  break;
	  case Idle:
		  task[1].task(&task[1]);
		  break;
	  default: break;
	  }

	  if(i>50000){
		  i=0;
		  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_1);
	  }
	  i++;

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void HAL_SYSTICK_Callback(void){
	/* Transmitiendo a 115200baudios con 8 bits de datos y uno de stop sin paridad, se necesitan 5ms para
	 * 64 bytes. Si la transmision anterior no termino, se esperan 5ms mas.
	 **/
	if(uwTick%20 == 0){
		if(DataBufferHandler.BufferDataLength > 0){
			printf("%d\t%d\t%d\t%d\r\n",
					(int)((elapsedTime_t*)(DataBufferHandler.BufferStart_p))[DataBufferHandler.BufferStartOffset].pulseWidth ,
					(int)((elapsedTime_t*)(DataBufferHandler.BufferStart_p))[DataBufferHandler.BufferStartOffset].pulseDelay ,
					(int)((elapsedTime_t*)(DataBufferHandler.BufferStart_p))[DataBufferHandler.BufferStartOffset].indice,
					(int)DataBufferHandler.BufferDataLength);

			DataBufferHandler.BufferStartOffset++;
			/* Si los datos a enviar NO dan la vuelta al buffer */
			if( DataBufferHandler.BufferStartOffset == DataBufferHandler.BufferLength ){
				/* Si el ultimo dato enviado esta al final del buffer la posicion siguiente esta en el inicio del buffer*/
				DataBufferHandler.BufferStartOffset = 0;
			}
			DataBufferHandler.BufferNextStartOffset = DataBufferHandler.BufferStartOffset;
			/* vaciado de datos */
			DataBufferHandler.BufferDataLength--;
		}
		else{
//			printf("esperando datos...\n\r");
		}
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	static elapsedTime_t pulseData;
	if(htim->Instance == TIM2){
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
			pulseData.pulseDelay = htim->Instance->CCR1;
			pulseData.indice++;
			pulseData.pulseWidth = 0U;

		}else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
			pulseData.pulseWidth = htim->Instance->CCR2;
			savePulseData((uint8_t*)&pulseData);
		}
	}
}


taskState getTaskState(taskContext_t * context){
	return context->state;
}

void setTaskState(taskContext_t * context, taskState state){
	context->state = state;
}


void parse_task(){
	static parseContext_t parseContext = {.ParseState = StateParseSTOPPED,};
	switch(parseContext.ParseState){
	case StateParseSTOPPED:
			parseInitialize(&parseContext);
			HAL_TIM_Base_Start(&htim2);
			break;
	case StateParseIDLE:
			if(sdinIsBusy() == BUFFER_busy){
				parseContext.ParseState = StateParseBUSY0;
			}
			break;
	case StateParseBUSY0:
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

			if(sdin_buffer[0] == 'S' || sdin_buffer[0] == 's'){
				if(sdin_buffer[2] == 'A' || sdin_buffer[2] == 'a'){
					requestState = StateSetDataBuffer;
					printf("\nstatr\n\r");
					__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC1 | TIM_IT_CC2 );
					  /* Enable the Input Capture channel */
					TIM_CCxChannelCmd(htim2.Instance, TIM_CHANNEL_2, TIM_CCx_ENABLE);
					TIM_CCxChannelCmd(htim2.Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);
					/* Enable the Peripheral */
					__HAL_TIM_ENABLE(&htim2);

				}
				else if(sdin_buffer[2] == 'O' || sdin_buffer[2] == 'o'){
					requestState = StateSetSdouBuffer;
					printf("\nstop\n\r");
					__HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC2 );
					__HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC1 );
					  /* Enable the Input Capture channel */
					TIM_CCxChannelCmd(htim2.Instance, TIM_CHANNEL_2, TIM_CCx_DISABLE);
					TIM_CCxChannelCmd(htim2.Instance, TIM_CHANNEL_1, TIM_CCx_DISABLE);
					/* Disable the Peripheral */
					__HAL_TIM_DISABLE(&htim2);

				}
			}

			printf("Receive complete: %s \n\n\r", sdin_buffer);
			SdInBufferState = BUFFER_idle;
			parseContext.ParseState = StateParseIDLE;
			break;
	case StateParseERROR:
			printf("\n\n\rParse Error: %s \n\n\r", sdin_buffer);
			break;

	}

}

void parseInitialize(parseContext_t * parseContext){
	parseContext->ParseState = StateParseIDLE;
}

void parseBusy0(){
	  if(sdinIsBusy() == BUFFER_busy){
		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		  printf("\n\n\rReceive complete: %s \n\n\r", sdin_buffer);
		  SdInBufferState = BUFFER_idle;
	  }
}

void comm_task(){
	static commContext_t commContext = {.CommState = StateCommSTOPPED,};
	switch(commContext.CommState){
		case StateCommSTOPPED:
			commInitialize(&commContext);
			break;
		case StateCommIDLE:
			if(commContext.CurrentBuffer->BufferState == BUFFER_idle){
				commContext.CommState = requestState;
				if(commContext.CommState == StateCommIDLE){
					if(commContext.CurrentBuffer->BufferDataLength > 0){
						TransmitionStart(commContext.CurrentBuffer);
					}
				}
				else{
					requestState = StateCommIDLE;
				}
			}
			break;
		case StateCommBUSY0:
			break;
		case StateSetDataBuffer:
			if(commContext.CurrentBuffer->BufferState == BUFFER_idle){
//				commContext.CurrentBuffer = &DataBufferHandler;
//				CurrentBufferType = dataBufferType;
				commContext.CommState = StateCommIDLE;
			}
			break;
		case StateSetSdouBuffer:
			if(commContext.CurrentBuffer->BufferState == BUFFER_idle){
				commContext.CurrentBuffer = &TxbufferHandler;
				CurrentBufferType = sdoutBufferType;
				commContext.CommState = StateCommIDLE;
			}
			break;
		case StateResetDataBuffer:
			if(commContext.CurrentBuffer->BufferState == BUFFER_idle){
				resetPulseBuffer();
				commContext.CommState = StateCommIDLE;
			}
			break;
		case StateResetSdoBuffer:
			if(commContext.CurrentBuffer->BufferState == BUFFER_idle){
				commContext.CommState = StateCommIDLE;
			}
			break;
		case StateCommERROR:
			break;
		}

}

void commInitialize(commContext_t * commContext){
	commContext->CommState = StateCommIDLE;
	commContext->CurrentBuffer = &TxbufferHandler;
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
