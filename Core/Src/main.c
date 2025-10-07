/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/*
 TODO : timer, reception/traitement message, eeprom lecture/ecriture, log en flash, pile envoi message lora,
 mesure mode STOP, watchdog, adresses LORA/Uart
 clignot sorties, pwm,  antirebond 2 boutons, 2e uart

 v1.5 10/2025 : eeprom, log_flash, rtc, messages binaires, attente dans en_queue
 v1.4 09/2025 : fct : refonte reception uart, watchdog contextuel, traitement_rx, opti stack
 v1.3 09/2025 : fct : augmentation stack taches, erreur_freertos
 v1.2 09/2025 : pile envoi uart, timer, code_erreur
 v1.1 09/2025 : STM32CubeMX + freertos+ subGhz+ Uart2+ RTC+ print_log+ event_queue
*/

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include <communication.h>
#include <fonctions.h>
#include <eeprom_emul.h>
#include <log_flash.h>
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "timers.h"
#include "queue.h"
#include <stdio.h>    // Pour sprintf
#include <string.h>   // Pour strlen
#include <stdarg.h>   // Pour va_list (si vous utilisez print_log)

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

extern TimerHandle_t HTimer_24h;
extern TimerHandle_t HTimer_20min;
extern osThreadId_t Uart_RX_TaskHandle;
extern osThreadId_t Uart_TX_TaskHandle;


HAL_StatusTypeDef configure_lse_oscillator(void);
HAL_StatusTypeDef configure_lsi_oscillator(void);

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
 QueueHandle_t Event_QueueHandle;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

/* Stack utilisée (en mots de 32 bits)
 création tache sans rien : 48
 LOG_INFO : 58
 LOG %s : 78
 envoie_mess_ASC simple: 64
 envoie_mess_ASC %s : 76
 eeprom : 90
 */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4  // 188 utilisé  +180byte/tache
};
/* Definitions for LORA_RX_Task */
osThreadId_t LORA_RX_TaskHandle;
const osThreadAttr_t LORA_RX_Task_attributes = {
  .name = "LORA_RX_Task",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 256 * 4    // 200 utilisé
};
/* Definitions for LORA_TX_Task */
osThreadId_t LORA_TX_TaskHandle;
const osThreadAttr_t LORA_TX_Task_attributes = {
  .name = "LORA_TX_Task",
  .priority = (osPriority_t) osPriorityLow4,
  .stack_size = 256 * 4    // 208 utilisé
};
/* Definitions for Appli_Task */
osThreadId_t Appli_TaskHandle;
const osThreadAttr_t Appli_Task_attributes = {
  .name = "Appli_Task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 480 * 4   // 222 utilisé
};

/* USER CODE BEGIN PV */

uint8_t test_val=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SUBGHZ_Init(void);
static void MX_RTC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_IWDG_Init(void);
void StartDefaultTask(void *argument);
void LORA_RXTsk(void *argument);
void LORA_TXTsk(void *argument);
void Appli_Tsk(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_SUBGHZ_Init();
  MX_RTC_Init();
  MX_USART2_UART_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  //LOG_INFO("=== RAK3172 LoRa System Started ===");
  //LOG_INFO("Log level: %d", get_log_level());

  char init_msg[] = "-- RAK3172 Init. Log level:x\r";
  init_msg[0] = dest_log;
  init_msg[1] = My_Address;
  init_msg[27] = get_log_level()+'0';
  uint16_t len = strlen(init_msg);
  HAL_UART_Transmit(&huart2, (uint8_t*)init_msg, len, 3000);
  HAL_Delay(500);

  // Afficher la cause du reset dès le démarrage
  //display_reset_cause();

  // configure l'oscillateur
 /* if (configure_lsi_oscillator() == HAL_OK) {
        LOG_INFO("LSI configured (no TCXO)");
  } else {
         LOG_ERROR("No RTC oscillator available");
            }*/
  /* Essayer LSE d'abord (TCXO)
      if (configure_lse_oscillator() == HAL_OK) {
          LOG_INFO("TCXO detected and configured");
      } else {
          // Fallback sur LSI
          if (configure_lsi_oscillator() == HAL_OK) {
              LOG_INFO("LSI configured (no TCXO)");
          } else {
              LOG_ERROR("No RTC oscillator available");
          }
      }*/


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  //const char* init_msg2 = "suite\r\n";
  //len = strlen(init_msg2);
  //HAL_UART_Transmit(&huart2, (uint8_t*)init_msg2, len, 3000);
  //HAL_Delay(100);
  //char lsi_status[20];
  //sprintf(lsi_status, "LSI status: %s", __HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY) ? "Ready" : "Not Ready");
  //len = strlen(lsi_status);
  //HAL_UART_Transmit(&huart2, (uint8_t*)lsi_status, len, 3000);
  //HAL_Delay(100);

  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Event_Queue */
  Event_QueueHandle = xQueueCreate(32, sizeof(uint16_t));

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  size_t freeHeap = xPortGetFreeHeapSize();
  char msgL[50];
  sprintf(msgL, "Free heap before tasks: %i bytes\r", freeHeap);
  HAL_Delay(500);
  HAL_UART_Transmit(&huart2, (uint8_t*)msgL, strlen(msgL), 3000);
  HAL_Delay(500);

  init_communication();


  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of LORA_RX_Task */
  LORA_RX_TaskHandle = osThreadNew(LORA_RXTsk, NULL, &LORA_RX_Task_attributes);

  /* creation of LORA_TX_Task */
  LORA_TX_TaskHandle = osThreadNew(LORA_TXTsk, NULL, &LORA_TX_Task_attributes);

  /* creation of Appli_Task */
  Appli_TaskHandle = osThreadNew(Appli_Tsk, NULL, &Appli_Task_attributes);

  uint8_t code_err_tache=0;
  if (defaultTaskHandle == NULL) code_err_tache=1;
  if (LORA_RX_TaskHandle == NULL) code_err_tache|=2;
  if (LORA_TX_TaskHandle == NULL) code_err_tache|=4;
  if (Appli_TaskHandle == NULL) code_err_tache|=8;
  if (Uart_RX_TaskHandle == NULL) code_err_tache|=16;
  if (Uart_TX_TaskHandle == NULL) code_err_tache|=32;

  if (code_err_tache) {
	   HAL_Delay(500);
	   sprintf(msgL, "erreur tache: %02X ", code_err_tache);
	  HAL_UART_Transmit(&huart2, (uint8_t*)msgL, strlen(msgL), 3000);
	   HAL_Delay(500);
      //LOG_ERROR("Failed to create Appli_Task");
      // La tâche n'a pas pu être créée
  }
  // Après la création :
   freeHeap = xPortGetFreeHeapSize();
  sprintf(msgL, "Free heap after tasks: %i bytes\r", freeHeap);
  HAL_Delay(500);
  HAL_UART_Transmit(&huart2, (uint8_t*)msgL, strlen(msgL), 3000);
  HAL_Delay(500);

  /* USER CODE BEGIN RTOS_THREADS */
  init_functions();

  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  //HAL_UART_Transmit(&huart2, (uint8_t*)"InitS", 5, 3000);
  //HAL_Delay(500);

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.LSIDiv = RCC_LSI_DIV1;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */
	#ifdef WATCHDOG

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Window = 0xfff;
  hiwdg.Init.Reload = 0xfff;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */
	#endif

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  hrtc.Init.BinMode = RTC_BINARY_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SUBGHZ Initialization Function
  * @param None
  * @retval None
  */
static void MX_SUBGHZ_Init(void)
{

  /* USER CODE BEGIN SUBGHZ_Init 0 */

  /* USER CODE END SUBGHZ_Init 0 */

  /* USER CODE BEGIN SUBGHZ_Init 1 */

  /* USER CODE END SUBGHZ_Init 1 */
  hsubghz.Init.BaudratePrescaler = SUBGHZSPI_BAUDRATEPRESCALER_8;
  if (HAL_SUBGHZ_Init(&hsubghz) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SUBGHZ_Init 2 */

  // Configuration LoRa
  // Fréquence 868 MHz
  uint8_t freq_config[] = {0x00, 0x00, 0x00, 0x00}; // À ajuster selon votre fréquence
  HAL_SUBGHZ_WriteRegisters(&hsubghz, 0x08, freq_config, 4);

  // Configuration LoRa (SF7, BW125, CR4/5)
  uint8_t lora_config[] = {0x72, 0x74, 0x00};
  HAL_SUBGHZ_WriteRegisters(&hsubghz, 0x1E, lora_config, 3);

  //LOG_INFO("SUBGHZ LoRa configured");

  /* USER CODE END SUBGHZ_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : bouton_Pin */
  GPIO_InitStruct.Pin = bouton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(bouton_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



/**
 * @brief Configuration de LSE (TCXO externe)
 * @retval HAL_StatusTypeDef: Statut de la configuration
 */
HAL_StatusTypeDef configure_lse_oscillator(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    HAL_StatusTypeDef status;

    LOG_DEBUG("Attempting to configure LSE (TCXO)");

    // Configuration LSE
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

    status = HAL_RCC_OscConfig(&RCC_OscInitStruct);

    if (status == HAL_OK) {
        // Vérifier que LSE est vraiment actif
        uint32_t timeout = 1000;
        while (!__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) && timeout > 0) {
            osDelay(1);
            timeout--;
        }

        if (__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY)) {
            LOG_INFO("LSE (TCXO) configured successfully");
            return HAL_OK;
        } else {
            LOG_WARNING("LSE startup timeout");
            return HAL_TIMEOUT;
        }
    } else {
        LOG_WARNING("LSE configuration failed: %d", status);
        return status;
    }
}

/**
 * @brief Configuration de LSI (oscillateur interne)
 * @retval HAL_StatusTypeDef: Statut de la configuration
 */
HAL_StatusTypeDef configure_lsi_oscillator(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    HAL_StatusTypeDef status;

    LOG_DEBUG("Attempting to configure LSI (internal oscillator)");

    // Configuration LSI
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

    status = HAL_RCC_OscConfig(&RCC_OscInitStruct);

    if (status == HAL_OK) {
        // Vérifier que LSI est actif
        uint32_t timeout = 1000;
        while (!__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY) && timeout > 0) {
            osDelay(1);
            timeout--;
        }

        if (__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY)) {
            LOG_INFO("LSI (internal oscillator) configured successfully");
            return HAL_OK;
        } else {
            LOG_WARNING("LSI startup timeout");
            return HAL_TIMEOUT;
        }
    } else {
        LOG_WARNING("LSI configuration failed: %d", status);
        return status;
    }
}


/**
 * @brief Envoyer un événement dans la queue
 * @param type: Type d'événement
 * @param source: Source de l'événement
 * @param data: Données de l'événement
 * @retval osStatus_t: Statut de l'envoi
 */
osStatus_t send_event(uint8_t type, uint8_t source, uint16_t data)
{
    event_t evt;
    evt.type = type;
    evt.source = source;
    evt.data = data;
    if (xQueueSend(Event_QueueHandle, &evt, 0) != pdPASS)
    {
        LOG_ERROR("Event queue full - message lost");
        return osErrorResource;
    }
    return osOK;
}

/**
 * @brief Envoyer un événement simple
 * @param type: Type d'événement
 * @retval osStatus_t: Statut de l'envoi
 */
osStatus_t send_simple_event(uint8_t type)
{
    return send_event(type, SOURCE_SYSTEM, 0);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  // Démarrer la surveillance watchdog pour cette tâche
	#ifdef WATCHDOG
	  watchdog_task_start(WATCHDOG_TASK_DEFAULT);
	  //LOG_INFO("DefaultTask started with watchdog protection");
	#endif
  
  uint32_t last_status_time = 0;
  uint32_t last_save_time = 0;
  
  /* Infinite loop */
  for(;;)
  {
    // Enregistrer un heartbeat pour le watchdog
    watchdog_task_heartbeat(WATCHDOG_TASK_DEFAULT);
    
    uint32_t current_time = HAL_GetTick();
    
    // Afficher le statut du watchdog toutes les 30 secondes
    if (current_time - last_status_time > 10000) {
        //watchdog_print_status();
        last_status_time = current_time;
        //osDelay(3000); // Attendre 3 seconde
		//check_stack_usage();

        uint8_t value;
        if (test_val ==1)
        	LOG_INFO("toto");
        if (test_val == 2)
            LOG_INFO("test:FLASH_PAGE_SIZE: %i octets", 1233330);
        if (test_val == 3)
        {
        	char messa[10];
        	messa[0] = '1';
            messa[1] = 'S';  // Accusï¿½ reception ok : S1
            messa[2] = '1';
            messa[3] = car_fin_trame;
            envoie_mess_ASC((const char*)messa);
        }
        if (test_val == 4)
        {
        	uint8_t messa = 'c';
            envoie_mess_ASC("1te%cVAnal %i", messa, 12);
        }
        if (test_val == 5)
        	EEPROM_Read8(1, &value);
        if (test_val == 6)
        	EEPROM_Write8(1, 12);
        if (test_val == 7)
        	EEPROM_Read8(1, &value);
        if (test_val == 8)
        	log_read(1, 1, '1', 0);
        if (test_val ==9)
        	log_write('T', 1, 0x02, 0x03, "testRxBl");    }
		if (test_val ==10)
			log_read(1, 4, '1', 0);
   	    if (test_val ==11)
		    log_read(1, 1, '1', 1);
    
    // Sauvegarder les données de diagnostic toutes les 60 secondes
    if (current_time - last_save_time > 60000) {
        //save_diagnostic_data();
        last_save_time = current_time;
    }
    HAL_IWDG_Refresh(&hiwdg);
    

    osDelay(1000); // Attendre 1 seconde entre chaque heartbeat
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_LORA_RXTsk */
/**
* @brief Function implementing the LORA_RX_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LORA_RXTsk */
void LORA_RXTsk(void *argument)
{
  /* USER CODE BEGIN LORA_RXTsk */
  // Démarrer la surveillance watchdog pour cette tâche
  watchdog_task_start(WATCHDOG_TASK_LORA_RX);
  //LOG_INFO("LoRa RX Task started with watchdog protection");
  
  /* Infinite loop */
	uint8_t rx_buffer[64];
	uint8_t radio_status;

    osDelay(100);

	for(;;)
	{
		// Enregistrer un heartbeat pour le watchdog
		watchdog_task_heartbeat(WATCHDOG_TASK_LORA_RX);

		// Vérifier l'état du radio
		if (HAL_SUBGHZ_ReadRegister(&hsubghz, 0x01, &radio_status) == HAL_OK) {

			// Vérifier si un message est disponible
			if (radio_status & 0x02) { // Bit RX_DONE
				LOG_DEBUG("LoRa message received");

				// Lire le message depuis le buffer radio
				if (HAL_SUBGHZ_ReadBuffer(&hsubghz, 0x00, rx_buffer, 64) == HAL_OK) {
					LOG_INFO("Received LoRa message: %s", rx_buffer);
		            send_event(EVENT_LORA_RX, SOURCE_LORA, strlen((char*)rx_buffer));
				} else {
					LOG_ERROR("Failed to read LoRa buffer");
				}

				// Effacer le flag RX_DONE
				HAL_SUBGHZ_WriteRegister(&hsubghz, 0x01, 0x02);
			} else {
				LOG_VERBOSE("No LoRa message");
			}
		} else {
			LOG_WARNING("Failed to read radio status");
		}

		osDelay(1000);
	}
  /* USER CODE END LORA_RXTsk */
}

/* USER CODE BEGIN Header_LORA_TXTsk */
/**
* @brief Function implementing the LORA_TX_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LORA_TXTsk */
void LORA_TXTsk(void *argument)
{
  /* USER CODE BEGIN LORA_TXTsk */
  // Démarrer la surveillance watchdog pour cette tâche
  watchdog_task_start(WATCHDOG_TASK_LORA_TX);
  //LOG_INFO("LoRa TX Task started with watchdog protection");
  
  /* Infinite loop */

  uint32_t message_count = 0;
  uint8_t tx_buffer[64];
  uint8_t radio_status;

  osDelay(100);

  for(;;)
  {
	  // Enregistrer un heartbeat pour le watchdog
	  watchdog_task_heartbeat(WATCHDOG_TASK_LORA_TX);
	  
	  // Attendre un délai
	  osDelay(12000);
	  //LOG_INFO("a");

	  // Créer un message avec timestamp
	  uint32_t timestamp = HAL_GetTick() / 1000; // secondes
	  sprintf((char *)tx_buffer, "LoRa message #%lu at %lu s", message_count++, timestamp);

       //LOG_DEBUG("Sending LoRa message: %s", tx_buffer);

       // Vérifier que le radio est libre
       if (HAL_SUBGHZ_ReadRegister(&hsubghz, 0x01, &radio_status) == HAL_OK) {
           if (!(radio_status & 0x01)) { // Pas en transmission

               // Écrire le message dans le buffer radio
               if (HAL_SUBGHZ_WriteBuffer(&hsubghz, 0x00, tx_buffer, strlen((char*)tx_buffer)) == HAL_OK) {

                   // Démarrer la transmission
                   /*uint8_t tx_cmd = 0x83; // Commande TX
                   if (HAL_SUBGHZ_ExecSetCmd(&hsubghz, tx_cmd, NULL, 0) == HAL_OK)
                   {
                       LOG_INFO("LoRa transmission started");

                       // Attendre la fin de transmission
                       osDelay(100);

                       // Vérifier le statut
                       if (HAL_SUBGHZ_ReadRegister(&hsubghz, 0x01, &radio_status) == HAL_OK) {
                           if (radio_status & 0x08) { // TX_DONE
                               LOG_INFO("LoRa message sent successfully");
                               send_event(EVENT_LORA_TX, SOURCE_LORA, message_count);
                           } else {
                               LOG_ERROR("LoRa transmission failed");
                               send_event(EVENT_ERROR, SOURCE_LORA, 1);
                           }
                       }
                   } else {
                       LOG_ERROR("Failed to start LoRa transmission");
                   }*/
               } else {
                   LOG_ERROR("Failed to write LoRa buffer");
               }
           } else {
               LOG_WARNING("LoRa radio busy");
           }
       } else {
           LOG_ERROR("Failed to read radio status");
       }
  }
  /* USER CODE END LORA_TXTsk */
}

/* USER CODE BEGIN Header_Appli_Tsk */
/**
* @brief Function implementing the Appli_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Appli_Tsk */
void Appli_Tsk(void *argument)
{
  /* USER CODE BEGIN Appli_Tsk */
  // Démarrer la surveillance watchdog pour cette tâche
	   //HAL_UART_Transmit(&huart2, (uint8_t*)"InitA", 5, 3000);
	   //HAL_Delay(500);

  watchdog_task_start(WATCHDOG_TASK_APPLI);
  //LOG_INFO("Appli_Task started with watchdog protection");

  //HAL_UART_Transmit(&huart2, (uint8_t*)"InitB", 5, 3000);
  //HAL_Delay(500);

    event_t evt;
    osStatus_t status;

    osDelay(1000);

    // Vérifier la configuration flash
    //check_flash_config();
    //check_flash_permissions();
    //osDelay(1000);

    osDelay(1000);

    if (EEPROM_Init() == HAL_OK)
            LOG_INFO("EEPROM initialisee");
         else
            LOG_ERROR("Erreur EEPROM");

    /*if (EEPROM_Init() == HAL_OK) {
        LOG_INFO("eeprom initialisee");
        osDelay(1000);
        // Test simple
        test_eeprom_simple();
    } else {
        LOG_ERROR("Erreur eeprom");
    }*/

    for(;;)
    {
        // Enregistrer un heartbeat pour le watchdog
        watchdog_task_heartbeat(WATCHDOG_TASK_APPLI);
        
        watchdog_set_context(WATCHDOG_TASK_APPLI, WATCHDOG_CONTEXT_WAITING);

        // Attendre un événement (bloque tant qu'il n'y a rien)
        status = osMessageQueueGet(Event_QueueHandle, &evt, NULL, osWaitForever);

        watchdog_set_context(WATCHDOG_TASK_APPLI, WATCHDOG_CONTEXT_ACTIVE);

        if (status == osOK)
        {
			// Traiter l'événement reçu
			//LOG_DEBUG("Processing event: type=%d, source=%d, data=%d",
			//		  evt.type, evt.source, evt.data);
			//osDelay(30);

			switch (evt.type) {

				case EVENT_BUTTON: {
					LOG_INFO("Button pressed event");
					// Actions pour bouton pressé
					HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1); // Toggle LED

					// Envoyer message LoRa
					char messa[] = "Button pressed!";
					send_lora_message((const char*)messa, 16, 'Q');
					break;
				}

				case EVENT_LORA_RX: {
					LOG_INFO("LoRa message received event");
					// Actions pour message LoRa reçu
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); // LED ON
					osDelay(500);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // LED OFF
					break;
				}

				case EVENT_LORA_TX: {
					LOG_INFO("LoRa message sent event");
					// Actions pour message LoRa envoyé
					for (int i = 0; i < 3; i++) {
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
						osDelay(100);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
						osDelay(100);
					}
					break;
				}

				case EVENT_UART_RX: {
					//LOG_INFO("UART message received event");
					in_message_t message_in;
			        if (xQueueReceive(in_message_queue, &message_in, portMAX_DELAY) == pdPASS)
			        {
			            // Traiter le message selon son type
						reception_message_Uart2(&message_in);
			        }
					break;
				}


				case EVENT_ERROR: {
					LOG_ERROR("Error event - data: %d", evt.data);
					// Actions pour erreur
					for (int i = 0; i < 5; i++) {
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
						osDelay(50);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
						osDelay(50);
					}
					break;
				}

				case EVENT_WAKE_UP: {
					LOG_INFO("Wake up event");
					// Actions pour réveil
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
					osDelay(100);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
					break;
				}

				case EVENT_SLEEP: {
					LOG_INFO("Sleep event");
					// Actions avant sleep
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
					osDelay(100);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
					break;
				}

				case EVENT_SYSTEM_RESET: {
					LOG_INFO("System reset event");
					// Actions avant reset
					for (int i = 0; i < 10; i++) {
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
						osDelay(50);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
						osDelay(50);
					}
					osDelay(1000);
					HAL_NVIC_SystemReset();
					break;
				}
				case EVENT_WATCHDOG_CHECK: {
				    watchdog_check_all_tasks();
				    break;
				}
				case EVENT_TIMER_24h: {
					envoie_mess_ASC("1Message periodique 24h\r\n");

					// Debug : afficher le temps restant avant la prochaine expiration
					TickType_t expiry = xTimerGetExpiryTime(HTimer_24h);
					TickType_t now = xTaskGetTickCount();
					LOG_INFO("Il reste %lu ticks avant la prochaine expiration\n",
						   (expiry > now) ? (expiry - now) : 0);
					break;
				}
				case EVENT_TIMER_20min: {

					//LOG_INFO("a");
					//LOG_INFO("TIMER 20s event");
					//osDelay(1000);
					//check_stack_usage();
					//uint8_t statut=envoie_mess_ASC("Message periodique 30s");
					//if (statut) { code_erreur= code_erreur_envoi; err_donnee1=statut;}
					//uint8_t statut=envoie_mess_ASC("PRS");
					//osDelay(100);
					//LOG_INFO("statut:%d", statut);
					//osDelay(1000);

					// Debug : afficher le temps restant avant la prochaine expiration
					//TickType_t expiry = xTimerGetExpiryTime(HTimer_20min);
					//TickType_t now = xTaskGetTickCount();
					osDelay(100);
					//LOG_INFO("Il reste %lu ticks avant la prochaine expir\n",
					//	   (expiry > now) ? (expiry - now) : 0);
					//osDelay(100);
					break;
				}
				case EVENT_UART_RAZ: {
					LOG_WARNING("RX Timeout for UART %i", evt.source);
					code_erreur=timeout_RX;   //timeout apres 1 car Recu
				    err_donnee1= evt.source+'0';
				    raz_Uart(evt.data);
				    break;
				}

				default: {
					LOG_WARNING("Unknown event type: %d", evt.type);
					break;
				}
			}
	        if (code_erreur)
	            envoi_code_erreur();
        }
        else {
            LOG_ERROR("Failed to receive event: %d", status);
        }
        osDelay(100);
    }

  /* USER CODE END Appli_Tsk */
}

/* USER CODE BEGIN Header_Uart1_Tsk */
/* USER CODE END Header_Uart1_Tsk */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_0)
    {
        LOG_DEBUG("Button pressed on PA0");

        if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET)
        {
            // Envoyer événement bouton pressé
            send_event(EVENT_BUTTON, SOURCE_BUTTON, 1);
        }
    }
}


/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM16 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM16)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();

  const char* error_msg = "ERROR: System failure\r\n";
  uint16_t len = strlen(error_msg);

  // Envoi direct via HAL_UART
  HAL_UART_Transmit(&huart2, (uint8_t*)error_msg, len, 3000);

  #ifdef DEBUG
		// Mode debug : boucle infinie pour debugging
		__disable_irq();
		while (1)
		{
			const char* debug_msg = "DEBUG: System halted\r\n";
			HAL_UART_Transmit(&huart2, (uint8_t*)debug_msg, strlen(debug_msg), 100);
			HAL_Delay(1000);
		}
	#else
		// Mode production : reset automatique
		const char* error_s_msg = "ERROR: System failure - Resetting\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t*)error_s_msg, strlen(error_s_msg), 100);
		HAL_Delay(100);
		HAL_NVIC_SystemReset();
	#endif

  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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

void assert_failed(const char *file, int line)
{
    char msg[100];
    int len = snprintf(msg, sizeof(msg), "-- ASSERT failed at %s:%d\r\n", file, line);
    msg[0] = dest_log;
    msg[1] = My_Address;
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, HAL_MAX_DELAY);

    HAL_Delay(2000);

    // Reset du système
    NVIC_SystemReset();
    // Bloquer ici
    //taskDISABLE_INTERRUPTS();
    //for(;;);
}

