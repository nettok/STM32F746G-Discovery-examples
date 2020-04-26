/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "lwip.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "stdarg.h"
#include "api.h"
#include "sntp.h"
#include "mqtt.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 2048 * 4
};
/* Definitions for udpEchoServer */
osThreadId_t udpEchoServerHandle;
const osThreadAttr_t udpEchoServer_attributes = {
  .name = "udpEchoServer",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 2048 * 4
};
/* Definitions for debugHeartbeatTimer */
osTimerId_t debugHeartbeatTimerHandle;
const osTimerAttr_t debugHeartbeatTimer_attributes = {
  .name = "debugHeartbeatTimer"
};
/* Definitions for usbCdcMutex */
osMutexId_t usbCdcMutexHandle;
const osMutexAttr_t usbCdcMutex_attributes = {
  .name = "usbCdcMutex"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartDefaultTask(void *argument);
void EchoServerTask(void *argument);
void DebugHeartbeatCallback(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void debug(const char* format, ...) {
  //if (osMutexAcquire(usbCdcMutexHandle, osWaitForever) == osOK) {
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
//    osMutexRelease(usbCdcMutexHandle);
//  }
}

const int MAX_USBD_BUSY_LOOPS = 1000;
int busyLoopCounter = 0;

int _write(int file, char *ptr, int len) {
  busyLoopCounter = 0;
  while (CDC_Transmit_FS((uint8_t*)ptr, len) == USBD_BUSY) {
    busyLoopCounter++;
    if (busyLoopCounter >= MAX_USBD_BUSY_LOOPS) {
      return 0;
    } else if (busyLoopCounter % 20 == 0) {
      osThreadYield();
    }
  }
  return len;
}
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of usbCdcMutex */
  usbCdcMutexHandle = osMutexNew(&usbCdcMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of debugHeartbeatTimer */
  debugHeartbeatTimerHandle = osTimerNew(DebugHeartbeatCallback, osTimerPeriodic, NULL, &debugHeartbeatTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of udpEchoServer */
  udpEchoServerHandle = osThreadNew(EchoServerTask, NULL, &udpEchoServer_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
 
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

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
  /* init code for LWIP */
  MX_LWIP_Init();

  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  //osTimerStart(debugHeartbeatTimerHandle, 3000);

//  sntp_setserver(idx, server)

  osDelay(3000); // wait for ethernet and DHCP to settle

  ip4_addr_t ntp_server_addr;
  netconn_gethostbyname("pool.ntp.org", &ntp_server_addr);

  sntp_init();

  mqtt_client_t *mqtt_client = mqtt_client_new();
  struct mqtt_connect_client_info_t mqtt_client_info;
  mqtt_client_info.client_id = "stm32f746";
  mqtt_client_info.client_user = NULL;
  mqtt_client_info.client_pass = NULL;
  mqtt_client_info.keep_alive = 1;
  mqtt_client_info.will_msg = NULL;
  mqtt_client_info.will_qos = 0;
  mqtt_client_info.will_retain = 0;
  mqtt_client_info.will_topic = NULL;
  ip4_addr_t revmac_addr;
  IP4_ADDR(&revmac_addr, 192, 168, 0, 31);
  err_t mqtt_connect_result = mqtt_client_connect(mqtt_client, &revmac_addr, 1883, NULL, NULL, &mqtt_client_info);

  for (;;) {
    uint32_t ticks = osKernelGetTickCount();
    if (ticks > 8000) {

      if (mqtt_client_is_connected(mqtt_client)) {
        mqtt_publish(mqtt_client, "test/1", "Olafo", 5, 1, 0, NULL, NULL);
      }

      //char ntp[32];

      //ip4addr_ntoa_r(&addr1, ntp, 32);
      //ip4addr_ntoa_r(&addr2, &google, 32);

      //debug("Heartbeat kernelTickCount=%d\r\n", ticks);
      //debug("pool.ntp.org: %s\r\n", ntp);
      //debug("google.com: %s\r\n", google);

      osDelay(1000);
    }


  }
//  osThreadExit();
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_EchoServerTask */
/**
* @brief Function implementing the udpEchoServer thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_EchoServerTask */
void EchoServerTask(void *argument)
{
  /* USER CODE BEGIN EchoServerTask */
  struct netconn *conn;
  struct netbuf *buf;
  ip_addr_t *addr;
  unsigned short port;

  err_t err, recv_err;

  LWIP_UNUSED_ARG(argument);

  conn = netconn_new(NETCONN_UDP);
  if (conn!= NULL)
  {
    err = netconn_bind(conn, IP_ADDR_ANY, 7777);
    if (err == ERR_OK)
    {
      while (1)
      {
        recv_err = netconn_recv(conn, &buf);

        if (recv_err == ERR_OK)
        {
          addr = netbuf_fromaddr(buf);
          port = netbuf_fromport(buf);
          netconn_sendto(conn, buf, addr, port);
          netbuf_delete(buf);
        }
        else
        {
          debug("UDP echo server netconn_recv error: %d", recv_err);
        }
      }
    }
    else
    {
      debug("UDP echo server netconn_bind error: %d", err);
      netconn_delete(conn);
    }
  }
  /* USER CODE END EchoServerTask */
}

/* DebugHeartbeatCallback function */
void DebugHeartbeatCallback(void *argument)
{
  /* USER CODE BEGIN DebugHeartbeatCallback */
  //debug("Heartbeat kernelTickCount=%d\r\n", osKernelGetTickCount());
  /* USER CODE END DebugHeartbeatCallback */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
