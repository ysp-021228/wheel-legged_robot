/* USER CODE BEGIN Header */

//%%88888888888888888888888888888888888888888888888888888888888888888888888888888%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%8%&%BB%B%8%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%i. .  `iJ8B8B%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%B8%%Bw''.  '.''`q%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%BB].. .......'.w$%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%BBm^'.........`+B8B%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%88' .........."B%BB%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%8Bf .........'.0&88%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%B&@8B%8%%%%%%%%%%%%BB8%%w  ........`]CB%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%BW@%B%8%%%%%%%%%%%%%%%@~  . .......^88%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%BB%8%%8#>d8%%$%8%%%%%%%B8%%b. ........  #8%%%%%8%%}}C,UWW%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%W&%pl` ''''`[LB%%BZq-x{Z@BJ   ..... ..lJ%%B}C}W%W*`...... ]8$%%%%B%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%O`'`'.    . '  '.'^ ?%%B%b:. ...... ..&@%%B~.... .. .....^` `w#&%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%W`.'..'    ... .'.X{BB%L..   ...  ..O%8BBz..            ..:C%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%BB_'......... ' `w@%B%B`  `......`Ib@%%&%JI .           ..QBB%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%J' ..... '. Ih@#BB%%{a'  .......'"&}BBBBBBB%B%%hu '.    . ''Q$&%B%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%B%%%%%%%8%8%b^''..' . "+{WW8%%%%&%J       .....0@BBBBBBB%%%%BB%B$w .'.     'h8%%%%B%%%B%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%@^ .'''"..''.''.'.''M%#8B%%%%%%8M`'..     ...h%BBBBB%%%%%%%BBB%BB%C[`   .. '','''.'` B%%%%%%%%%%%%%%%%%%%
//%%%%%%%%Bj`    .'. '......^B%B%%%%%%%%%%k"'.'       ,M@BBB&%%BBBB8%&ZUYhaw#%&W'`  ...........' 8%%%%%%%%%%%%%%%%%%
//%%%%%%%8L``'.   '    . ^`?@%%8%%%%%%B%8L` '        ''.'...'...........'..z@BB@BL.'...        .`_Z%%%%%%%%%%%%%%%%%
//%%%%%%%m;`''''''......."&8%%8%%%%%%%%B%` .       . . .'.  .......    '.'C%%BB%B&J'         ..'':_%@%%%%%%%%%%%%%%%
//%%%%%%%%B%%%%%o ..... .v%B%%%%%%%%%BB%'^..                          .''J&88%%%B%8t.....   0&@88%8%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%f      .'$B%%%%%%%%%%8Q+'. . ... '''..    '`^.`.        a8&8%8%%%%B$`.......kB%%888%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%+'      ^B%%%%%%%%8%%&.'  . '';Y%%%8%BB%BBBBb  . . .   08BB88%%%%%%}'.     .n%88888%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%B8h^      'MB8%%%%%%%8%:.   ......^u&%%B%%BB&h       '.'q%8%%%%%%%%%BZ'.   . .bB88888%%%%%%%%%%%%%%%%%
//%%%%%%%W%$MQh:..     .._B%%%%%%%B#j       .  .U%%88888BB!..    ... v%8%%%%%%%%%%Bl     .'''nw&W@$%%%%%%%%%%%%%%%%%
//%%%%%%%#+''.''`'''... . *$%%%%%%W''        . zB%%%8888B<'`    ....J&8%%%%%%%%%BBw.      ...'.',.b%%%%%%%%%%%%%%%%%
//%%%%%%%B8l.............'."O@%%8W^.  ..     :u&%&%B%%%p' '''..   .bBW%%%%%%%%%%C+.'     ....... %W%%%%%%%%%%%%%%%%%
//%%%%%%%%%&..'.''. `  ..''' n&%QnkhJM&8B@BBB%%88%8888q...        o%88%%%%%%%&%]`.. ...  .  .  'M&%%%%%%%%%%%%%%%%%%
//%%%%%%%B%@ZadJLO888h`'''..   ;%@8%BB%%8&8%%888888%%Q'..       '+&8%%%%%%%W@l. .. .'''C%BW0Xkkq%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%B%%B%8%BWQ '.''''' `>{BW%%B8%%%%%8888BZ`'.     . 'q88%BB%B8B!^. ..'... [&%%BBB%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%B%8%%8%%Bk .'''''`.^`'xY8%%%%%%%%%8f ^       . r$8%%&Bt`^   .. ... Y@&88%%%%8%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%B%%%%B%8B%8%Wl^'.. ......  ''',[J&%8Bv` .       .,%8%_.`' .....   . 'Z&88%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%B%%%%%%%%%%%%%%%%%%@k'`''........ .   . +B%%X!.        ..b}%al    '... .    '.`tBB8%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%8B{~''`'........:j,'.']BBBJ. .     ..  v%@q ^  '``... .     '.'L8%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%@%&BQ` ''  ,#%B%8BB&8B&0' .      ..'<&8&888%88&%%Bu     'w#$%8%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%B%%%%%%%%%%%%%%%%%%%88WkMB8%%%%%%%%%%f'          .b%8%%%%%%%%%%%%8%@mW@B%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%8B%%%B%%8%%%%%B%x`.        . -B8%%%%%%%%%%%B&8%%%%%B%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%B%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#z'`.      .' .B&B8%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//B%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%8B@M` ..      . ^nOB%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%C'.. .     ..']%B%%%8%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%BB#....        ' BB%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Bq .   .    ' Q%BB%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%88888] .^^.   "p@B%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%B%%B%$d'"  ^.&BB8%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%BB%8M '^hB%8%88%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%B88%%%%W&%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "rng.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "bsp_delay.h"
#include "bsp_usart.h"
#include "remote.h"
#include "Referee.h"
#include "Auto.h"
//#include "remote_control.h"
//
#include "Calibrate.h"
//#include "chassis_task.h"
//#include "detect_task.h"
//#include "gimbal_task.h"
//#include "INS_task.h"
//#include "led_flow_task.h"
//#include "oled_task.h"
//#include "referee_usart_task.h"
//#include "usb_task.h"
//#include "voltage_task.h"
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

/* USER CODE BEGIN PV */





/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The App entry point.
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART3_UART_Init();
  MX_TIM8_Init();
  MX_CRC_Init();
  MX_RNG_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_RTC_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM10_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
    can_filter_init();
    delay_init();
    usart6_init(&usart6_buf[0], REFEREE_BUFFER_SIZE);
    usart1_init(&usart1_receive_buf[0], VISION_BUFFER_SIZE);
    cali_param_init();
    remote_control_init();

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_HSE_DIV30;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
