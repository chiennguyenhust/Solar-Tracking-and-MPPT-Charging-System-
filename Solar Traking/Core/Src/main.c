/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "ADS1015_ADS1115.h"
#include "liquidcrystal_i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


typedef enum {
    STATE_IDLE,
    STATE_SUNNY,
    STATE_TRACKING,
    STATE_WAITING,
    STATE_RETURNING,
	STATE_MANUAL
} SystemState;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define S1 0
#define S2 1

#define MIN_SUN_THRESHOLD 20		// Luong anh sang toi thieu
#define RecoverTime 5   			// Thoi gian cho nang on dinh
#define SunLowDelay 5   			// Nang yeu, dang cho nang len tro lai
#define K           10  		    // Do nhay
#define maxTrackingTime 15 		    // Thoi gian tracking toi da
#define maxReturnTime   20          // Thoi gian tro ve vi tri ban dau toi da
#define maxWaitingTrackingTime 5   // Thoi gian cho cho lan tracking tiep theo
#define OffSetValue 2580
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
ADS1xx5_I2C i2c;

uint16_t Sensor1 = 0;
uint16_t Sensor2 = 0;

float T = 0;
float Sun;

uint32_t sunny_start_time = 0;
uint32_t waiting_start_time = 0;
uint32_t tracking_start_time = 0;
uint32_t  return_start_time   = 0;

volatile SystemState currentState = STATE_IDLE;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_0)
	{
		HD44780_Clear();
		if(currentState == STATE_MANUAL)
		{
			currentState = STATE_IDLE;
		}
		else
		{
			currentState = STATE_MANUAL;
		}

		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
	}
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
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  ADS1115(&i2c, &hi2c2, ADS_ADDR_GND);
  ADSsetGain(&i2c, GAIN_ONE);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
  HD44780_Init(2);





  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)

  {
	 state_machine();

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Beep(int times, int time) {
  for (int i = 0; i < times; i++) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // "ON" state to beep
    HAL_Delay(time);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // "OFF" state to stop beep
    HAL_Delay(time);
  }
}


void state_machine(void) {
    switch (currentState) {

        case STATE_MANUAL:
            HD44780_Clear();
            HD44780_SetCursor(2, 0);
            HD44780_PrintStr("STATE_MANUAL");
            HAL_Delay(100);

            int button1State = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
            int button2State = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);

            if (button1State == 1) {

                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 7000);
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
            } else if (button2State == 1) {

                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 7000);
            } else {

                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
            }

            break;

        case STATE_IDLE:
            Sun = OffSetValue - ADSreadADC_SingleEnded(&i2c, S1) / 10;
            if (Sun > MIN_SUN_THRESHOLD) {
                currentState = STATE_SUNNY;
                sunny_start_time = HAL_GetTick();
            } else {
                currentState = STATE_WAITING;
                waiting_start_time = HAL_GetTick();
            }
            break;

        case STATE_SUNNY:
            HD44780_Clear();
            HD44780_SetCursor(2, 0);
            HD44780_PrintStr("STATE_SUNNY");
            HAL_Delay(100);

            while (1) {

                if (currentState == STATE_MANUAL) {
                    break;
                }

                uint32_t elapsed_seconds = (HAL_GetTick() - sunny_start_time) / 1000;
                // Cập nhật và hiển thị th�?i gian trôi qua
         	   display_time_on_lcd(elapsed_seconds, 2, 1);



                // Kiểm tra đi�?u kiện thay đổi trạng thái
                if ((HAL_GetTick() - sunny_start_time) >= (RecoverTime * 1000)) {
                    Sun = OffSetValue - ADSreadADC_SingleEnded(&i2c, S1) / 10;
                    if (Sun > MIN_SUN_THRESHOLD) {
                        currentState = STATE_TRACKING;
                    } else {
                        currentState = STATE_IDLE;
                    }
                    break;
                }

            }
            break;


        case STATE_TRACKING:
            HD44780_Clear();
            HD44780_SetCursor(1, 0);
            HD44780_PrintStr("STATE_TRACKING");
            enter_tracking_mode();

            if (currentState == STATE_MANUAL) {
                break;
            }
            HD44780_Clear();
            HD44780_SetCursor(0, 0);
            HD44780_PrintStr("WAITING_Tracking");
            for (int countdown = maxWaitingTrackingTime; countdown > 0; countdown--) {

                if (currentState == STATE_MANUAL) {
                    break;
                }

          	   display_time_on_lcd(countdown, 2, 1);


                HAL_Delay(1000);
            }
            currentState = STATE_IDLE;
            break;

        case STATE_WAITING:
            HD44780_Clear();
            HD44780_SetCursor(1, 0);
            HD44780_PrintStr("STATE_WAITING");

            while (1) {

                if (currentState == STATE_MANUAL) {
                    break;
                }
                uint32_t elapsed_seconds = (HAL_GetTick() - waiting_start_time) / 1000;

         	   display_time_on_lcd(elapsed_seconds, 2, 1);

                Sun = OffSetValue - ADSreadADC_SingleEnded(&i2c, S1) / 10;
                if (Sun > MIN_SUN_THRESHOLD) {
                    currentState = STATE_IDLE;
                    break;
                } else if ((HAL_GetTick() - waiting_start_time) >= (SunLowDelay * 1000)) {
                    currentState = STATE_RETURNING;
                    break;
                }
            }
            break;


        case STATE_RETURNING:
        	HD44780_Clear();
        	HD44780_SetCursor(1, 0);
        	HD44780_PrintStr("STATE_RETURNING");
            return_to_initial_position();
            currentState = STATE_IDLE;
            break;

        default:
            break;
    }
}

void enter_tracking_mode(void) {
    tracking_start_time = HAL_GetTick();
    uint32_t elapsed_seconds = 0;

    while ((HAL_GetTick() - tracking_start_time) < (maxTrackingTime * 1000)) {


        if (currentState == STATE_MANUAL) {
            break;
        }
	   elapsed_seconds = (HAL_GetTick() - tracking_start_time) / 1000;
	   display_time_on_lcd(elapsed_seconds, 2, 1);

	   Sensor1 = ADSreadADC_SingleEnded(&i2c, S1) / 10;
	   Sensor2 = ADSreadADC_SingleEnded(&i2c, S2) / 10;
	       T = Sensor1 - Sensor2;
	   if (T < -K || T > K)
	   {
		   if(T>=0)
		   {
			   __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 5000);
			   __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
		   }
		   else
		   {
			   __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
			   __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 5000);
		   }
	   }
	   else
	   {
		   __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
		   __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);

	   }

//        if (T > 0) {
//            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 5000);
//            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
//        } else {
//            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
//            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 5000);
//        }
//        Sensor1 = ADSreadADC_SingleEnded(&i2c, S1) / 10;
//        Sensor2 = ADSreadADC_SingleEnded(&i2c, S2) / 10;
//        T = Sensor1 - Sensor2;
//        if (T > -K && T < K) {
//            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
//            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
//       }
    }
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
}

void return_to_initial_position(void) {
    // Thực hiện hoạt động trở v�? vị trí ban đầu
	return_start_time = HAL_GetTick();
	uint32_t elapsed_seconds = 0;

	while ((HAL_GetTick() - return_start_time) < (maxReturnTime * 1000)) {
        if (currentState == STATE_MANUAL) {
            break;
        }

	// Tính th�?i gian đã trôi qua tính bằng giây
   elapsed_seconds = (HAL_GetTick() - return_start_time) / 1000;
   display_time_on_lcd(elapsed_seconds, 2, 1);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 5000);
}

}

void display_number(int num) {
    char buffer[10];


    if (num >= 100) {
        snprintf(buffer, sizeof(buffer), "%d", num);
    } else if (num >= 10) {
        snprintf(buffer, sizeof(buffer), " %d", num);
    } else {
        snprintf(buffer, sizeof(buffer), "  %d", num);
    }


    HD44780_PrintStr(buffer);
}

void display_time_on_lcd(uint32_t elapsed_seconds, uint8_t row, uint8_t col) {
    HD44780_SetCursor(col, row);
    HD44780_PrintStr("Time = ");
    HD44780_SetCursor(col + 7, row);
    display_number(elapsed_seconds);
    HD44780_SetCursor(col + 10, row);
    HD44780_PrintStr("s");
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
