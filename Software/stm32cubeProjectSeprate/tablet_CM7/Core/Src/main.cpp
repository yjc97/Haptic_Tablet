/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "ik.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
char rece_uart1_[50];
char rece_uart2_[50];

/* USER CODE BEGIN PV */
constexpr int tablet_size_x = 1280;
constexpr int tablet_size_y = 800;
constexpr double ratio = 0.16875;

float left_big_cal, left_fore_cal, right_big_cal, right_fore_cal;
//float rsa, tsax, tsay;

uint8_t ready = 0;
uint8_t ready_k = 0;

//uint8_t ready_c = 0;
//uint8_t calibrated = 0;

int point_1_x;
int point_1_y;
int point_2_x;
int point_2_y;
char host_read[17];
char host_cal[50];
uint8_t new_data = 0;
uint8_t new_data_k = 0;
double x_1_screen_mm, y_1_screen_mm, x_2_screen_mm, y_2_screen_mm;;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
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
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
  int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  // start get screen data
  HAL_UART_Receive_IT(&huart3, (uint8_t *)host_read, 1);
  // calibrate motor
  {
//    char cal_cmd_0_[] = "w axis0.requested_state 3\n";
//    char cal_cmd_1_[] = "w axis1.requested_state 3\n";
    char set_vel_max_0[] = "w axis0.controller.config.vel_limit 8 \n";
    char set_vel_max_1[] = "w axis1.controller.config.vel_limit 8 \n";
	char calsw_cmd_0_[] = "w axis0.requested_state 8\n";
	char calsw_cmd_1_[] = "w axis1.requested_state 8\n";
	char idel_cmd_0_[] = "w axis0.requested_state 1\n";
	char idel_cmd_1_[] = "w axis1.requested_state 1\n";

	HAL_UART_Transmit(&huart1, (uint8_t *)idel_cmd_0_, strlen(calsw_cmd_0_), 0xffff); HAL_Delay(200);
	HAL_UART_Transmit(&huart1, (uint8_t *)idel_cmd_1_, strlen(calsw_cmd_1_), 0xffff); HAL_Delay(200);
	HAL_UART_Transmit(&huart2, (uint8_t *)idel_cmd_0_, strlen(calsw_cmd_0_), 0xffff); HAL_Delay(200);
	HAL_UART_Transmit(&huart2, (uint8_t *)idel_cmd_1_, strlen(calsw_cmd_1_), 0xffff); HAL_Delay(200);
	 HAL_Delay(15000);

    HAL_UART_Transmit(&huart1, (uint8_t *)set_vel_max_0, strlen(set_vel_max_0), 0xffff); HAL_Delay(500);
    HAL_UART_Transmit(&huart1, (uint8_t *)set_vel_max_1, strlen(set_vel_max_1), 0xffff); HAL_Delay(500);
	HAL_UART_Transmit(&huart1, (uint8_t *)calsw_cmd_0_, strlen(calsw_cmd_0_), 0xffff); HAL_Delay(500);
	HAL_UART_Transmit(&huart1, (uint8_t *)calsw_cmd_1_, strlen(calsw_cmd_1_), 0xffff); HAL_Delay(500);
    HAL_UART_Transmit(&huart2, (uint8_t *)set_vel_max_0, strlen(set_vel_max_0), 0xffff); HAL_Delay(500);
    HAL_UART_Transmit(&huart2, (uint8_t *)set_vel_max_1, strlen(set_vel_max_1), 0xffff); HAL_Delay(500);
	HAL_UART_Transmit(&huart2, (uint8_t *)calsw_cmd_0_, strlen(calsw_cmd_0_), 0xffff); HAL_Delay(500);
	HAL_UART_Transmit(&huart2, (uint8_t *)calsw_cmd_1_, strlen(calsw_cmd_1_), 0xffff); HAL_Delay(500);

//    HAL_UART_Transmit(&huart1, (uint8_t *)cal_cmd_0_, strlen(cal_cmd_0_), 0xffff);
//    HAL_Delay(12000);
//    HAL_UART_Transmit(&huart1, (uint8_t *)cal_cmd_1_, strlen(cal_cmd_1_), 0xffff);
//    HAL_Delay(16000);
  }
//  HAL_UART_Transmit(&huart2, (uint8_t *)cal_cmd_0_, strlen(cal_cmd_0_), 0xffff);
//  HAL_UART_Transmit(&huart2, (uint8_t *)cal_cmd_1_, strlen(cal_cmd_0_), 0xffff);


  // calibrate zero point with switch
  {
//  v1
//	char out_temp[100] = "";
//	char calsw_cmd_0_[] = "w axis0.requested_state 8\n";
//	char calsw_cmd_1_[] = "w axis1.requested_state 8\n";
//	char calsw_t_cmd_0_[] = "w axis0.controller.config.control_mode 1\n";
//	char calsw_t_cmd_1_[] = "w axis1.controller.config.control_mode 1\n";
//    char calsw_cmd_left_0_[] = "c 0 -0.04\n";
//    char calsw_cmd_left_1_[] = "c 1 -0.075\n";
//    char calsw_cmd_idel_0_[] = "c 0 0\n";
//    char calsw_cmd_idel_1_[] = "c 1 0\n";
//    char calsw_cmd_right_0_[] = "c 0 0.035\n";
//    char calsw_cmd_right_1_[] = "c 1 0.045\n";
//
//    char req_0_pos_[] = "f 0\n";
//    char req_1_pos_[] = "f 1\n";
//	char clc_cmd_0_[] = "w axis0.controller.config.control_mode 3\n";
//	char clc_cmd_1_[] = "w axis1.controller.config.control_mode 3\n";
//	char temp_char;
//
//    HAL_UART_Transmit(&huart1, (uint8_t *)calsw_cmd_0_, strlen(calsw_cmd_0_), 0xffff); HAL_Delay(500);
//    HAL_UART_Transmit(&huart1, (uint8_t *)calsw_t_cmd_0_, strlen(calsw_t_cmd_0_), 0xffff); HAL_Delay(500);
//    HAL_UART_Transmit(&huart1, (uint8_t *)calsw_cmd_left_0_, strlen(calsw_cmd_left_0_), 0xffff); HAL_Delay(500);
//
//    while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_RESET) HAL_Delay(100);
//    HAL_UART_Transmit(&huart1, (uint8_t *)calsw_cmd_idel_0_, strlen(calsw_cmd_idel_0_), 0xffff); HAL_Delay(500);
//    while (HAL_UART_Receive(&huart1, (uint8_t *)&temp_char, 1, 1) != HAL_TIMEOUT);
//    HAL_UART_Transmit(&huart1, (uint8_t *)req_0_pos_, strlen(req_0_pos_), 0xffff);
//    HAL_UART_Receive(&huart1, (uint8_t *)rece_uart1_, 18, 0xffff);
//    while (HAL_UART_Receive(&huart1, (uint8_t *)&temp_char, 1, 1) != HAL_TIMEOUT);
//    double temp;
//    sscanf(rece_uart1_,"%f %f", &left_big_cal, &temp);
//    sprintf(out_temp, "w axis0.controller.input_pos %f\n", left_big_cal);
//    HAL_UART_Transmit(&huart1, (uint8_t *)out_temp, strlen(out_temp), 0xffff); HAL_Delay(500);
//    HAL_UART_Transmit(&huart1, (uint8_t *)clc_cmd_0_, strlen(clc_cmd_0_), 0xffff); HAL_Delay(500);
//    left_big_cal /= REDUCTION_RATIO;
//    left_big_cal += left_big_offset;
//
//    HAL_UART_Transmit(&huart1, (uint8_t *)calsw_cmd_1_, strlen(calsw_cmd_1_), 0xffff); HAL_Delay(500);
//    HAL_UART_Transmit(&huart1, (uint8_t *)calsw_t_cmd_1_, strlen(calsw_t_cmd_1_), 0xffff); HAL_Delay(500);
//    HAL_UART_Transmit(&huart1, (uint8_t *)calsw_cmd_left_1_, strlen(calsw_cmd_left_1_), 0xffff); HAL_Delay(500);
//    HAL_UART_Transmit(&huart1, (uint8_t *)calsw_cmd_1_, strlen(calsw_cmd_1_), 0xffff); HAL_Delay(1000);
//
//
//    while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_RESET) HAL_Delay(100);
//    HAL_UART_Transmit(&huart1, (uint8_t *)calsw_cmd_idel_1_, strlen(calsw_cmd_idel_1_), 0xffff); HAL_Delay(500);
//    while (HAL_UART_Receive(&huart1, (uint8_t *)&temp_char, 1, 1) != HAL_TIMEOUT);
//    HAL_UART_Transmit(&huart1, (uint8_t *)req_1_pos_, strlen(req_1_pos_), 0xffff);
//    HAL_UART_Receive(&huart1, (uint8_t *)rece_uart1_, 18, 0xffff);
//    while (HAL_UART_Receive(&huart1, (uint8_t *)&temp_char, 1, 1) != HAL_TIMEOUT);
//    sscanf(rece_uart1_,"%f %f", &left_fore_cal, &temp);
//    sprintf(out_temp, "w axis1.controller.input_pos %f\n", left_fore_cal);
//    HAL_UART_Transmit(&huart1, (uint8_t *)out_temp, strlen(out_temp), 0xffff); HAL_Delay(500);
//    HAL_UART_Transmit(&huart1, (uint8_t *)clc_cmd_1_, strlen(clc_cmd_1_), 0xffff); HAL_Delay(500);
//    left_fore_cal /= REDUCTION_RATIO;
//    left_fore_cal += left_fore_offset;
//
//    sprintf(out_temp, "w axis1.controller.input_pos %f\n", (left_fore_cal + 0.35) * REDUCTION_RATIO);
//    HAL_UART_Transmit(&huart1, (uint8_t *)out_temp, strlen(out_temp), 0xffff); HAL_Delay(3000);
//    sprintf(out_temp, "w axis0.controller.input_pos %f\n", (left_big_cal + left_center_big) * REDUCTION_RATIO);
//    HAL_UART_Transmit(&huart1, (uint8_t *)out_temp, strlen(out_temp), 0xffff); HAL_Delay(500);
//    sprintf(out_temp, "w axis1.controller.input_pos %f\n", (left_fore_cal + left_center_fore) * REDUCTION_RATIO);
//    HAL_UART_Transmit(&huart1, (uint8_t *)out_temp, strlen(out_temp), 0xffff); HAL_Delay(3000);

//    sprintf(test,"q 1 %f 1 0.1\n", (left_fore_cal + left_center_fore) * REDUCTION_RATIO);
//    HAL_UART_Transmit(&huart1, (uint8_t *)test, strlen(test), 0xffff); HAL_Delay(2000);
//
//    sprintf(test,"q 0 %f 1 0.1\n", (left_big_cal + left_center_big) * REDUCTION_RATIO);
//    HAL_UART_Transmit(&huart1, (uint8_t *)test, strlen(test), 0xffff);


//
//	char req_0_pos_[] = "f 0\n";
//	char req_1_pos_[] = "f 1\n";
//    double temp;
//    HAL_UART_Transmit(&huart1, (uint8_t *)req_0_pos_, strlen(req_0_pos_), 0xffff);
//    HAL_UART_Receive(&huart1, (uint8_t *)rece_uart1_, 18, 0xffff);
//    sscanf(rece_uart1_,"%f %f", &left_big_cal, &temp);
//    left_big_cal /= REDUCTION_RATIO;
//    left_big_cal -= left_center_big;
//
//
//    HAL_UART_Transmit(&huart1, (uint8_t *)req_1_pos_, strlen(req_1_pos_), 0xffff);
//    HAL_UART_Receive(&huart1, (uint8_t *)rece_uart1_, 18, 0xffff);
//    sscanf(rece_uart1_,"%f %f", &left_fore_cal, &temp);
//    left_fore_cal /= REDUCTION_RATIO;
//    left_fore_cal -= left_center_fore;

//    while (!calibrated) {
//      HAL_Delay(2000);
//    }
  }

// zero reference getting
  {
	    char req_0_pos_[] = "f 0\n";
	    char req_1_pos_[] = "f 1\n";
	    float temp;
	    char temp_char;
	    HAL_UART_Transmit(&huart1, (uint8_t *)req_0_pos_, strlen(req_0_pos_), 0xffff);
	    HAL_UART_Receive(&huart1, (uint8_t *)rece_uart1_, 18, 0xffff);
	    while (HAL_UART_Receive(&huart1, (uint8_t *)&temp_char, 1, 1) != HAL_TIMEOUT);
	    sscanf(rece_uart1_,"%f %f", &left_big_cal, &temp);
	    HAL_UART_Transmit(&huart1, (uint8_t *)req_1_pos_, strlen(req_1_pos_), 0xffff);
	    HAL_UART_Receive(&huart1, (uint8_t *)rece_uart1_, 18, 0xffff);
	    while (HAL_UART_Receive(&huart1, (uint8_t *)&temp_char, 1, 1) != HAL_TIMEOUT);
	    sscanf(rece_uart1_,"%f %f", &left_fore_cal, &temp);

	    HAL_UART_Transmit(&huart2, (uint8_t *)req_0_pos_, strlen(req_0_pos_), 0xffff);
	    HAL_UART_Receive(&huart2, (uint8_t *)rece_uart1_, 18, 0xffff);
	    while (HAL_UART_Receive(&huart2, (uint8_t *)&temp_char, 1, 1) != HAL_TIMEOUT);
	    sscanf(rece_uart1_,"%f %f", &right_big_cal, &temp);
	    HAL_UART_Transmit(&huart2, (uint8_t *)req_1_pos_, strlen(req_1_pos_), 0xffff);
	    HAL_UART_Receive(&huart2, (uint8_t *)rece_uart1_, 18, 0xffff);
	    while (HAL_UART_Receive(&huart2, (uint8_t *)&temp_char, 1, 1) != HAL_TIMEOUT);
	    sscanf(rece_uart1_,"%f %f", &right_fore_cal, &temp);
  }


  Arm left_arm(Arm::Left, 27.5774374, 173.34744929, 0);
  Arm right_arm(Arm::Right, -52.0225626, 173.34744929, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  {
	char out[30];
	sprintf(out,"q 1 %f 12 0.2\n", 0.4 + left_fore_cal);
	HAL_UART_Transmit(&huart1, (uint8_t *)out, strlen(out), 0xffff);
	sprintf(out,"q 1 %f 12 0.2\n", -1 + right_fore_cal);
	HAL_UART_Transmit(&huart2, (uint8_t *)out, strlen(out), 0xffff);
	HAL_Delay(10000);
  }

  while (1)
  {
	double big_angle_l, fore_angle_l, pos_x_l, pos_y_l, big_angle_r, fore_angle_r, pos_x_r, pos_y_r;
    /* USER CODE END WHILE */
// 	v1
//	if (new_data) {
//		dis2 = x_1_screen_mm * x_1_screen_mm + y_1_screen_mm * y_1_screen_mm;
//		if (dis2 < 900) {
//			pos_x = x_1_screen_mm;
//			pos_y = y_1_screen_mm;
//		} else {
//			pos_x = x_1_screen_mm - 8 * (x_1_screen_mm /sqrt(dis2));
//			pos_y = y_1_screen_mm - 8 * (y_1_screen_mm /sqrt(dis2));
//		}
//		left_arm.GetArmAngle(big_angle, fore_angle, pos_x, pos_y);
//		char out[30];
//		sprintf(out,"q 1 %f 12 0.2\n", (left_fore_cal + fore_angle) * REDUCTION_RATIO);
//		HAL_UART_Transmit(&huart1, (uint8_t *)out, strlen(out), 0xffff);
//		sprintf(out,"q 0 %f 12 0.2\n", (left_big_cal + big_angle) * REDUCTION_RATIO);
//		HAL_UART_Transmit(&huart1, (uint8_t *)out, strlen(out), 0xffff);
//		new_data = 0;
//	}
	// circle (left arm)
//	{
//	static float t = 0;
//	char out[30];
//	left_arm.GetArmAngle(big_angle, fore_angle, 30*cosf(t), 30*sinf(t));
//	sprintf(out,"q 1 %f 10 1\n", (left_fore_cal + fore_angle) * REDUCTION_RATIO);
//	HAL_UART_Transmit(&huart1, (uint8_t *)out, strlen(out), 0xffff);
//	sprintf(out,"q 0 %f 10 1\n", (left_big_cal + big_angle) * REDUCTION_RATIO);
//	HAL_UART_Transmit(&huart1, (uint8_t *)out, strlen(out), 0xffff);
//	t += 0.01;
//	HAL_Delay(10);
//	}

	{
		static int t = 0 ;
		if (new_data) {
			if (point_1_x < 300) {
				pos_x_l = x_1_screen_mm + 3;
				pos_x_r = (300 - 640) * ratio;
				pos_y_r = (400 - 600) * ratio;
			} else if (point_1_x > 980) {
				pos_x_l = x_1_screen_mm - 3;
				pos_x_r = (980 - 640) * ratio;
				pos_y_r = (400 - 639) * ratio;
			} else {
				pos_x_l = x_1_screen_mm;
				pos_x_r = x_1_screen_mm;
				pos_y_r =  (400 - 600 - 80 * sin((-300.0 + point_1_x) / 100.0)) * ratio;
			}
			if (point_1_y < 185) {
				pos_y_l = y_1_screen_mm - 3;
			} else if (point_1_y > 300) {
				pos_y_l = (400 - 300) * ratio + 3;
			} else if (point_1_y > 215) {
				pos_y_l = y_1_screen_mm + 3;
			} else pos_y_l = y_1_screen_mm;
			left_arm.GetArmAngle(big_angle_l, fore_angle_l, pos_x_l, pos_y_l);
			right_arm.GetArmAngle(big_angle_r, fore_angle_r, pos_x_r, pos_y_r);

			char out[30];
			if (++t > 20) {
				sprintf(out,"q 1 %f 12 0.2\n", left_fore_cal + fore_angle_l);
				HAL_UART_Transmit(&huart1, (uint8_t *)out, strlen(out), 0xffff);
				sprintf(out,"q 0 %f 12 0.2\n", left_big_cal + big_angle_l);
				HAL_UART_Transmit(&huart1, (uint8_t *)out, strlen(out), 0xffff);
			}
			sprintf(out,"q 1 %f 12 0.2\n", right_fore_cal + fore_angle_r);
			HAL_UART_Transmit(&huart2, (uint8_t *)out, strlen(out), 0xffff);
			sprintf(out,"q 0 %f 12 0.2\n", right_big_cal + big_angle_r);
			HAL_UART_Transmit(&huart2, (uint8_t *)out, strlen(out), 0xffff);
			new_data = 0;
		} else if (new_data_k) {
			left_arm.GetArmAngle(big_angle_l, fore_angle_l, x_1_screen_mm, y_1_screen_mm);
			right_arm.GetArmAngle(big_angle_r, fore_angle_r, x_2_screen_mm, y_2_screen_mm);
			char out[30];
			sprintf(out,"q 1 %f 12 0.2\n", left_fore_cal + fore_angle_l);
			HAL_UART_Transmit(&huart1, (uint8_t *)out, strlen(out), 0xffff);
			sprintf(out,"q 0 %f 12 0.2\n", left_big_cal + big_angle_l);
			HAL_UART_Transmit(&huart1, (uint8_t *)out, strlen(out), 0xffff);
			sprintf(out,"q 1 %f 12 0.2\n", right_fore_cal + fore_angle_r);
			HAL_UART_Transmit(&huart2, (uint8_t *)out, strlen(out), 0xffff);
			sprintf(out,"q 0 %f 12 0.2\n", right_big_cal + big_angle_r);
			HAL_UART_Transmit(&huart2, (uint8_t *)out, strlen(out), 0xffff);
			new_data_k = 0;
		}

	}

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x307075B1;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 1152000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 1152000;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_Init;
  GPIO_Init.Pin = GPIO_PIN_0;
  GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_Init.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_Init.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_Init);

  GPIO_Init.Mode = GPIO_MODE_INPUT;
  GPIO_Init.Speed = GPIO_SPEED_FREQ_MEDIUM;
  GPIO_Init.Pull = GPIO_PULLDOWN;
  GPIO_Init.Pin = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOB, &GPIO_Init);
  GPIO_Init.Pin = GPIO_PIN_3;
  HAL_GPIO_Init(GPIOB, &GPIO_Init);
  GPIO_Init.Pin = GPIO_PIN_4;
  HAL_GPIO_Init(GPIOB, &GPIO_Init);
  GPIO_Init.Pin = GPIO_PIN_5;
  HAL_GPIO_Init(GPIOB, &GPIO_Init);
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

  if (huart == &huart3){
	if (ready) {
      sscanf(host_read, "%d %d", &point_1_x, &point_1_y);
      x_1_screen_mm = (point_1_x - 640) * ratio;
      y_1_screen_mm = (-point_1_y + 400) * ratio;
	  ready = 0;
	  HAL_UART_Receive_IT(&huart3, (uint8_t *)host_read, 1);
	  new_data = 1;
	} else if (ready_k){
	  sscanf(host_read, "%d %d %d %d", &point_1_x, &point_1_y, &point_2_x, &point_2_y);
      x_1_screen_mm = (point_1_x - 640) * ratio;
      y_1_screen_mm = (-point_1_y + 400) * ratio;
      x_2_screen_mm = (point_2_x - 640) * ratio;
      y_2_screen_mm = (-point_2_y + 400) * ratio;
      ready_k = 0;
	  HAL_UART_Receive_IT(&huart3, (uint8_t *)host_read, 1);
      new_data_k = 1;
//	} else if (ready_c){
//      sscanf(host_cal, "%f %f %f %f %f", &rsa, &tsax, &tsay, &left_big_cal, &left_fore_cal);
//      ready_c = 0;
//      calibrated = 1;
//      HAL_UART_Receive_IT(&huart3, (uint8_t *)host_read, 1);
	} else if (host_read[0] == 'b'){
	  ready = 1;
	  HAL_UART_Receive_IT(&huart3, (uint8_t *)host_read, 9);
//	} else if (host_read[0] == 'a'){
//	  char req_0_pos_[] = "f 0\n";
//	  char req_1_pos_[] = "f 1\n";
//	  double pos1, pos2, vel;
//	  HAL_UART_Transmit(&huart1, (uint8_t *)req_0_pos_, strlen(req_0_pos_), 0xffff);
//	  HAL_UART_Receive(&huart1, (uint8_t *)rece_uart1_, 18, 0xffff);
//	  sscanf(rece_uart1_,"%f %f", &pos1, &vel);
//	  HAL_UART_Transmit(&huart1, (uint8_t *)req_1_pos_, strlen(req_1_pos_), 0xffff);
//	  HAL_UART_Receive(&huart1, (uint8_t *)rece_uart1_, 18, 0xffff);
//	  sscanf(rece_uart1_,"%f %f", &pos2, &vel);
//	  char temp_out[20];
//	  sprintf(temp_out, "%7.4f %7.4f", pos1, pos2);
//	  HAL_UART_Transmit(&huart3, (uint8_t *)temp_out, strlen(temp_out), 0xffff);
//	} else if (host_read[0] == 'c'){
//	  ready_c = 1;
//	  HAL_UART_Receive_IT(&huart3, (uint8_t *)host_cal, 39);
	} else if (host_read[0] == 'k'){
	  ready_k = 1;
	  HAL_UART_Receive_IT(&huart3, (uint8_t *)host_read, 17);
	} else HAL_UART_Receive_IT(&huart3, (uint8_t *)host_read, 1);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
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
