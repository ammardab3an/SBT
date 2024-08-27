/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#include <main.h>
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"
#include "iwdg.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <MPU6050.hpp>
#include <MEKFcRP.hpp>
#include <PID.hpp>
#include "string.h"
#include "stdio.h"
#include "math.h"

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

/* Sensors */
MPU6050_t MPU6050;

/* Kalman filter */
MEKFcRP kal;

/* Definitions */
float KAL_Qw = 1;
float KAL_Qa = 0.01;
float KAL_Rw = 1e-8;
float KAL_Ra = 1e-8;
const bool CALC_EN = true;
const bool PID_EN = true;
const bool TEAPOT_EN = false;
const uint32_t CALC_TIME = 0;
const uint32_t PID_TIME = 0;
const uint32_t TEAPOT_TIME = 20;

/* Variables */
uint32_t cur_tick;
uint32_t calc_dt, calc_lst;
uint32_t pid_dt, pid_lst;
uint32_t teapot_lst;
volatile uint32_t cnt = 0;

/* Flags */
volatile uint8_t recalc_filters_flag;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Reset_Filters(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t init_tim;

float quat_f[4];
int16_t quat[4];
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

float zyx[3], zxy[3];
float measurement[2];

PIDController pid[2];
float target[2] = {0, 0};
float pid_kp[2] = {4, 3};
float pid_ki[2] = {0, 0};
float pid_kd[2] = {0, -50};
float pid_tau[2] = {0.02, 1};
float pid_lim_min[2] = {-1000, -90};
float pid_lim_max[2] = {+1000, +90};
float pid_lim_int_min[2] = {-300, -30};
float pid_lim_int_max[2] = {+300, +30};
float pid_res[2];

uint16_t servo_val[2] = {1500, 1500};

void calc_zyx();
void calc_zxy();
float mapArduino(float val, float I_Min, float I_Max, float O_Min, float O_Max);

uint16_t sim_pos = 0;
float sim_len = 12;
float sim_steps[] = {0, +45, -90, +45, -90, 0, 20, -40, 0, 10, 0, 0};
bool sim_enable = false;
uint32_t sim_lst;
uint16_t sim_out;
uint16_t sim_dt = 500;
uint16_t sim_print_lst;
uint16_t sim_print_dt = 10;

volatile bool global_reset = false;

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	MX_IWDG_Init();
	MX_TIM3_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */

	while (MPU6050_Init(&hi2c1));
//	MPU6050_Calc_Gyro_Bias(&hi2c1, &MPU6050);
	Reset_Filters();

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, servo_val[0]);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, servo_val[1]);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

	for(int i = 0; i < 2; i++){

		pid[i] = (PIDController) {
			pid_kp[i], pid_ki[i], pid_kd[i], pid_tau[i],
			pid_lim_min[i], pid_lim_max[i],
			pid_lim_int_min[i], pid_lim_int_max[i],
			15 * 0.001
		};

		PIDController_Init(&pid[i]);
	}

	HAL_IWDG_Refresh(&hiwdg);

//	while(true){
//		MPU6050_Calc_Gyro_Bias(&hi2c1, &MPU6050);
//		MPU6050_Read_All(&hi2c1, &MPU6050);
//		printf("%f %f %f %f\r\n", MPU6050.Temperature, MPU6050.Gx_bias, MPU6050.Gy_bias, MPU6050.Gz_bias);
////		HAL_IWDG_Refresh(&hiwdg);
//	}

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	init_tim = HAL_GetTick();
	calc_lst = init_tim;
	teapot_lst = init_tim;
	pid_lst = init_tim + 2000;

	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		if(global_reset){
			NVIC_SystemReset();
		}

		if (recalc_filters_flag) {
			recalc_filters_flag = 0;
			Reset_Filters();
		}

		cur_tick = HAL_GetTick();

		if(CALC_EN) if (cur_tick - calc_lst >= CALC_TIME) {

			auto status = MPU6050_Read_All(&hi2c1, &MPU6050);

			if(status == HAL_OK){

				calc_dt = cur_tick - calc_lst;
				calc_lst = cur_tick;

				float am[] = {MPU6050.Ax, MPU6050.Ay, MPU6050.Az};
				float wm[] = {MPU6050.Gx, MPU6050.Gy, MPU6050.Gz};
				kal.updateIMU(am, wm, calc_dt * 0.001f);

				if (++cnt % 50 == 0) {
					HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
				}

				HAL_IWDG_Refresh(&hiwdg);
			}
		}

		if(!sim_enable) if(PID_EN) if (cur_tick > PID_TIME + pid_lst){

			pid_dt = cur_tick - pid_lst;
			pid_lst = cur_tick;
			kal.get_q(quat_f);
			calc_zyx();
			calc_zxy();

			if(pid_dt > 20){
				pid_dt = 20;
			}

			measurement[0] = -zyx[1];
			measurement[1] = -zxy[1];

			for(int i = 0; i < 2; i++){
		        PIDController_Update(&pid[i], target[i], measurement[i], pid_dt, i==0);
			}

			pid_res[0] += pid[0].out * (pid_dt * 0.001);
			pid_res[1] = -pid[1].out;

			if(pid_res[0] > 90){
				pid_res[0] = 90;
			}
			else if(pid_res[0] < -90){
				pid_res[0] = -90;
			}

			servo_val[0] = (uint16_t) mapArduino(pid_res[0], -90, +90, 500, 2500);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, servo_val[0]);

//			servo_val[1] = (uint16_t) mapArduino(pid_res[1], -90, +90, 1000+25, 2000-25);
//			servo_val[1] += servo_val[1] >= 1500 ? +25 : -25;
//			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, servo_val[1]);

			servo_val[1] = (uint16_t) mapArduino(pid_res[1], -90, +90, 1000, 2000);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, servo_val[1]);

//			printf("%f %f %f \n", zyx[0], zyx[1], zyx[2]);
//			printf("%f %f %f \n", zxy[0], zxy[1], zxy[2]);
//			printf("%f %f %d %f %f \n", zyx[1], zxy[1], servo_val[0], error[0], pid[0]);

//			printf("%d %f %f %f\n", (int)cur_tick, target[0], pid[0].prevError, pid[0].out);
//			printf("%d %f %f %f %f %f %f\n", (int)cur_tick, target[1], pid[1].prevError, pid[1].out, pid[1].differentiator, pid[1].proportional, measurement[1]);
//			printf("%f %f %f\n", target[1], measurement[1], pid[1].out);

			printf("%f %f\n", measurement[0], measurement[1]);
		}

//		if(sim_enable){
//
//			if(cur_tick-sim_lst > sim_dt){
//
//				sim_pos++;
//				sim_lst = cur_tick;
//
//				if(sim_pos==sim_len){
//					sim_pos = 0;
//					sim_enable = false;
//				}
//				else{
//					sim_out = (uint16_t) mapArduino(sim_steps[sim_pos], -90, +90, 1000+25, 2000-25);
//					sim_out += sim_out >= 1500 ? +25 : -25;
//					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, sim_out);
//				}
//			}
//
//			if(cur_tick-sim_print_lst > sim_print_dt){
//				sim_print_lst= cur_tick;
//				kal.get_q(quat_f);
//				calc_zxy();
//				printf("%f %f\n", sim_steps[sim_pos], -zxy[1]);
//			}
//		}

//		if(TEAPOT_EN) if (cur_tick - teapot_lst > TEAPOT_TIME) {
//
//			teapot_lst = cur_tick;
//
//			kal.get_q(quat_f);
//
//			quat[0] = ((int16_t)(quat_f[0] * (1<<14)));
//			quat[1] = ((int16_t)(quat_f[1] * (1<<14)));
//			quat[2] = ((int16_t)(quat_f[2] * (1<<14)));
//			quat[3] = ((int16_t)(quat_f[3] * (1<<14)));
//
//			teapotPacket[2] = (quat[0] >> 8) & 0xffff;
//			teapotPacket[3] = (quat[0] >> 0) & 0xffff;
//			teapotPacket[4] = (quat[1] >> 8) & 0xffff;
//			teapotPacket[5] = (quat[1] >> 0) & 0xffff;
//			teapotPacket[6] = (quat[2] >> 8) & 0xffff;
//			teapotPacket[7] = (quat[2] >> 0) & 0xffff;
//			teapotPacket[8] = (quat[3] >> 8) & 0xffff;
//			teapotPacket[9] = (quat[3] >> 0) & 0xffff;
//
//			teapotPacket[10] = calc_dt;
//			teapotPacket[11] = 0;
//
//			HAL_UART_Transmit(&huart2, teapotPacket, sizeof(teapotPacket), 1000);
//		}
	}

	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_I2C1;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

static void Reset_Filters(void) {
	/* Initialize Kalman filter */
	kal.reset_orientation();
	kal.set_Qa(KAL_Qa);
	kal.set_Qw(KAL_Qw);
	kal.set_Ra(KAL_Ra);
	kal.set_Rw(KAL_Rw);
	kal.set_W0(0.04);
	kal.set_chartUpdate(true);
}

float atan2d_snf(float u0, float u1) {

	float y;
	int32_t u0_0;
	int32_t u1_0;

	if (isnan(u0) || isnan(u1)) {
		y = nan("");
	} else if (isinf(u0) && isinf(u1)) {
		if (u0 > 0.0) {
			u0_0 = 1;
		} else {
			u0_0 = -1;
		}
		if (u1 > 0.0) {
			u1_0 = 1;
		} else {
			u1_0 = -1;
		}
		y = atan2(u0_0, u1_0);
	} else if (u1 == 0.0) {
		if (u0 > 0.0) {
			y = PI / 2.0;
		} else if (u0 < 0.0) {
			y = -(PI / 2.0);
		} else {
			y = 0.0;
		}
	} else {
		y = atan2(u0, u1);
	}

	return y;
}

void calc_zyx(){

	float u0 = -2 * (quat_f[1]*quat_f[3]-quat_f[0]*quat_f[2]);

	if (u0 > 1.0) {
		u0 = 1.0;
	} else {
		if (u0 < -1.0) {
			u0 = -1.0;
		}
	}
//	zyx[0] = atan2d_snf(2*(quat_f[1]*quat_f[2]+quat_f[0]*quat_f[3]), quat_f[0]*quat_f[0]+quat_f[1]*quat_f[1]-quat_f[2]*quat_f[2]-quat_f[3]*quat_f[3]);
	zyx[1] = asin(u0);
//	zyx[2] = atan2d_snf(2*(quat_f[2]*quat_f[3]+quat_f[0]*quat_f[1]), quat_f[0]*quat_f[0]-quat_f[1]*quat_f[1]-quat_f[2]*quat_f[2]+quat_f[3]*quat_f[3]);

	for(uint8_t i = 0; i < 3; i++){
		zyx[i] *= RAD_TO_DEG;
	}
}

void calc_zxy(){

	float u0 = 2 * (quat_f[2]*quat_f[3] + quat_f[0]*quat_f[1]);

	if (u0 > 1.0) {
		u0 = 1.0;
	} else {
		if (u0 < -1.0) {
			u0 = -1.0;
		}
	}
//	zxy[0] = atan2d_snf(-2*(quat_f[1]*quat_f[2]-quat_f[0]*quat_f[3]), quat_f[0]*quat_f[0]-quat_f[1]*quat_f[1]+quat_f[2]*quat_f[2]-quat_f[3]*quat_f[3]);
	zxy[1] = asin(u0);
//	zxy[2] = atan2d_snf(-2*(quat_f[1]*quat_f[3]-quat_f[0]*quat_f[2]), quat_f[0]*quat_f[0]-quat_f[1]*quat_f[1]-quat_f[2]*quat_f[2]+quat_f[3]*quat_f[3]);

	for(uint8_t i = 0; i < 3; i++){
		zxy[i] *= RAD_TO_DEG;
	}
}

float mapArduino(float val, float I_Min, float I_Max, float O_Min, float O_Max) {
    return (((val - I_Min) * ((O_Max - O_Min) / (I_Max - I_Min))) + O_Min);
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
