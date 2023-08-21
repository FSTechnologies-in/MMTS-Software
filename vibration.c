/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "UartRingbuffer.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MODE 2
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char User_Name[30];
char Mode_of_Rotation[30];
char Time_Duration[30];
int Time_from_Iot;
char data;
char buff1[90];
char buff2[90];
char buf[30];
char clockwise[20];
char time_buff[20];
uint8_t count_sec = 0;
uint8_t flag=0;
uint8_t time=0;
uint8_t stop=0;
uint8_t tim_val = 0;
uint8_t time_flag=0;
uint8_t sw1_flag=0;
uint8_t sw2_flag=0;
uint8_t time_end_flag=0;
uint8_t iot_flag=0;
uint8_t event_flag=0;
uint8_t mode=0;
int Roll_calib;
int Pitch_calib;
int Height_calib;
int Roll_live;
int Pitch_live;
int Height_live;
char Data;
char R_data[22];
char P_data[22];
char H_data[22];
char actual_data[100];
int sensor_flag=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */
int data_from_iot(void);
void motor_drive_clockwise(void);
void motor_drive_anticlockwise(void);
int iot_data_validation(char data[]);
void switch1_release(void);
void laser_detect(void);
void timer_start(void);
void Buzzer(void);
void motor_drive_stop(void);
void motor_diagnostic(void);
int led_diagnostic(void);
void laser_diagnostic(void);
void Vibration_sensor_calibration(void);
void vibration_livedata(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int data_from_iot(void)
{
	/*
	 * checking for IoT data,checking for name,mode,dutation of time;
	 */

	uint8_t count=0;
	uint8_t quote_double=0;
	uint8_t name_increment=0;
	uint8_t mode_increment=0;
        uint8_t time_increment=0;

	if (IsDataAvailable())
	{
		 HAL_UART_Transmit(&huart2, "data", strlen("data"),100);
                if(Wait_for("{")==1)
                {
        	       Copy_upto("}", buff1);
        	       HAL_UART_Transmit(&huart2, buff1, strlen(buff1),100);
                }

		while (1)
		{
		      if (buff1[count] == '"')
		      {
			    ++quote_double;
		      }

		     if (quote_double == 3)
		     {
			   if (buff1[count + 1] != '"')
			   {
				User_Name[name_increment] = buff1[count + 1];
				name_increment++;
			   }
			   else
			   {
				   if (buff1[count + 2] == '}')
					  {
					   quote_double=0;
					   sprintf(buff2, "Name:%s\r\n", User_Name);
					   HAL_UART_Transmit(&huart2, buff2, strlen(buff2), 100);
						  mode=2;
						  break;
					  }
			   }
		    }

		    if (quote_double == 7)
		    {
			   if (buff1[count + 1] != '"')
			   {
				Mode_of_Rotation[mode_increment] = buff1[count + 1];
				mode_increment++;
			   }
		   }

		   if (quote_double == 11)
		   {
			  if (buff1[count + 1] != '"')
			  {
				Time_Duration[time_increment] = buff1[count + 1];
				time_increment++;
			  }

		   }
		   if (quote_double >11)
		   		   {
			   Uart_flush();
			   quote_double=0;
			   	   	   	   	   	   mode=1;
			  					  break;
		   		   }
//		  if (quote_double == 12)
//		  {
//			  mode=2;
//			break;
//		  }
		  ++count;
          }


	}
	if (mode == 1) {
		iot_flag = 1;
				Time_from_Iot = atoi(Time_Duration);
				sprintf(buff2, "Time:%d\r\n", Time_from_Iot);
				HAL_UART_Transmit(&huart2, buff2, strlen(buff2), 100);
//				if (strcmp(Mode_of_Rotation, "Slice") == 0 && Time_from_Iot) {
					memset(Mode_of_Rotation, 0, sizeof(Mode_of_Rotation));
					sprintf(buff2, "Duration:%d\r\n", Time_from_Iot);
					HAL_UART_Transmit(&huart2, buff2, strlen(buff2), 100);
					Uart_flush();
					iot_flag = 1;
					return 1;
//				} else {
//					HAL_UART_Transmit(&huart2, "exit\r\n", strlen("exit\r\n"), 100);
//					Uart_flush();
//					return 0;
//				}
			}
	if(mode==2)
	{

		if(strcmp(User_Name,"MT") ==0)
			{
				//call motor function
				motor_diagnostic();
				Uart_write(2);
				Uart_flush();
				return 0;
			}
		 if(strcmp(User_Name,"LT") ==0)
			{
				//call led function
				led_diagnostic();
				Uart_write(3);
				Uart_flush();

			}
		 if(strcmp(User_Name,"LS") ==0)
			{
				//call laser function
				laser_diagnostic();
				Uart_flush();
				return 0;
			}
			else
			{
				Uart_flush();
				return 0;
			}
		 Uart_flush();
	}
}
void Vibration_sensor_calibration(void)
{
	 int count_sensor=0;

    while(1)
    {
    	 int i=0;
	    	  do
	    	   {

	    		  HAL_UART_Receive(&huart4, &Data, 1, 100);
				actual_data[i++]=Data;
	    	   }while(Data!='\n' );

	       HAL_UART_Transmit(&huart2, actual_data, sizeof(actual_data),100);
	       HAL_UART_Transmit(&huart2, "\n\r", 2,100);
	       if(actual_data[0]=='R')
	       {
	    	   int actal_i=2;
	    	   int data_i=0;
	    	   HAL_UART_Transmit(&huart2, "R\n\r", 3,100);

	    	   do{
	    		   R_data[data_i]=actual_data[actal_i];
	    		   data_i++;
	    		   actal_i++;
	    	   }while(actual_data[actal_i]!='\t');
	    	    HAL_UART_Transmit(&huart2,R_data ,sizeof(R_data),100);
//	           HAL_UART_Transmit(&huart2, "\n\r", 2,100);

	           data_i=0;
	           actal_i +=3;

	    	   do{
	    		   P_data[data_i]=actual_data[actal_i];
	    		   data_i++;
	    		   actal_i++;

	    	   }while(actual_data[actal_i]!='\t');
	    	    HAL_UART_Transmit(&huart2,P_data ,sizeof(P_data),100);
//
//	           HAL_UART_Transmit(&huart2, "\n\r", 2,100);

	           data_i=0;
			  actal_i +=3;
			  HAL_UART_Transmit(&huart2, "H\n\r", 3,100);
		   do{
			   H_data[data_i]=actual_data[actal_i];
			   data_i++;
			   actal_i++;
		   }while(actual_data[actal_i]!='\n');
			HAL_UART_Transmit(&huart2,H_data ,sizeof(H_data),100);
//			HAL_UART_Transmit(&huart2, "\n\r", 2,100);

	       }
			memset(actual_data,0,sizeof(actual_data));
			count_sensor++;
			if(count_sensor > 15)
			{
				HAL_UART_Transmit(&huart2, "greater100\n\r", 15,100);
				 Height_calib = atof(H_data);
				 Pitch_calib = atof(P_data);
				 Roll_calib = atof(R_data);
				 if(Height_calib != 0 && Pitch_calib!=0 && Roll_calib != 0 )
				 {
					 HAL_UART_Transmit(&huart2, "break\n\r", 7,100);
					 break;
				 }
			}

    }

}

void vibration_livedata(void)
{
	int i=0;
	HAL_UART_Transmit(&huart2, "live data\n\r", 12,100);
	          do
		    	   {

		    		  HAL_UART_Receive(&huart4, &Data, 1, 100);
					actual_data[i++]=Data;
		    	   }while(Data!='\n' );

		       HAL_UART_Transmit(&huart2, actual_data, sizeof(actual_data),100);
		       HAL_UART_Transmit(&huart2, "\n\r", 2,100);
		       if(actual_data[0]=='R')
		       {
		    	   int actal_i=2;
		    	   int data_i=0;
		    	   HAL_UART_Transmit(&huart2, "R\n\r", 3,100);

		    	   do{
		    		   R_data[data_i]=actual_data[actal_i];
		    		   data_i++;
		    		   actal_i++;
		    	   }while(actual_data[actal_i]!='\t');
//		    	    HAL_UART_Transmit(&huart2,R_data ,sizeof(R_data),100);
		    	   Roll_live = atof(R_data);
//		           HAL_UART_Transmit(&huart2, "\n\r", 2,100);

		           data_i=0;
		           actal_i +=3;

		    	   do{
		    		   P_data[data_i]=actual_data[actal_i];
		    		   data_i++;
		    		   actal_i++;

		    	   }while(actual_data[actal_i]!='\t');
//		    	    HAL_UART_Transmit(&huart2,P_data ,sizeof(P_data),100);
	//
//		           HAL_UART_Transmit(&huart2, "\n\r", 2,100);
		    	   Pitch_live = atof(P_data);

		           data_i=0;
				  actal_i +=3;
				  HAL_UART_Transmit(&huart2, "H\n\r", 3,100);
			   do{
				   H_data[data_i]=actual_data[actal_i];
				   data_i++;
				   actal_i++;
			   }while(actual_data[actal_i]!='\n');
//				HAL_UART_Transmit(&huart2,H_data ,sizeof(H_data),100);

			   Height_live = atof(H_data);

		       }
				memset(actual_data,0,sizeof(actual_data));
}
void motor_drive_clockwise(void)
{
	/*
	 * start motor clockwise to reach Default position
	 * Motor Input 1 - SET
	 * Motor Input 2 - RESET
	 */

	HAL_UART_Transmit(&huart2, "clockwise enter\r\n", strlen("clockwise enter\r\n"),100);
	TIM2->CCR1 = 25;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, SET); //Motor ON
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, RESET); //Direction of Pin(CLOCK WISE)
}

void motor_drive_anticlockwise(void)
{
	/*
	 * start motor anti - clockwise to reach 90 degree position
	 * Motor Input 1- RESET
	 * Motor Input 2- SET
	 */

	HAL_UART_Transmit(&huart2, "anti clockwise motor enter\r\n", strlen("anti clockwise motor enter\r\n"),100);
	TIM2->CCR1 = 25;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, SET); //Motor ON
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, SET); //Direction of Pin(COUNTER CLOCK WISE)
}

void motor_drive_stop(void)
{
	 /*
	 * turn off the motor
	 * Motor Input 1- Reset
	 * Motor Input 2- Reset
	 */
	HAL_UART_Transmit(&huart2, "motor stop\r\n", strlen("motor stop\r\n"), 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, RESET); //Motor OFF
}

void timer_start(void)
{
	  /*
	  *  start timer for 10 seconds and buzzer on
	  */
	if(sensor_flag==0)
	{
	Vibration_sensor_calibration();
		sensor_flag=1;
	}
	HAL_UART_Transmit(&huart2, "time enter\r\n", strlen("time enter\r\n"), 100);
	Buzzer();
	
	time_flag = 1; // Set Timer flag to indicate the timer on call laser detect function in superloop
}

void Buzzer(void)
{
	 /*
	 *  buzzer to indicate timer as started
	 */
	HAL_GPIO_WritePin(GPIOB, BUZZER_Pin, 1);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOB, BUZZER_Pin, 0);

}
void motor_diagnostic(void)
{
	TIM2->CCR1 = 25;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, RESET); //Direction of Pin(CLOCK WISE)
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, SET); //Motor ON

	HAL_Delay(500);
  while(1)
  {

	if (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == 0)
	{



				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, RESET); //Motor OFF
						HAL_Delay(1000);
					TIM2->CCR1 = 25;
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, SET); //Direction of Pin(COUNTER CLOCK WISE)
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, SET); //Motor ON
					HAL_Delay(500);
	}
	if(HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == 0)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, RESET); //Motor OFF
						mode=0;
							iot_flag = 0;
							Uart_flush();
							HAL_UART_Transmit(&huart2, "IR2\r\n",strlen("LED\r\n"),1000);
						break;

	}

  }

}
int led_diagnostic(void)
{
	HAL_UART_Transmit(&huart2, "LED\r\n",strlen("LED\r\n"),1000);
	for(int led=0;led<5;led++)
	{
		HAL_GPIO_TogglePin(GPIOA, LED_Pin);
		HAL_Delay(300);
	}
	memset(User_Name,0,sizeof(User_Name));
	mode=0;
	iot_flag = 0;
	Uart_flush();
	return 0;
}
void laser_diagnostic(void)
{


	int shoot_flag=0;
	HAL_UART_Transmit(&huart2, "Shoot\r\n",strlen("Shoot\r\n"),1000);
	 Vibration_sensor_calibration();
	int count_time_flag=0;
	while(1)
	{

		  vibration_livedata();
		  if(((Pitch_live )> Pitch_calib+2) )
		  	  {
		  		  sprintf(buf,"Pitchlive:%d\r\n",Pitch_live);
		  		  HAL_UART_Transmit(&huart2, buf,sizeof(buf),1000);
		  		  sprintf(buf,"Pitchcalib:%d\r\n",Pitch_calib);
		  		  HAL_UART_Transmit(&huart2, buf,sizeof(buf),1000);
		  		Buzzer();
					HAL_UART_Transmit(&huart2, "Shoot succ\r\n",strlen("Shoot succ\r\n"),1000);
					Uart_write(4);
					Uart_flush();
					shoot_flag=1;
					break;
		  	  }
         count_time_flag++;
		if(count_time_flag>80)
		{
			break;
		}


	}
	if(shoot_flag==1)
	{
		HAL_UART_Transmit(&huart2, "Shoot Done\r\n",strlen("Shoot Done\r\n"),1000);
	}
	else
	{
		Buzzer();
		HAL_Delay(300);
		Buzzer();
		HAL_Delay(300);
		Buzzer();
		HAL_Delay(300);

	}

	mode=0;
    iot_flag = 0;
    Uart_flush();

}
void laser_detect(void)
{
	 /*
	 * as timer started meanwhile start detecting the laser gun is short r not,
	 * if laser gun is short then move motor to default position (clock-wise direction),
	 * stop the timer.
	 *
	 */

     if(sensor_flag==1)
     { 
	HAL_TIM_Base_Start_IT(&htim6);
	 vibration_livedata();
			  if(((Pitch_live )> Pitch_calib+2) )
			  	  {
			  		  sprintf(buf,"Pitchlive:%d\r\n",Pitch_live);
			  		  HAL_UART_Transmit(&huart2, buf,sizeof(buf),1000);
			  		  sprintf(buf,"Pitchcalib:%d\r\n",Pitch_calib);
			  		  HAL_UART_Transmit(&huart2, buf,sizeof(buf),1000);
			  		HAL_TIM_Base_Stop_IT(&htim6);
						time_flag = 0; // Reset timer flag for another time will execute this function
						HAL_UART_Transmit(&huart2, "laser enter\r\n", strlen("laser enter\r\n"),100);
						Buzzer();
						Uart_write(1);
						memset(Time_Duration,0,sizeof(Time_Duration));
						HAL_Delay(5000);
						motor_drive_anticlockwise();
						 sw1_flag=1;// Increment sw flag to turn on switch sw2
						count_sec=0;
						sensor_flag=0;
			  	  }
	     
     }
//	if (HAL_GPIO_ReadPin(GPIOC, LASER_Pin) == 1)
//	{
//		HAL_TIM_Base_Stop_IT(&htim6);
//		time_flag = 0; // Reset timer flag for another time will execute this function
//		HAL_UART_Transmit(&huart2, "laser enter\r\n", strlen("laser enter\r\n"),100);
//		Buzzer();
//		Uart_write(1);
//		memset(Time_Duration,0,sizeof(Time_Duration));
//		HAL_Delay(5000);
//		motor_drive_anticlockwise();
//
//         sw1_flag=1;// Increment sw flag to turn on switch sw2
//		count_sec=0;
//
//	}

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
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  Ringbuf_init ();
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_UART_Transmit(&huart2, "welcome\r\n",strlen("welcome\r\n"), 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	if (iot_flag == 0)
	{
		//  if IoT flag is clear then entering to Data_from_iot
		data_from_iot();    // Data_from_IoT function return 1 then set the I0T flag

	}
	if (iot_flag)
	{
		//if IoT flag is set enter into the  function
		 /*
		 * initially switch-1 flag is zero,
		 * then motor start running from default position to 90 degree position (anti-clock wise direction).
		 */
		if ( event_flag == 0)
		{
			event_flag++;
			HAL_UART_Transmit(&huart2, "clockwise motor start\r\n",strlen("clockwise motor start\r\n"), 100);
			motor_drive_clockwise();
		}

	         /*
		* once motor touch switch-2 then motor stop, timer starts for 10 seconds,
		* and motor also stay in 90 degree position
		*/
		if (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == 0 && event_flag == 1)
		{
           event_flag++;
			HAL_UART_Transmit(&huart2, "motor stop\r\n",strlen("motor stop\r\n"), 100);
			motor_drive_stop();
			HAL_Delay(1000);
			timer_start();
		}
	     /*if laser is not short until after 10 seconds,
	     * then motor start running from 90 degree position to default position(clockwise direction)
	     */
		 /*
		 * Once motor touch the switch-1 again(default position) stop the motor
		 */
	   if (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == 0 && event_flag == 2)
            {
		time_end_flag=0;
		HAL_UART_Transmit(&huart2, "Home position\r\n",strlen("Home position\r\n"), 100);
		motor_drive_stop();
		iot_flag = 0;
		mode=0;
		event_flag=0;
	   }
	 /*
	 *  once the timer flag is set call the laser_detect function
	 *  if timer flag is clear then stop transmitting the RF signal
	 */
	if (time_flag == 1)
	{

		laser_detect();
	}
	/*
	 * Timer flag is reset in timer interrupt to disable the Rf tx pin
	 */

	if(time_end_flag)
	{
		Uart_flush();
		motor_drive_anticlockwise();
	}

}
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 15999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_14|BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 LED_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SW1_Pin */
  GPIO_InitStruct.Pin = SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(SW1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LASER_Pin */
  GPIO_InitStruct.Pin = LASER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LASER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB14 BUZZER_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SW2_Pin */
  GPIO_InitStruct.Pin = SW2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(SW2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// if timer is elapsed and laser is not short, then send the msg to IOT and RIS ECU
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	count_sec++;
	HAL_UART_Transmit(&huart2, "done\r\n", strlen("done\r\n"), 100);

	if(count_sec==Time_from_Iot)
	{
		Uart_write(0);
		sensor_flag=0;
		memset(Time_Duration,0,sizeof(Time_Duration));
		Uart_flush();
		HAL_TIM_Base_Stop_IT(&htim6);
		time_end_flag=1;
		event_flag=2;
		HAL_UART_Transmit(&huart2, "Time over\r\n",strlen("Time over\r\n"), 100);
		time_flag=0;
		count_sec=0;
	}
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
