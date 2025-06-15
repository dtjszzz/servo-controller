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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define white 1
#define black 2
#define human_chess 1
#define PC_chess 0
#define SIZE 3
#define human_chess 1
#define PC_chess 0
#define SIZE 3
#define black 2
#define white 1
#define BASE_HEIGHT 85  
#define SHOULDER_LEN 117.0 
#define ELBOW_LEN 250.0    
#define M_PI 3.14159
#define BASE_HEIGHT 85  
#define SHOULDER_LEN 117.0 
#define ELBOW_LEN 250.0    
#define M_PI 3.14159
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
typedef struct{
		uint8_t mode;
		uint8_t color;
		uint8_t number;
		uint8_t board_position;
}Pieces;
Pieces pieces[4];
uint8_t temp_data;
char received_data1[6],received_data2[3];
int pieces_placed,data_cnt1,data_cnt2;
int duty[3]={110,180,180};
int8_t human=-1,PC,first_step,rounds=black,control_cnt;
int8_t using_pieces_black[5],using_pieces__white[5],to_use[2];

double board_location[9][3]={{40,210,50},{10,210,50},{-20,210,50},{40,270,100},{10,266,100},{-20,266,100},{40,300,110},{10,300,110},{-20,300,110}};
double white_chequer[5][3]={{92,190,100},{100,220,100},{100,261,100},{100,300,100},{100,350,100}};
double black_chequer[5][3]={{-85,190,100},{-85,235,100},{-78,261,100},{-78,300,100},{-78,350,100}};
int white_chess[5],black_chess[5],board[9];
extern double theta0,theta1,theta2;
int duty0[3]={110,180,120};
double x,y,z;double theta0,theta1,theta2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
		if(huart==&huart1){
				if(temp_data==0xAA){
					received_data1[data_cnt1]=temp_data;
					data_cnt1++;
					}
				else if(temp_data<128&&temp_data>0){
						received_data1[data_cnt1]=temp_data;
						data_cnt1++;
				}
				if(temp_data==0xBB){
						data_cnt1=0;
						pieces[pieces_placed].mode=received_data1[1];
						pieces[pieces_placed].color=received_data1[2];
						pieces[pieces_placed].number=received_data1[3];
						pieces[pieces_placed].board_position=received_data1[4];
						memset(received_data1,0,sizeof(received_data1));
						pieces_placed++;
				}
				if(pieces_placed==4){pieces_placed=0;}
				if(temp_data==0xCC){
						received_data2[0]=temp_data;
				}
				
			HAL_UART_Receive_IT(&huart1,&temp_data,1);
		}
}

void move(int* duty){
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,duty[0]);
	HAL_Delay(500);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,duty[1]);
	HAL_Delay(500);
	//??????????
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,duty[2]);
	//??????????
	HAL_Delay(2000);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,duty0[1]);
	HAL_Delay(500);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,duty0[0]);
	HAL_Delay(500);
	//??????????
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,duty0[2]);
	HAL_Delay(500);
	//??????????
}

void inverseKinematics(int numberin,int* duty,int color) {
    
		int number=numberin-1;
		switch(color){
			case 1:
				x=black_chequer[number][0];
				y=black_chequer[number][1];
				z=black_chequer[number][2];
				break;
			case 2:
				x=white_chequer[number][0];
				y=white_chequer[number][1];
				z=white_chequer[number][2];
				break;
			case 0:
				x=board_location[number][0];
				y=board_location[number][1];
				z=board_location[number][2];
			default:
				break;
		}
				
    theta0 = atan2(y,x); 

    double xp = sqrt(x*x + y*y); // ??????
    double zp = z - BASE_HEIGHT; // ????????
    double d = sqrt(xp*xp + zp*zp); // ???????????

    // ??????
    if (d > (SHOULDER_LEN + ELBOW_LEN) || d < fabs(SHOULDER_LEN - ELBOW_LEN)) {
				duty[0]=-1; 
    }

    double alpha = atan2(zp, xp); // ????
    double beta = acos((SHOULDER_LEN*SHOULDER_LEN + d*d - ELBOW_LEN*ELBOW_LEN) 
                      / (2.0 * SHOULDER_LEN * d)); // ????

    // ?????? 
    theta1 = alpha + beta;          
    theta2 = M_PI-acos((SHOULDER_LEN*SHOULDER_LEN + ELBOW_LEN*ELBOW_LEN - d*d) 
                  / (2.0 * SHOULDER_LEN * ELBOW_LEN)); 

    // ??????
    duty[0] = (theta0/M_PI/1.5*0.1+0.025)*2000;
    duty[1] = (theta1/M_PI*0.1+0.025)*2000;
    duty[2] = (theta2/M_PI*0.1+0.025)*2000;

    
}





int check_win(int player) {//player=1||2
    // ?????
    for(int i=0; i<SIZE; i++) {
			if(board[i]==player&&board[i+1]==player&&board[i+2]==player) return 1;
			if(board[i]==player&&board[i+3]==player&&board[i+6]==player) return 1;
		} 
    // ?????
    if(board[0]== player && board[4]== player && board[8]== player) return 1;
    if(board[2] == player && board[4] == player && board[6] == player) return 1;
    return 0;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);	
	  move(duty);
		HAL_UART_Receive_IT(&huart1,&temp_data,1);
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==GPIO_PIN_SET){
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
			while(pieces[control_cnt].number!=0x00 && control_cnt<pieces_placed){
					inverseKinematics(pieces[control_cnt].number,duty,pieces[control_cnt].color);
					HAL_Delay(1000);
					move(duty);
					HAL_Delay(1000);
					inverseKinematics(pieces[control_cnt].board_position,duty,0);
					HAL_Delay(1000);
					move(duty);
					HAL_Delay(1000);
					control_cnt++;
			}
			
			}
		else if (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==GPIO_PIN_RESET){HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);}
			
		if(human!=-1){PC=1-human;}
		if(human==white&&first_step==0&&pieces_placed!=0){
				inverseKinematics(pieces[pieces_placed].number,duty,pieces[pieces_placed].color);
				//used_sth(pieces[pieces_placed].number,pieces[pieces_placed].color);//board+pieces used
				move(duty);//moving the servo to the pieces and reset
				inverseKinematics(pieces[pieces_placed].board_position,duty,0);//moving to the position to place the pieces
				move(duty);
			  rounds=white;first_step=1;//change rounds
		}
		if(rounds==human){
				inverseKinematics(pieces[pieces_placed].number,duty,pieces[pieces_placed].color);
				move(duty);
				inverseKinematics(pieces[pieces_placed].board_position,duty,0);
				move(duty);
				rounds=PC;
				check_win(human);
				if(first_step==0){first_step=1;}
		}
		else if(rounds==PC){
			  //PC_thinking(board,PC,using_pieces,to_use);
				inverseKinematics(to_use[0],duty,pieces[pieces_placed].color);
				move(duty);
				inverseKinematics(to_use[1],duty,0);
				move(duty);
				to_use[0]=0;to_use[1]=0;
				check_win(PC);
		}
		
			//transimit the position of the human placed pieces 

}//battling{}
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  
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
