#include "NJS.h"
#include "stm32f1xx_hal.h"
#include "tim.h"
#include "stm32f1xx_hal_tim.h"

#include <stdio.h>
#include <math.h>

#define human_chess 1
#define PC_chess 0

#define SIZE 3
#define black 2
#define white 1
double board_location[9][3]={{55,220,110},{-1,-1,-1},{-1,-1,-1},{-1,-1,-1},{-1,-1,-1},{-1,-1,-1},{-1,-1,-1},{-1,-1,-1},{-1,-1,-1}};
double white_chequer[5][3]={{92,190,130},{92,225,130},{92,261,130},{92,290,130},{92,320,120}};
double black_chequer[5][3]={{-85,-1,-1},{-1,-1,-1},{-1,-1,-1},{-1,-1,-1},{-1,-1,-1}};
int white_chess[5],black_chess[5],board[9];
extern double theta0,theta1,theta2;
int duty0[3]={50,180,180};


void used_sth(int number,int color){
		switch(color){
			case 1:
				black_chess[number]=black;
				board[number]=black;
				break;
			case 2:
				white_chess[number]=white;
				board[number]=white;
				break;
			default:
				break;
		}

}
void move(int* duty){
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,duty[0]);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,duty[1]);
	//插入四号舵机垂直函数
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,duty[2]);
	//插入四号舵机垂直函数
	
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,duty0[0]);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,duty0[1]);
	//插入四号舵机垂直函数
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,duty0[2]);
	//插入四号舵机垂直函数
}

void inverseKinematics(int number,int* duty,int color) {
    double x,y,z;
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
				
    theta0 = atan2(y, x); 

    double xp = sqrt(x*x + y*y); // 平面投影距离
    double zp = z - BASE_HEIGHT; // 相对肩关节的高度
    double d = sqrt(xp*xp + zp*zp); // 末端到肩关节的直线距离

    // 检查是否可达
    if (d > (SHOULDER_LEN + ELBOW_LEN) || d < fabs(SHOULDER_LEN - ELBOW_LEN)) {
				duty[0]=-1; 
    }

    double alpha = atan2(zp, xp); // 基础角度
    double beta = acos((SHOULDER_LEN*SHOULDER_LEN + d*d - ELBOW_LEN*ELBOW_LEN) 
                      / (2.0 * SHOULDER_LEN * d)); // 余弦定理

    // 计算关节角度 
    theta1 = alpha + beta;          
    theta2 = M_PI-acos((SHOULDER_LEN*SHOULDER_LEN + ELBOW_LEN*ELBOW_LEN - d*d) 
                  / (2.0 * SHOULDER_LEN * ELBOW_LEN)); 

    // 弧度转占空比
    duty[0] = (theta0/M_PI/1.5*0.1+0.025)*2000;
    duty[1] = (theta1/M_PI*0.1+0.025)*2000;
    duty[2] = (theta2/M_PI*0.1+0.025)*2000;

    
}





int check_win(int player) {//player=1||2
    // 检查行和列
    for(int i=0; i<SIZE; i++) {
			if(board[i]==player&&board[i+1]==player&&board[i+2]==player) return 1;
			if(board[i]==player&&board[i+3]==player&&board[i+6]==player) return 1;
		} 
    // 检查对角线
    if(board[0]== player && board[4]== player && board[8]== player) return 1;
    if(board[2] == player && board[4] == player && board[6] == player) return 1;
    return 0;
}




