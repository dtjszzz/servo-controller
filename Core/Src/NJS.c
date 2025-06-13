#include "NJS.h"

#include <stdio.h>
#include <math.h>

#define human_chess 1
#define PC_chess 0

#define SIZE 3
char board[3][3];
double board_location[9][4]={{55,220,110},{-1,-1,-1},{-1,-1,-1},{-1,-1,-1},{-1,-1,-1},{-1,-1,-1},{-1,-1,-1},{-1,-1,-1},{-1,-1,-1}};
double white_chequer[5][4]={{80,190,130},{80,225,130},{80,261,130},{80,290,130},{90,320,120}};
double black_chequer[5][4]={{-1,-1,-1},{-1,-1,-1},{-1,-1,-1},{-1,-1,-1},{-1,-1,-1}};
int haveused_black=0,haveused_white=0;
char human,PC;
int rounds;



void chequter_init(){
	haveused_black=0;haveused_white=0;
	for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){board[i][j]=0;}
	}

}

void use_block(int number,char player){
	if(player==human_chess){}
}
void inverseKinematics(double* position,int* duty) {
    double x,y,z;double theta0,theta1,theta2;
    x=position[0];y=position[1];z=position[2];
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

void judge();//判断是否可达或传输包是否合理
void bag_handler(char bag[5]){
	if(1)//此处用于读取串口数据是否为人选择黑棋
	{human=1;PC=0;}
	else {human=0;PC=1;}
	//下面省略一堆用于读取改写串口包或直接取用串口包数值的代码……
		
}
void battle(){
	rounds=1-human;
	if(rounds==0){}//AI回合
	else{}//人类回合
	
}

int check_win(char player) {//player=0||1
    // 检查行和列
    for(int i=0; i<SIZE; i++) {
        if(board[i][0] == player && board[i][1] == player && board[i][2] == player) return 1;
        if(board[0][i] == player && board[1][i] == player && board[2][i] == player) return 1;
    }
    // 检查对角线
    if(board[0][0] == player && board[1][1] == player && board[2][2] == player) return 1;
    if(board[0][2] == player && board[1][1] == player && board[2][0] == player) return 1;
    return 0;
}

void numberhandler(int flag_chess,int number,int *duty,char player){
	switch(flag_chess){
		case 0:
			inverseKinematics(white_chequer[haveused_white],duty);
			
			haveused_white++;
			if(haveused_white>4) haveused_white=0;
			break;
		case -1:
			inverseKinematics(board_location[number],duty);
			board[number/3][number%3]=player;
			break;
		case 1:
			inverseKinematics(black_chequer[haveused_black],duty);
			haveused_black++;
			if(haveused_black>4) haveused_black=0;
			break;
		default:
			break;
	
	}
}



