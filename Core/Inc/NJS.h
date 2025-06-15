#ifndef NJS_H
#define NJS_H

#define BASE_HEIGHT 85  
#define SHOULDER_LEN 117.0 
#define ELBOW_LEN 250.0    
#define M_PI 3.14159


void used_sth(int number,int color);

void move(int* duty);

void inverseKinematics(int number,int* duty,int color);

int check_win(int player);

#endif
