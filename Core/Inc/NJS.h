#ifndef NJS_H
#define NJS_H

#define BASE_HEIGHT 85  
#define SHOULDER_LEN 117.0 
#define ELBOW_LEN 250.0    
#define M_PI 3.14159

void inverseKinematics(double* position,int* duty);

void numberhandler(int flag_chess,int number,int* duty,char player);

void chequter_init();


#endif
