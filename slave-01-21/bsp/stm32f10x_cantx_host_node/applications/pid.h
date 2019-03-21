#ifndef __PID_H
#define __PID_H

#define OUTTER_LOOP_KP 0 //0.257 * 0.83 0.255
#define OUTTER_LOOP_KI 0
#define OUTTER_LOOP_KD 0

#define INNER_LOOP_KP 0.03f
#define INNER_LOOP_KI 0
#define INNER_LOOP_KD 0

#define SUM_ERRO_MAX  900
#define SUM_ERRO_MIN -900

#define PID_IMAX 30
#define PID_IMIN -30


typedef struct {
    float   previous_err;
    float 	p;
    float 	i;
    float 	d;
		float   integrator;
	  float   integrator_limit;
    int 	  output;
		float   last_derivate;
} PID;

void pid_init(void);
int  pid(int nowAngle,int setAngle,int outMAX,int outMIN);
void reset_I();
#endif
