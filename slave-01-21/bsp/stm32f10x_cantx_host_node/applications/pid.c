#include <rtthread.h>
#include <string.h>
#include <stdio.h>
#include "stm32f10x.h"
#include "pid.h"
#include "pwm.h"

// refrence document
// https://github.com/but0n/Avem/blob/7661b46ad8983f6aecb4b9d964ed8054a1f85a85/docs/README.md
#define PWM_PID_CAL_FREQ   10
#define dt 	PWM_PID_CAL_FREQ*1e-3
#define _filter		7.9577e-3
static  PID _pid;
#define NUM_HISTORY  10
int error_history[NUM_HISTORY];
uint8_t error_id = 0;
void pid_init(void)
{
	//_pid.p = 4;
	//_pid.i = 0.05;
	//_pid.d = 0.6;
	_pid.p = 0.3;
	_pid.i = 0.02;
	_pid.d = 0;
//	_pid.p = 3;
//	_pid.i = 0.01;
	
	for(int i=0; i<NUM_HISTORY; ++i){
		error_history[i] = 0;
	}
	
	
//	_pid.d = 0.05;
	_pid.integrator = 0;
	_pid.previous_err = 0;
	_pid.integrator_limit = 200;
	_pid.last_derivate = 0;
}

float pid_get_integral(void)
{
	return _pid.integrator;
}
void reset_I()
{
	_pid.integrator = 0;
}
	

int pid(int nowAngle,int setAngle,int outMAX,int outMIN) {
	
	int error = nowAngle - setAngle;
	error_history[error_id] = error;
	++error_id;
	if(error_id >= NUM_HISTORY){
		error_id = 0;
	}
	_pid.integrator = 0;
	for(int i=0; i<NUM_HISTORY; ++i){
		_pid.integrator += error_history[i] ;
	}
	
	if (_pid.integrator > _pid.integrator_limit) {
	    _pid.integrator = _pid.integrator_limit;
	}
	else if(_pid.integrator < -1 * _pid.integrator_limit) {
			_pid.integrator = -1 * _pid.integrator_limit;
	}
//	_pid.output  = -1 * _pid.p * error;
//	_pid.output += -1 * _pid.i * _pid.integrator / PWM_PID_CAL_FREQ;
//	_pid.output += -1 * _pid.d * (error-_pid.previous_err) * PWM_PID_CAL_FREQ;
//	_pid.output  = -1 * _pid.p * error;
//	_pid.output -= -1 * _pid.i * _pid.integrator;
//	_pid.output += -1 * _pid.d * (error-_pid.previous_err);	
	float derivate=0;
	derivate = (error - _pid.previous_err) / (dt);
	derivate = _pid.last_derivate + (dt/(_filter + dt) * (derivate - _pid.last_derivate));
	
	_pid.output  = -1 * _pid.p * error;
	_pid.output += -1 * _pid.i * _pid.integrator;
	_pid.output += -1 * _pid.d * derivate;
	
	_pid.previous_err = error;
	_pid.last_derivate = derivate;
	
	if(_pid.output > outMAX) _pid.output = outMAX;
	if(_pid.output < outMIN) _pid.output = outMIN;

	return _pid.output;
}

#include <finsh.h>

static int func(char *p_str,char *i_str,char *d_str)
{
	float kp, ki, kd;
	char float_str[20];  

	kp = atof(p_str);
	ki = atof(i_str);
	kd = atof(d_str);
	_pid.p = kp;
	_pid.i = ki;
	_pid.d = kd;
	sprintf(float_str, "P is %f\n", kp); 
	rt_kprintf(float_str);
	sprintf(float_str, "I is %f\n", ki); 
	rt_kprintf(float_str);
	sprintf(float_str, "D is %f\n", kd); 
	rt_kprintf(float_str);
	return 0;
}
FINSH_FUNCTION_EXPORT(func, func test)

static int print_pid_gains(void)
{
  char float_str[20];
 	sprintf(float_str, "P is %f\n", _pid.p); 
	rt_kprintf(float_str);
	sprintf(float_str, "I is %f\n", _pid.i); 
	rt_kprintf(float_str);
	sprintf(float_str, "D is %f\n", _pid.d); 
	rt_kprintf(float_str);
	return 0;
}
FINSH_FUNCTION_EXPORT(print_pid_gains, print pid parameters)
