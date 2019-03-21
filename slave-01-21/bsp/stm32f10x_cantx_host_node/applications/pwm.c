#include <rtthread.h>
#include <stdlib.h>
#include "can.h"
#include "stm32f10x.h"
#include "string.h"
#include "sys.h"
#include "MLX90316.h"
#include "pwm.h"
#include "pid.h"
#include "mpu9250.h"
#include "math.h"
ALIGN(RT_ALIGN_SIZE)
static rt_timer_t PWMTimer;


int joint_id = 8;
int MotorStopCNT;

int outMAX =  500;
int outMIN = -500;

int ReadJointAngle;

int min_angle = 0;
int max_angle = 0;
int MidAngle = 0;
uint8_t initialized = 0;

int SetJointAngle;
void JointAngleInit(){
	switch(joint_id){
		case 1:{
			max_angle = 2050;
			min_angle = 193;
			 MotorStopCNT = 1550;
			break;
		}
		case 2:{
			max_angle = 2701;
			min_angle = 867;
			MotorStopCNT = 1555;
			break;
		}
		case 3:{
			max_angle = 2771;
			min_angle = 971;
			MotorStopCNT = 1560;
			break;
		}
		case 4:{
			max_angle = 2821;
			min_angle = 1001;
			MotorStopCNT = 1543;
			break;
		}
		case 5:{
			max_angle = 2715;
			min_angle = 916;
			MotorStopCNT = 1555;
			break;
		}
		case 6:{
			max_angle = 2653;
			min_angle = 852;
			MotorStopCNT = 1552;
			break;
		}
		case 7:{
			max_angle = 2782;
			min_angle = 944;
			MotorStopCNT = 1550;
			break;
		}
		case 8:{
			max_angle = 2749;
			min_angle = 922;
			MotorStopCNT = 1560;
			break;
		}	
		case 9:{
			max_angle = 2710;
			min_angle = 913;
			MotorStopCNT = 1560;
			break;
		}		
		case 10:{
			max_angle = 2081;
			min_angle = 237;
			MotorStopCNT = 1557;
			break;
		}	
		case 11:{
			max_angle = 2700;
			min_angle = 892;
			MotorStopCNT = 1556;
			break;
		}	
		case 12:{
			max_angle = 2643;
			min_angle = 814;
			MotorStopCNT = 1555;
			break;
		}
		case 13:{
			max_angle = 2020;
			min_angle = 274;
			MotorStopCNT = 1553;
			break;
		}	
		case 14:{
			max_angle = 2575;
			min_angle = 776;
			MotorStopCNT = 1558;
			break;
		}	
		case 15:{
			max_angle = 2770;
			min_angle = 917;
			MotorStopCNT = 1518;
			break;
		}			
		case 16:{
			max_angle = 2745;
			min_angle = 912;
			MotorStopCNT = 1550;
			break;
		}	
		case 17:{
			max_angle = 2588;
			min_angle = 1032;
			MotorStopCNT = 1548;
			break;
		}	
		}
	  MidAngle = (max_angle + min_angle)/2;
		SetJointAngle = 0;	
		
}
void MotorPWMInit(void)
{		
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;  
    TIM_OCInitTypeDef  			 TIM_OCInitStructure;  
    GPIO_InitTypeDef    		 GPIO_InitStructure;  
	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE); //外设时钟使能
    
      
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //推挽输出 使能TIM3_CH3  复用PB0
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; 
    GPIO_Init(GPIOB,&GPIO_InitStructure);     
    //set PWM Freq 50hz  Freq = 72M /(TIM_Prescaler+1) /(TIM_Period + 1)
	  //Freq --> 333hz
    TIM_TimeBaseStructure.TIM_Period 		=	19999;           			 //自动重载的周期值  
    TIM_TimeBaseStructure.TIM_Prescaler =	71;         					 //预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;    	 //时钟分频值  
    TIM_TimeBaseStructure.TIM_CounterMode 	= TIM_CounterMode_Up;//向上记数模式    
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);  
                                                                        
    TIM_OCStructInit(&TIM_OCInitStructure);       
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  //PWM模式1
    TIM_OCInitStructure.TIM_Pulse = 0;                  
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性为高    
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);     		 //TIM3写入通道3
		
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);//预装载使能  
    //TIM_ARRPreloadConfig(TIM3,ENABLE);//使能TIM在ARR的预装载寄存器
		TIM_Cmd(TIM3, ENABLE); 
		JointAngleInit();
		pid_init();
		initialized = 1;
}

void Set_Joint_Angel(int angle)
{
	SetJointAngle = angle;
}
// angle range 0 ~ 3600
// pwm pulse width 900~2100us  set pwm width to 1520us stop the motor
int c = 0;
int flag = 0;
extern short gyr[3],accl[3];
extern uint16_t Angle;
static void PWMTimerOut(void* parameter)
{
	//if(Angle < 3601 && SetJointAngle < 3601){
	  int error;
	  int out = 0;
	  static int last_out;
	if (initialized == 0){
		TIM_SetCompare3(TIM3,MotorStopCNT);
		return;
	}
	
		rt_kprintf("%d \n", Angle );
		
		//these two lines are used to measure the stop point from input using m_StopCNT()
		//TIM_SetCompare3(TIM3,MotorStopCNT);
		//return;
	
		//uint8_t read_angle[2];
	
		//MPU9250_Set_Value(NULL,NULL,read_angle);
	
		//ReadJointAngle=((int)read_angle[0]<<8)|read_angle[1];
	  //ReadJointAngle = mlx90316_ReadAngle();
		
		//printf("%d\n",rt_tick_get());		
	  error =  -Angle + (SetJointAngle + MidAngle) ; 
		
		//																																												rt_kprintf("angle : %d",Angle);
		// TODO:pid ctrl 
		//rt_kprintf("%d -- %d \n",ReadJointAngle,SetJointAngle);

			c = c + 1;
			out = pid(Angle,(SetJointAngle + MidAngle),outMAX,outMIN);
			if(last_out != out) {
			//rt_kprintf("out: %d\n",out);
			 //rt_kprintf("angle:%d",Angle);
			 last_out = out;
			}
			TIM_SetCompare3(TIM3,MotorStopCNT+out);

//		else {
//			TIM_SetCompare3(TIM3,MotorStopCNT);//stop the motor
//			reset_I();
//			if(!flag)
//			{
//				//rt_kprintf("final count: %d\n",c);
//				//rt_kprintf("final angle:%d",Angle);
//				flag = 1;
//				c = 0;
//			}
//			//pid_init();
//		}
	//}
}

void PWMTimerInit(void)
{

	PWMTimer = rt_timer_create("timer1", 
															PWMTimerOut, 
															RT_NULL, 
															100,          /*10 ticks, so pwm refresh use 100 HZ frequency*/
															RT_TIMER_FLAG_PERIODIC); 

	if (PWMTimer != RT_NULL) rt_timer_start(PWMTimer);
}
#include <finsh.h>
static int set_r_angle(int angle)
{
	flag = 1;
	c = 0;
	if(angle < 0) SetJointAngle = 0;
	else if(angle > 3600) SetJointAngle = 3600;
	else SetJointAngle = angle;
	
	return 0;
}
static int sim_joint_angle(int angle)
{
	flag = 0;
	c=0;
	if(angle < 0) ReadJointAngle = 0;
	else if(angle > 3600) ReadJointAngle = 3600;
	else ReadJointAngle = angle;
	
	return 0;
}
static int get_joint_angle(void)
{
	rt_kprintf("joint angle:%d\n",Angle);
	
	return 0;
}
static int m_StopCNT(int StopCNT){
	

	MotorStopCNT=StopCNT ;
	return 0;
}

static void test_pwm(int cnt)
{
	TIM_SetCompare3(TIM3,cnt);
}
FINSH_FUNCTION_EXPORT(sim_joint_angle,simulate input joint data)
FINSH_FUNCTION_EXPORT(m_StopCNT,set StopCNT)
FINSH_FUNCTION_EXPORT(set_r_angle,set joint angle)
FINSH_FUNCTION_EXPORT(get_joint_angle,set get_joint_angle )
FINSH_FUNCTION_EXPORT(test_pwm,test_pwm)