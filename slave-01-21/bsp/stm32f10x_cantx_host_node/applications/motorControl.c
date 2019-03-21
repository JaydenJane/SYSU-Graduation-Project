#include <rtthread.h>
#include "can.h"
#include "stm32f10x.h"
#include "string.h"
#include "sys.h"

void motorInit()
{
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;  
    TIM_OCInitTypeDef  TIM_OCInitStructure;  
    GPIO_InitTypeDef    GPIO_InitStructure;  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE); //外设时钟使能
    
      
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //推挽输出 使能TIM3_CH3  复用PB0
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; 
    GPIO_Init(GPIOB,&GPIO_InitStructure);     
  
    TIM_TimeBaseStructure.TIM_Period =9999;           //自动重载的周期值  
    TIM_TimeBaseStructure.TIM_Prescaler =143;         //预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;    //时钟分频值  
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上记数模式    
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);  
                                                                        
    TIM_OCStructInit(& TIM_OCInitStructure);       
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  //PWM模式1
    TIM_OCInitStructure.TIM_Pulse =0;                  
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性为高    
      
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);     //TIM3写入通道3
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);//预装载使能  
		
    TIM_Cmd(TIM3, ENABLE);  
    TIM_CtrlPWMOutputs(TIM3, ENABLE);  

}
// 参考--->>  http://blog.csdn.net/scliu12345/article/details/38899397#

/*void SetJointAngle(float angle) 
{
	
		angle=angle+90.0;                  // -90 < angle < 90    
		angle=(u16)(50.0*angle/9.0+249.0);       
		TIM_SetCompare1(TIM3,angle);          
}*/