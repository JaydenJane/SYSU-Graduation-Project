#ifndef __SPI_H
#define __SPI_H
#include "stm32f10x.h"
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//SPI驱动 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/9
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
#define MPU_9250_DISENABLE  do {PAout(4) = 1; }while(0)
#define MPU_9250_ENABLE  do {PAout(4) = 0; }while(0)

#define USER_CTRL						0X6A //用户配置 当为0X10时使用SPI模式
#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	CONFIG			                        0x1A	
#define	SMPLRT_DIV1		                      0x19	//陀螺仪采样率
#define	GYRO_CONFIG		                      0x1B	
#define	ACCEL_CONFIG	                      0x1C	
#define	ACCEL_CONFIG_2                      0x1D 


#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40

#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
		


void SPI2_Init(void);			 //初始化SPI口
void SPI2_SetSpeed(uint8_t SpeedSet); //设置SPI速度   
uint16_t SPI2_ReadWriteByte(uint16_t TxData);

void SPI1_Init(void);			 //初始化SPI口
void SPI1_SetSpeed(uint8_t SpeedSet); //设置SPI速度   
uint8_t SPI1_ReadWriteByte(uint8_t TxData);//SPI总线读写一个字节
void Init_MPU9250(void);
void READ_MPU9250_ACCEL(int16_t *Accel);
void READ_MPU9250_GYRO(int16_t *Gyro);


#endif

