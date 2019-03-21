#ifndef __SPI_H
#define __SPI_H
#include "stm32f10x.h"
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//SPI���� ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/9
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
#define MPU_9250_DISENABLE  do {PAout(4) = 1; }while(0)
#define MPU_9250_ENABLE  do {PAout(4) = 0; }while(0)

#define USER_CTRL						0X6A //�û����� ��Ϊ0X10ʱʹ��SPIģʽ
#define	PWR_MGMT_1		0x6B	//��Դ��������ֵ��0x00(��������)
#define	CONFIG			                        0x1A	
#define	SMPLRT_DIV1		                      0x19	//�����ǲ�����
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
		


void SPI2_Init(void);			 //��ʼ��SPI��
void SPI2_SetSpeed(uint8_t SpeedSet); //����SPI�ٶ�   
uint16_t SPI2_ReadWriteByte(uint16_t TxData);

void SPI1_Init(void);			 //��ʼ��SPI��
void SPI1_SetSpeed(uint8_t SpeedSet); //����SPI�ٶ�   
uint8_t SPI1_ReadWriteByte(uint8_t TxData);//SPI���߶�дһ���ֽ�
void Init_MPU9250(void);
void READ_MPU9250_ACCEL(int16_t *Accel);
void READ_MPU9250_GYRO(int16_t *Gyro);


#endif

