#include "stm32f10x.h"                  // Device header
#include "spi.h"
#include <rtthread.h>
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
 
//以下是SPI模块的初始化代码，配置成主机模式，访问SD Card/W25Q64/NRF24L01						  
//SPI口初始化

#define Delay_Ms(ms)                     rt_thread_delay(ms)


unsigned char BUF[6];       //接收数据缓存区

static inline void _delay_us(int us)
{
	for (int i = us;i > 0;i--)
		for (int j = 100;j > 0;j--);
}

void SPI2_Init(void)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;

	RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO, ENABLE);  //复用功能IO使能
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );//PORTB时钟使能 
	RCC_APB1PeriphClockCmd(	RCC_APB1Periph_SPI2,  ENABLE );//SPI2时钟使能 	
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13  |GPIO_Pin_14| GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //PB13/15复用推挽输出 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOB

 	GPIO_SetBits(GPIOB,GPIO_Pin_13 |GPIO_Pin_14| GPIO_Pin_15);  //PB13/15上拉
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;            
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;           
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;           
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;		//设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//串行同步时钟的空闲状态为高电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//串行同步时钟的第二个跳变沿（上升或下降）数据被采样
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;		//定义波特率预分频的值:波特率预分频值为256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式
	SPI_Init(SPI2, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
 
	SPI_Cmd(SPI2, ENABLE); //使能SPI外设
	SPI2_ReadWriteByte(0xffff);//启动传输		 
}   

//SPI 速度设置函数
//SpeedSet:
//SPI_BaudRatePrescaler_2   2分频   
//SPI_BaudRatePrescaler_8   8分频   
//SPI_BaudRatePrescaler_16  16分频  
//SPI_BaudRatePrescaler_256 256分频 
  
void SPI2_SetSpeed(uint8_t SPI_BaudRatePrescaler)
{
  assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));
	SPI2->CR1&=0XFFC7;
	SPI2->CR1|=SPI_BaudRatePrescaler;	//设置SPI2速度 
	SPI_Cmd(SPI2,ENABLE); 
} 

//SPIx 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
uint16_t SPI2_ReadWriteByte(uint16_t TxData)
{		
	uint8_t retry=0;				 	
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) //检查指定的SPI标志位设置与否:发送缓存空标志位
		{
		retry++;
		if(retry>200)return 0;
		}			  
		SPI_I2S_SendData(SPI2, TxData); //通过外设SPIx发送一个数据
		retry=0;

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)//检查指定的SPI标志位设置与否:接受缓存非空标志位
		{
		retry++;
		if(retry>200)return 0;
		}	  						    
	return SPI_I2S_ReceiveData(SPI2); //返回通过SPIx最近接收的数据					    
}


void SPI1_Init(void)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_SPI1, ENABLE );//PORTA SPI1 时钟使能 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //5/6/7复用推挽输出 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA

 	GPIO_SetBits(GPIOA, GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7);  //PA5/6/7上拉

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;            
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;           
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;           
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_4);
	
	SPI_Cmd(SPI1,DISABLE);  //SPI2不使能
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;//SPI_CPOL_High;		//串行同步时钟的空闲状态为高电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;//SPI_CPHA_2Edge;	//串行同步时钟的第二个跳变沿（上升或下降）数据被采样
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;		//定义波特率预分频的值:波特率预分频值为256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式
	SPI_Init(SPI1, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
 
	SPI_Cmd(SPI1, ENABLE); //使能SPI外设
	//MPU_9250_DISENABLE;
	//SPI1_ReadWriteByte(0xff);//启动传输		 
 
}   
//SPI 速度设置函数
//SpeedSet:
//SPI_BaudRatePrescaler_2   2分频   
//SPI_BaudRatePrescaler_8   8分频   
//SPI_BaudRatePrescaler_16  16分频  
//SPI_BaudRatePrescaler_256 256分频 
  
void SPI1_SetSpeed(uint8_t SPI_BaudRatePrescaler)
{
  assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));
	SPI1->CR1&=0XFFC7;
	SPI1->CR1|=SPI_BaudRatePrescaler;	//设置SPI1速度 
	SPI_Cmd(SPI1,ENABLE); 

} 

//SPIx 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
uint8_t SPI1_ReadWriteByte(uint8_t TxData)
{		
	uint8_t retry=0;				 	
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) //检查指定的SPI标志位设置与否:发送缓存空标志位
		{
		retry++;
		if(retry>200)return 0;
		}			  
		SPI_I2S_SendData(SPI1, TxData); //通过外设SPIx发送一个数据
		retry=0;

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)//检查指定的SPI标志位设置与否:接受缓存非空标志位
		{
		retry++;
		if(retry>200)return 0;
		}	  						    
	return SPI_I2S_ReceiveData(SPI1); //返回通过SPIx最近接收的数据					    
}

static u8 MPU9250_Write_Reg(u8 reg,u8 value)
{
	u8 status;
	MPU_9250_ENABLE;   //	MPU9250_CS=0;  //片选MPU9250
	_delay_us(100);
	status=SPI1_ReadWriteByte(reg); //发送reg地址
	SPI1_ReadWriteByte(value);//发送数据
	_delay_us(100);
	MPU_9250_DISENABLE;//	MPU9250_CS=1;  //失能MPU9250
	return(status);//
}
//---------------------------------------------------------------//
//SPI读取
//reg: addr
static u8 MPU9250_Read_Reg(u8 reg)
{
	  u8 reg_val;
		MPU_9250_ENABLE;//	MPU9250_CS=0;  //片选MPU9250
	  _delay_us(100);
	  SPI1_ReadWriteByte(reg|0x80); //reg地址+读命令
	  reg_val=SPI1_ReadWriteByte(0xff);//任意数据
	  _delay_us(100);
		MPU_9250_DISENABLE;//	MPU9250_CS=1;  //失能MPU9250
	return(reg_val);
}

void Init_MPU9250(void)
{	
	SPI1_Init();
	Delay_Ms(100);
	//MPU9250_Write_Reg(PWR_MGMT_1, 0x00);	//解除休眠状态
	//MPU9250_Write_Reg(CONFIG, 0x07);      //低通滤波频率，典型值：0x07(3600Hz)此寄存器内决定Internal_Sample_Rate==8K
	
/*******************Init GYRO and ACCEL******************************/	
	/*MPU9250_Write_Reg(SMPLRT_DIV1, 0x07);  //陀螺仪采样率，典型值：0x07(1kHz) (SAMPLE_RATE= Internal_Sample_Rate / (1 + SMPLRT_DIV) )
	MPU9250_Write_Reg(GYRO_CONFIG, 0x18); //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
	MPU9250_Write_Reg(ACCEL_CONFIG, 0x18);//加速计自检、测量范围及高通滤波频率，典型值：0x18(不自检，16G)
	MPU9250_Write_Reg(ACCEL_CONFIG_2, 0x08);//加速计高通滤波频率 典型值 ：0x08  （1.13kHz）		
*/
		MPU9250_Write_Reg(USER_CTRL,0X10); //使能MPU9250SPI
		MPU9250_Write_Reg(PWR_MGMT_1,0X80);  //电源管理,复位MPU9250
		MPU9250_Write_Reg(SMPLRT_DIV1,0x07);//采样率1000/(1+7)=125HZ
		MPU9250_Write_Reg(CONFIG,0X06);				//低通滤波器 0x06 5hz
		MPU9250_Write_Reg(GYRO_CONFIG,0X18);  //陀螺仪测量范围 0X18 正负2000度
		MPU9250_Write_Reg(ACCEL_CONFIG,0x18); //加速度计测量范围 0X18 正负16g
	}

//************************加速度读取**************************/
void READ_MPU9250_ACCEL(int16_t *Accel)
{ 

   BUF[0]=MPU9250_Read_Reg(ACCEL_XOUT_L); 
   BUF[1]=MPU9250_Read_Reg(ACCEL_XOUT_H);
   Accel[0]=	(BUF[1]<<8)|BUF[0];
   Accel[0]/=164; 						   //读取计算X轴数据
   BUF[2]=MPU9250_Read_Reg(ACCEL_YOUT_L);
   BUF[3]=MPU9250_Read_Reg(ACCEL_YOUT_H);
   Accel[1]=	(BUF[3]<<8)|BUF[2];
   Accel[1]/=164; 						   //读取计算Y轴数据
   BUF[4]=MPU9250_Read_Reg(ACCEL_ZOUT_L); 
   BUF[5]=MPU9250_Read_Reg(ACCEL_ZOUT_H);
   Accel[2]=  (BUF[5]<<8)|BUF[4];
   Accel[2]/=164; 					      //读取计算Z轴数据 
}
/**********************陀螺仪读取*****************************/
void READ_MPU9250_GYRO(int16_t *Gyro)
{ 

   BUF[0]=MPU9250_Read_Reg(GYRO_XOUT_L); 
   BUF[1]=MPU9250_Read_Reg(GYRO_XOUT_H);
   Gyro[0]=	(BUF[1]<<8)|BUF[0];
   Gyro[0]/=16.4; 						   //读取计算X轴数据

   BUF[2]=MPU9250_Read_Reg(GYRO_YOUT_L);
   BUF[3]=MPU9250_Read_Reg(GYRO_YOUT_H);
   Gyro[1]=	(BUF[3]<<8)|BUF[2];
   Gyro[1]/=16.4; 						   //读取计算Y轴数据
   BUF[4]=MPU9250_Read_Reg(GYRO_ZOUT_L);
   BUF[5]=MPU9250_Read_Reg(GYRO_ZOUT_H);
   Gyro[2]=	(BUF[5]<<8)|BUF[4];
   Gyro[2]/=16.4; 					       //读取计算Z轴数据
}


