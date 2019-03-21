#include <stm32f10x.h>
#include <rtthread.h>
#include "sys.h"
#include "MLX90316.h"

#define MLX_CS_HIGH    do {PBout(12) = 1;}while(0)
#define MLX_CS_LOW     do {PBout(12) = 0;}while(0)

#define MLX_IO_WRITE   PBout(15)
#define MLX_IO_READ    PBin(15)
#define MLX_CLK        PBout(13)

GPIO_InitTypeDef GPIO_InitStructure;

#define MLX_IO_DIR_IN do{	\
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;\
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;\
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;\
	GPIO_Init(GPIOB, &GPIO_InitStructure);\
	}while(0)

#define MLX_IO_DIR_OUT do{	\
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;\
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;\
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;\
	GPIO_Init(GPIOB, &GPIO_InitStructure);\
	}while(0)

void mlx90316_Init(void)
{
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );//PORTB ±÷” πƒ‹ 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13 |GPIO_Pin_15;;            
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;           
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;           
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	MLX_CS_HIGH;
}
 uint8_t SpiReadByte(uint8_t tx);

/***********************mlx90316 SPI DATA READ***************************/
int mlx90316_ReadAngle(void) 
 {
	uint8_t bb;
  int ret=-1;
  unsigned int rr;
  int lo;

	MLX_CS_LOW;

	bb=SpiReadByte(0x55);  // send sync byte ( AA reversed order = 55?)
	bb=SpiReadByte(0xFF);  // send sync byte FF)

	bb=SpiReadByte(0xFF); // receive 1. byte of mlx msb data
	rr= (bb << 8);    
	bb=SpiReadByte(0xFF); // receive 2. byte of lsb mlx data
	rr=rr+bb;

  if ((rr & 3) == 2) {
    
    if ( rr & (1 << 4)) ret=-2;  // signal to strong
    if ( rr & (1 << 5)) ret=-3;  // signal to weak
}
    
  if ((rr & 3) == 1) { // valid angle data ?
     rr = (rr >> 2);
     lo= rr ;
     lo=lo *  3600 / 16384;	// scale output to 360 deg, untit in tens of deg.	
     ret= lo;
  }

  MLX_CS_HIGH;
  return(ret);
 } 

 uint8_t SpiReadByte(uint8_t tx)
 {
	int ix;
	uint8_t byte;
  for (ix = 0; ix < 8; ix++){  // receive/transmit 8 SPI bits
			MLX_IO_DIR_OUT;
			// write SPI transmit bit to sensor
			if ((tx & 1) != 0)MLX_IO_WRITE = 1 ; 
			else MLX_IO_WRITE = 0;
			tx= ( tx >> 1);
			MLX_CLK = 1;    // clocksignal positive slope
			MLX_CLK = 0;   // clocksignal negative slope
			MLX_IO_DIR_IN;
			byte = ( byte << 1);           // shift received byte left
			if (MLX_IO_READ == 1) byte = byte | 1; // read respose bit from sensor
			MLX_IO_DIR_OUT;
	}
	return (byte);
 }

