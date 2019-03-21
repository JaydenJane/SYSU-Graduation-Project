#include <rtthread.h>
#include "can.h"
#include "stm32f10x.h"
#include "string.h"
#include "sys.h"
#include "mpu9250.h"
#include "spi.h"
#include "MLX90316.h"
#include "board.h"
#include "pwm.h"
#include "math.h"

#ifdef USING_MPU9250
//////////////////////////////////////////////////////////////////////////
static s16 MPU9250_AK8963_ASA[3] = {0, 0, 0};
//////////////////////////////////////////////////////////////////////////


#define MPU9250_SPIx_SendByte(byte)      SPI1_ReadWriteByte(byte)
#define MPU9250_SPIx_SetDivisor(divisor) SPI1_SetSpeed(divisor)
#define Delay_Ms(ms)                     rt_thread_delay(ms)
#define MPU9250_Dev_SELECT             	 do {PAout(4) = 0; }while(0)
#define MPU9250_Dev_DESELECT       			 do {PAout(4) = 1; }while(0)

//////////////////////////////////////////////////////////////////////////



static inline void _delay_us(int us)
{
	for (int i = us;i > 0;i--)
		for (int j = 100;j > 0;j--);
}

//init
void MPU9250_Init(void)
{
	uint8_t data = 0, state = 0;
	uint8_t response[3] = {0, 0, 0};
	//Lower level hardware Init
	SPI1_Init();
	//EXTIx_Init(pMPU9250INT);
  Delay_Ms(100);
	//MPU9250 Reset
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_PWR_MGMT_1, MPU9250_RESET);
	Delay_Ms(100);
	//MPU9250 Set Clock Source
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_PWR_MGMT_1,  MPU9250_CLOCK_PLLGYROZ);
	Delay_Ms(1);
	//MPU9250 Set Interrupt
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_INT_PIN_CFG,  MPU9250_INT_ANYRD_2CLEAR);
	Delay_Ms(1);
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_INT_ENABLE, ENABLE);
	Delay_Ms(1);
	//MPU9250 Set Sensors
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_PWR_MGMT_2, MPU9250_XYZ_GYRO & MPU9250_XYZ_ACCEL);
	Delay_Ms(1);
	//MPU9250 Set SampleRate
	//SAMPLE_RATE = Internal_Sample_Rate / (1 + SMPLRT_DIV)
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_SMPLRT_DIV, SMPLRT_DIV);
	Delay_Ms(1);
	//MPU9250 Set Full Scale Gyro Range
	//Fchoice_b[1:0] = [00] enable DLPF
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_GYRO_CONFIG, (MPU9250_FSR_2000DPS << 3));
	Delay_Ms(1);
	//MPU9250 Set Full Scale Accel Range PS:2G
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_ACCEL_CONFIG, (MPU9250_FSR_16G << 3));
	Delay_Ms(1);
	//MPU9250 Set Accel DLPF
	data = MPU9250_SPIx_Read(MPU9250_SPIx_ADDR, MPU9250_ACCEL_CONFIG2);
	data |= MPU9250_ACCEL_DLPF_41HZ;
	Delay_Ms(1);
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_ACCEL_CONFIG2, data);
	Delay_Ms(1);
	//MPU9250 Set Gyro DLPF
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_CONFIG, MPU9250_GYRO_DLPF_41HZ);
	Delay_Ms(1);
	//MPU9250 Set SPI Mode
	state = MPU9250_SPIx_Read(MPU9250_SPIx_ADDR, MPU9250_USER_CTRL);
	Delay_Ms(1);
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_USER_CTRL, state | MPU9250_I2C_IF_DIS);
	Delay_Ms(1);
	state = MPU9250_SPIx_Read(MPU9250_SPIx_ADDR, MPU9250_USER_CTRL);
	Delay_Ms(1);
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_USER_CTRL, state | MPU9250_I2C_MST_EN);
	Delay_Ms(1);
	//////////////////////////////////////////////////////////////////////////
	//AK8963 Setup
	//reset AK8963
	MPU9250_AK8963_SPIx_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL2, MPU9250_AK8963_CNTL2_SRST);
	Delay_Ms(2);

	MPU9250_AK8963_SPIx_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_POWER_DOWN);
	Delay_Ms(1);
	MPU9250_AK8963_SPIx_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_FUSE_ROM_ACCESS);
	Delay_Ms(1);
	//
	//AK8963 get calibration data
	MPU9250_AK8963_SPIx_Reads(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_ASAX, 3, response);
	//AK8963_SENSITIVITY_SCALE_FACTOR
	//AK8963_ASA[i++] = (s16)((data - 128.0f) / 256.0f + 1.0f) ;
	MPU9250_AK8963_ASA[0] = (s16)(response[0]) + 128;
	MPU9250_AK8963_ASA[1] = (s16)(response[1]) + 128;
	MPU9250_AK8963_ASA[2] = (s16)(response[2]) + 128;
	Delay_Ms(1);
	MPU9250_AK8963_SPIx_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_POWER_DOWN);
	Delay_Ms(1);
	//
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_CTRL, 0x5D);
	Delay_Ms(1);
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV0_ADDR, MPU9250_AK8963_I2C_ADDR | MPU9250_I2C_READ);
	Delay_Ms(1);
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV0_REG, MPU9250_AK8963_ST1);
	Delay_Ms(1);
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV0_CTRL, 0x88);
	Delay_Ms(1);
	//
	MPU9250_AK8963_SPIx_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_CONTINUOUS_MEASUREMENT);
	Delay_Ms(1);

	//
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_CTRL, 0x09);
	Delay_Ms(1);
	//
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_DELAY_CTRL, 0x81);
	Delay_Ms(100);
}

int MPU9250_SPIx_Write(uint8_t addr, uint8_t reg_addr, uint8_t data){
	MPU9250_Dev_SELECT;
	_delay_us(1);
	MPU9250_SPIx_SendByte(reg_addr);
	_delay_us(1);
	MPU9250_SPIx_SendByte(data);
	_delay_us(1);
	MPU9250_Dev_DESELECT;
	return 0;
}

int MPU9250_SPIx_Writes(uint8_t addr, uint8_t reg_addr, uint8_t len, uint8_t* data){
	u32 i = 0;
	MPU9250_Dev_SELECT;
	_delay_us(1);
	MPU9250_SPIx_SendByte(reg_addr);
	while(i < len){
		_delay_us(1);
		MPU9250_SPIx_SendByte(data[i++]);
	}
	MPU9250_Dev_DESELECT;
	return 0;
}

uint8_t MPU9250_SPIx_Read(uint8_t addr, uint8_t reg_addr)
{
	uint8_t dummy = 0;
	uint8_t data = 0;

	MPU9250_Dev_SELECT;
	_delay_us(1);
	MPU9250_SPIx_SendByte(0x80 | reg_addr);
	_delay_us(1);
	data = MPU9250_SPIx_SendByte(dummy);
	MPU9250_Dev_DESELECT;
	return data;
}

int MPU9250_SPIx_Reads(uint8_t addr, uint8_t reg_addr, uint8_t len, uint8_t* data){
	u32 i = 0;
	uint8_t dummy = 0x00;

	MPU9250_Dev_SELECT;
	_delay_us(1);
	MPU9250_SPIx_SendByte(MPU9250_I2C_READ | reg_addr);
	while(i < len){
		_delay_us(1);
		data[i++] = MPU9250_SPIx_SendByte(dummy);
	}
	MPU9250_Dev_DESELECT;
	return 0;
}

int MPU9250_AK8963_SPIx_Read(uint8_t akm_addr, uint8_t reg_addr, uint8_t* data) {
	uint8_t status = 0;
	u32 timeout = 0;

	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_REG, 1, &reg_addr);
	Delay_Ms(1);
	reg_addr = akm_addr | MPU9250_I2C_READ;
	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_ADDR, 1, &reg_addr);
	Delay_Ms(1);
	reg_addr = MPU9250_I2C_SLV4_EN;
	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_CTRL, 1, &reg_addr);
	Delay_Ms(1);

	do {
		if (timeout++ > 50){
			return -2;
		}
		MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_STATUS, 1, &status);
		Delay_Ms(1);
	} while ((status & MPU9250_I2C_SLV4_DONE) == 0);
	MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_DI, 1, data);
	return 0;
}

int MPU9250_AK8963_SPIx_Reads(uint8_t akm_addr, uint8_t reg_addr, uint8_t len, uint8_t* data){
	uint8_t index = 0;
	uint8_t status = 0;
	u32 timeout = 0;
	uint8_t tmp = 0;

	tmp = akm_addr | MPU9250_I2C_READ;
	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_ADDR, 1, &tmp);
	Delay_Ms(1);
	while(index < len){
		tmp = reg_addr + index;
		MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_REG, 1, &tmp);
		Delay_Ms(1);
		tmp = MPU9250_I2C_SLV4_EN;
		MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_CTRL, 1, &tmp);
		Delay_Ms(1);

		do {
			if (timeout++ > 50){
				return -2;
			}
			MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_STATUS, 1, &status);
			Delay_Ms(2);
		} while ((status & MPU9250_I2C_SLV4_DONE) == 0);
		MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_DI, 1, data + index);
		Delay_Ms(1);
		index++;
	}
	return 0;
}

int MPU9250_AK8963_SPIx_Write(uint8_t akm_addr, uint8_t reg_addr, uint8_t data)
{
	u32 timeout = 0;
	uint8_t status = 0;
	uint8_t tmp = 0;

	tmp = akm_addr;
	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_ADDR, 1, &tmp);
	Delay_Ms(1);
	tmp = reg_addr;
	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_REG, 1, &tmp);
	Delay_Ms(1);
	tmp = data;
	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_DO, 1, &tmp);
	Delay_Ms(1);
	tmp = MPU9250_I2C_SLV4_EN;
	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_CTRL, 1, &tmp);
	Delay_Ms(1);

	do {
		if (timeout++ > 50)
			return -2;

		MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_STATUS, 1, &status);
		Delay_Ms(1);
	} while ((status & MPU9250_I2C_SLV4_DONE) == 0);
	if (status & MPU9250_I2C_SLV4_NACK)
		return -3;
	return 0;
}

int MPU9250_AK8963_SPIx_Writes(uint8_t akm_addr, uint8_t reg_addr, uint8_t len, uint8_t* data)
{
	u32 timeout = 0;
	uint8_t status = 0;
	uint8_t tmp = 0;
	uint8_t index = 0;

	tmp = akm_addr;
	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_ADDR, 1, &tmp);
	Delay_Ms(1);

	while(index < len){
		tmp = reg_addr + index;
		MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_REG, 1, &tmp);
		Delay_Ms(1);
		MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_DO, 1, data + index);
		Delay_Ms(1);
		tmp = MPU9250_I2C_SLV4_EN;
		MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_CTRL, 1, &tmp);
		Delay_Ms(1);

		do {
			if (timeout++ > 50)
				return -2;
			MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_STATUS, 1, &status);
			Delay_Ms(1);
		} while ((status & MPU9250_I2C_SLV4_DONE) == 0);
		if (status & MPU9250_I2C_SLV4_NACK)
			return -3;
		index++;
	}
	return 0;
}
//////////////////////////////////////////////////////////////////////////
//
void MPU9250_Get9AxisRawData(short *accel, short * gyro, short * mag)
{
	uint8_t data[22];
	MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_ACCEL_XOUT_H, 22, data);
	
	accel[0] = (data[0] << 8) | data[1];
	accel[1] = (data[2] << 8) | data[3];
	accel[2] = (data[4] << 8) | data[5];
	
	gyro[0] = (data[8] << 8) | data[9];
	gyro[1] = (data[10] << 8) | data[11];
	gyro[2] = (data[12] << 8) | data[13];

	if (!(data[14] & MPU9250_AK8963_DATA_READY) || (data[14] & MPU9250_AK8963_DATA_OVERRUN)){
		return;
	}
	if (data[21] & MPU9250_AK8963_OVERFLOW){
		return;
	}
	mag[0] = (data[16] << 8) | data[15];
	mag[1] = (data[18] << 8) | data[17];
	mag[2] = (data[20] << 8) | data[19];

	//ned x,y,z
	mag[0] = ((long)mag[0] * MPU9250_AK8963_ASA[0]) >> 8;
	mag[1] = ((long)mag[1] * MPU9250_AK8963_ASA[1]) >> 8;
	mag[2] = ((long)mag[2] * MPU9250_AK8963_ASA[2]) >> 8;
}
//////////////////////////////////////////////////////////////////////////
//
void MPU9250_Get6AxisRawData(short *accel, short * gyro)
{
	uint8_t data[14];
	MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_ACCEL_XOUT_H, 14, data);
	
	accel[0] = (short)((data[0] << 8) | data[1]);
	accel[1] = (short)((data[2] << 8) | data[3]);
	accel[2] = (short)((data[4] << 8) | data[5]);

	gyro[0] = (short)((data[8] << 8) | data[9]);
	gyro[1] = (short)((data[10] << 8) | data[11]);
	gyro[2] = (short)((data[12] << 8) | data[13]);
}
//////////////////////////////////////////////////////////////////////////
//
void MPU9250_Get3AxisAccelRawData(short * accel)
{
	uint8_t data[6];
	MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_ACCEL_XOUT_H, 6, data);

	accel[0] = (data[0] << 8) | data[1];
	accel[1] = (data[2] << 8) | data[3];
	accel[2] = (data[4] << 8) | data[5];
}
//////////////////////////////////////////////////////////////////////////
//
void MPU9250_Get3AxisGyroRawData(short * gyro)
{
	uint8_t data[6];
	MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_GYRO_XOUT_H, 6, data);

	gyro[0] = (data[0] << 8) | data[1];
	gyro[1] = (data[2] << 8) | data[3];
	gyro[2] = (data[4] << 8) | data[5];
}
//////////////////////////////////////////////////////////////////////////
//
void MPU9250_Get3AxisMagnetRawData(short *mag)
{
	uint8_t data[8];

	MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_EXT_SENS_DATA_00, 8, data);
	if (!(data[0] & MPU9250_AK8963_DATA_READY) || (data[0] & MPU9250_AK8963_DATA_OVERRUN)){
		return;
	}
	if (data[7] & MPU9250_AK8963_OVERFLOW){
		return;
	}
	mag[0] = (data[2] << 8) | data[1];
	mag[1] = (data[4] << 8) | data[3];
	mag[2] = (data[6] << 8) | data[5];

	mag[0] = ((long)mag[0] * MPU9250_AK8963_ASA[0]) >> 8;
	mag[1] = ((long)mag[1] * MPU9250_AK8963_ASA[1]) >> 8;
	mag[2] = ((long)mag[2] * MPU9250_AK8963_ASA[2]) >> 8;
}
//////////////////////////////////////////////////////////////////////////
//
void MPU9250_GetTemperatureRawData(long *temperature)
{
	uint8_t data[2];
	MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_TEMP_OUT_H, 2, data);
	temperature[0] = (((s16)data[0]) << 8) | data[1];
}

static uint8_t MPU9250_IsNewData = 0;

int MPU9250_IsDataReady(void)
{
	int isNewData = MPU9250_IsNewData;
	MPU9250_IsNewData = 0;
	return isNewData;
}

#define Init_angle1 1121
#define Init_angle2 1784
#define Init_angle3 1871
#define Init_angle4 1911
#define Init_angle5 1815
#define Init_angle6 1752
#define Init_angle7 1863
#define Init_angle8 1835
#define Init_angle9 1812
#define Init_angle10 1159
#define Init_angle11 1796
#define Init_angle12 1728
#define Init_angle13 1147
#define Init_angle14 1675
#define Init_angle15 1843
#define Init_angle16 1828
#define Init_angle17 1810


int16_t gyro[3],accel[3];
uint16_t Angle = Init_angle8;

void mpu9250_thread_entry(void* parameter)
{
	
	MPU9250_Init();
	//Init_MPU9250();
	//Delay_Ms(100);
	mlx90316_Init();
	while(1)
	{	
		MPU9250_Get6AxisRawData(accel,gyro);
		float Time_Offset = 0;
		float Local_Time = rt_tick_get()*0.001 + Time_Offset;
		if(Bias > 180){
			Bias = Bias - 256;
		}
		
		float Target_Angle = (0.1 * p_1+ 0.1 * p_2 * (16 - joint_id) / 16) * Amplitude * sin( 3.14 /180 * (Frequency * Local_Time + Phase*1.5) ) + Bias;
		
		//rt_kprintf("Amplitude : %d, Frequency : %d, Phase : %d, p_1 : %d, p_2 : %d, Bias : %d \n", Amplitude, Frequency, Phase, p_1, p_2, Bias );
		
		Target_Angle = Target_Angle * 10;
		Angle=(uint16_t)mlx90316_ReadAngle();
		Set_Joint_Angel((int)Target_Angle);
		
//		rt_kprintf("%d\n", Angle);
//		if(gyro[0]<minn0) minn0=gyro[0];
//		if(gyro[1]<minn1) minn1=gyro[1];
//		if(gyro[2]<minn2) minn2=gyro[2];
//		if(gyro[0]>maxn0) maxn0=gyro[0];
//		if(gyro[1]>maxn1) maxn1=gyro[1];
//		if(gyro[2]>maxn2) maxn2=gyro[2];
//		
//		//rt_kprintf("accel: %d %d %d\n",accel[0],accel[1],accel[2]);
//		rt_kprintf("gyro 0 : %d %d,",minn0,maxn0);
//		rt_kprintf("gyro 1 : %d %d,",minn1,maxn1);
//		rt_kprintf("gyro 2 : %d %d,",minn2,maxn2);
//		rt_kprintf("\n");
		
		rt_thread_delay(RT_TICK_PER_SECOND/100 );
//		rt_kprintf("accel:%d %d %d\t gyro:%d %d %d\n",accel[0],accel[1],accel[2],gyro[0],gyro[1],gyro[2]);
//		rt_kprintf("angle:%d \n",mlx90316_ReadAngle());
	}
}
void get_Gyro(uint8_t *cp_gyro)
{
	cp_gyro[0]=(((uint16_t)gyro[0])>>8);
	cp_gyro[1]=(((uint16_t)gyro[0])&0xFF);
	cp_gyro[2]=(((uint16_t)gyro[1])>>8);
	cp_gyro[3]=(((uint16_t)gyro[1])&0xFF);
	cp_gyro[4]=(((uint16_t)gyro[2])>>8);
	cp_gyro[5]=(((uint16_t)gyro[2])&0xFF);
	
	
	//rt_kprintf("gyro : %d %d %d\n",(int16_t)(cp_gyro[0]<<8|cp_gyro[1]),(int16_t)(cp_gyro[2]<<8|cp_gyro[3]),(int16_t)(cp_gyro[4]<<8|cp_gyro[5]));
	//memcpy(cp_gyro,gyro,3);
  //*cp_angel = mlx90316_ReadAngle();
}

void get_Accel(uint8_t *cp_accel)
{
	cp_accel[0]=(((uint16_t)accel[0])>>8);
	cp_accel[1]=(((uint16_t)accel[0])&0xFF);
	cp_accel[2]=(((uint16_t)accel[1])>>8);
	cp_accel[3]=(((uint16_t)accel[1])&0xFF);
	cp_accel[4]=(((uint16_t)accel[2])>>8);
	cp_accel[5]=(((uint16_t)accel[2])&0xFF);
	
	//rt_kprintf("accel: %d %d %d\n",(int16_t)(cp_accel[0]<<8|cp_accel[1]),(int16_t)(cp_accel[2]<<8|cp_accel[3]),(int16_t)(cp_accel[4]<<8|cp_accel[5]));
	
	
	cp_accel[6]=(Angle)>>8;
	cp_accel[7]=(Angle)&0xFF;
	//memcpy(cp_accel,gyro,3);
}

#endif

