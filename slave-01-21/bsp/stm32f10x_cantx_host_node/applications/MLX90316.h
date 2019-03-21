#ifndef __MLX90316_H
#define __MLX90316_H
#include <stm32f10x.h>

void mlx90316_Init(void);
int mlx90316_ReadAngle(void);
#endif
