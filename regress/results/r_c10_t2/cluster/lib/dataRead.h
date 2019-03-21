#ifndef DATAREAD_H
#define DATAREAD_H
#include "definition.h"

struct LDatas {
	int jointNum;
	double *joints;
	double accelX, accelY, accelZ;
	double jointDiff;
	double ampli, phase, speed, offset;
};

int getOpData(string fileName);
#endif // !DATAREAD_H
