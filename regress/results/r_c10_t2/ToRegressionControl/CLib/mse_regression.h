#ifndef MSE_REGRESSION_H
#define MSE_REGRESSION_H

#include "definition.h"


struct DisData{
    int pos;
    double dis;
};

int getAim();
//void knnRegression(LDatas *tp);
void getFitting();
void getRegression(LDatas *tp);

#endif // !MSE_REGRESSION_H
