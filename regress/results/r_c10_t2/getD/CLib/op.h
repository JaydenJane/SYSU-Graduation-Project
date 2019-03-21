#ifndef OP_H
#define OP_H

#include <iostream>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <sstream>
#include <math.h>
using namespace std;

#define PI acos(-1)


const int moduleNum = 17;

/*Delay function
Unit: Second */
void timeDelay(double theSecond);
/*Unpack Double Function
 divide with space char ' '*/
void string2array(string ts, int sizeA, double **rData);
/*Pack Double Functioni
 divide with char ','*/
string dArray2String(double *dArray, int len);


/************* Gaits *************/
/*Return value:
 the packed string of joints control data*/
string getRolling(double ampli, double phase, double speed,
        double offset, double nTime, double *joints, double **outAngle);
#endif
