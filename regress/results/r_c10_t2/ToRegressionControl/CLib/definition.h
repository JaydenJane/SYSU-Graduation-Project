#pragma once

#include <iostream>
#include <sstream>
#include <fstream>
#include <cstdlib>
#include <string>
#include <vector>
#include <ctime>
#include <cmath>
#include <map>

using namespace std;

const double INIT_MAX = 1e9;
const int POINT_NUM = 100000;
const int CENTER_NUM = 10;
const int CLUSTERING_NUM = 1000;

const double STEP_LEN = 0.1;
const int MAX_DATA = 100000;
const int DATA_NUM = 30;
const int JOINT_NUM = 16;

const double STEP_RATE = 0.5;
const double NEAR_ZERO = 1e-6;
const int MAX_ITER = 1e3;
struct LDatas {
	int jointNum;
	double *joints;
	double accelX, accelY, accelZ;
	double jointDiff;
	double ampli, phase, speed, offset;
};

int random(int x);
double getVectorDis(vector<double> v1, vector<double> v2);
