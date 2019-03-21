#pragma once

#include <iostream>
#include <sstream>
#include <fstream>
#include <cstdlib>
#include <string>
#include <vector>
#include <cmath>
#include <ctime>

using namespace std;

const double INIT_MAX = 1e9;
const int POINT_NUM = 100000;
const int CENTER_NUM = 24;
const int CLUSTERING_NUM = 1000;

const int MAX_DATA = 100000;
const int DATA_NUM = 30;
const int JOINT_NUM = 16;

int random(int x);
double getVectorDis(vector<double> v1, vector<double> v2);
