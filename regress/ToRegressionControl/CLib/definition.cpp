#include "definition.h"

int random(int x) {
	srand((int)time(0));
	return rand() % x;
}
double getVectorDis(vector<double> v1, vector<double> v2) {
	int len = v1.size();
	//cout << v1.size() << " " << v2.size() << endl;
    double dis = 0.0;
	for (int i = 0; i < len; i++) {
		dis += (v1[i] - v2[i])*(v1[i] - v2[i]);
	}
	dis = sqrt(dis);

	return dis;
}