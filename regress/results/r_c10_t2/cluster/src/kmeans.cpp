#include "kmeans.h"
#include "dataRead.h"
#include <cmath>

/**********************************************/
//From "dataRead"
extern LDatas trainData[MAX_DATA];
/**********************************************/
vector<double> dataPoint[POINT_NUM];
vector<double> centerPoint[CENTER_NUM];
vector<int> clusterResult[CENTER_NUM];
double clusterPoints[CENTER_NUM];
double clusterNum[CENTER_NUM];

int pointIndex[POINT_NUM];
int centerCheck[POINT_NUM];
int pointSize = 0;
/**********************************************/

/******************************************************************/
void dataInit(string fileName) {
	for (int i = 0; i < POINT_NUM; i++) {
		pointIndex[i] = -1;
		centerCheck[i] = 0;
	}
	pointSize = getOpData(fileName);
	int tJointsNum = JOINT_NUM;
	for (int i = 0; i < pointSize; i++) {
		dataPoint[POINT_NUM].clear();
		for (int j = 0; j < JOINT_NUM; j++) {
			dataPoint[i].push_back(trainData[i].joints[j]);
		}
		dataPoint[i].push_back(trainData[i].jointDiff);
	
	}
	/*for (int i = 0; i < pointSize; i++) {
		int dNum = dataPoint[i].size();
		for (int j = 0; j < dNum; j++) {
			cout << dataPoint[i][j] << " ";
		}
		cout << endl;
	}*/
}
void centerInit() {
	int cPoint = random(pointSize);
	centerPoint[0] = dataPoint[cPoint];
	centerCheck[cPoint] = 1;
	for (int i = 1; i < CENTER_NUM; i++) {
		double maxDis = 0;
		int newCN = -1;
		for (int j = 0; j < pointSize; j++) {
			double tDis = 0.0;
			for (int k = 0; k < i; k++) {
				tDis += getVectorDis(centerPoint[k], dataPoint[j]);
			}
			if (tDis > maxDis) {
				if(centerCheck[j] != 1)				newCN = j;
				maxDis = tDis;
			}
		}
		centerPoint[i] = dataPoint[newCN];
	}

	/*for (int i = 0; i < CENTER_NUM; i++) {
		int endLen = centerPoint[i].size();
		cout << i << " " << endLen << " ------- " << endl;
		for (int j = 0; j < endLen; j++) {
			cout << centerPoint[i][j] << " ";
		}
		cout << endl;
	}*/
}
/******************************************************************/
void kmeans() {
	bool clusterChanged = true;
	int stepNum = 0;
	while (clusterChanged) {
		stepNum++;
		clusterChanged = false;
		for (int i = 0; i < pointSize; i++) {
			int theIndex = -1;
			double minDis = INIT_MAX;

			for (int j = 0; j < CENTER_NUM; j++) {
				double nowDis = getVectorDis(dataPoint[i], centerPoint[j]);
				if (nowDis < minDis) {
					minDis = nowDis;
					theIndex = j;
				}
			}
			if (theIndex != pointIndex[i]) {
				clusterChanged = true;
				pointIndex[i] = theIndex;
			}
		}
		updateCenter();
	}
	int total = 0;
	for (int i = 0; i < CENTER_NUM; i++) {
		total += clusterPoints[i];
		int endLen = centerPoint[i].size();
		cout << i << " " << endLen << " " << clusterPoints[i] << " ------- " << endl;
		for (int j = 0; j < endLen; j++) {
			cout << centerPoint[i][j] << " ";
		}
		cout << endl;
	}
	cout << "Step Number: " << stepNum << endl;
	cout << total << " " << pointSize << endl;
}
/******************************************************************/
void updateCenter() {
	int vectorSize = dataPoint[0].size();
	int nowNum = 0;
	for (int i = 0; i < CENTER_NUM; i++) {
		clusterPoints[i] = 0.0;
		for (int j = 0; j < vectorSize; j++) {
			centerPoint[i][j] = 0.0;
		}
	}
	for (int i = 0; i < pointSize; i++) {
		int nowIndex = pointIndex[i];
		clusterPoints[nowIndex] += 1.0;
		for (int j = 0; j < vectorSize; j++) {
			centerPoint[nowIndex][j] += dataPoint[i][j];
		}
	}
	for (int i = 0; i < CENTER_NUM; i++) {
		for (int j = 0; j < vectorSize; j++) {
			centerPoint[i][j] /= clusterPoints[i];
		}
	}
	/******************************************************/
}

/***************************************************************/
void resultOut() {
	for (int i = 0; i < CENTER_NUM; i++) {
		clusterResult[i].clear();
	}

	for (int i = 0; i < pointSize; i++) {
		int nowIndex = pointIndex[i];
		clusterResult[nowIndex].push_back(i);
	}

	cout << "-------------------------" << endl;
	for (int i = 0; i < CENTER_NUM; i++) {
		cout << clusterResult[i].size() << " ";
	}
	cout << endl;

	ofstream fOut("cluster.txt");
	int vectorSize = centerPoint[0].size();

	for (int i = 0; i < CENTER_NUM; i++) {
		for (int j = 0; j < vectorSize; j++) {
			fOut << centerPoint[i][j] << " ";
		}
		fOut << endl;
		int cLen = clusterResult[i].size();
		for (int j = 0; j < cLen; j++) {
			fOut << clusterResult[i][j] << " ";
		}
		fOut << endl;
	}
	fOut.close();
	
}
