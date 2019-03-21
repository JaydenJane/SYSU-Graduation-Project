#include "dataRead.h"

LDatas trainData[MAX_DATA];
double readData[DATA_NUM];

int getOpData(string fileName) {
	const char *theFile = fileName.c_str();
	ifstream fIn(theFile);
	string line;
	bool first = true;
	int dCount = 0;

	while (getline(fIn, line)) {
		string nums;
		stringstream ss(line);
		int dPos = 0;
		while (getline(ss, nums, ' ')) {
			stringstream s2n(nums);
			double tData;
			s2n >> tData;
			readData[dPos++] = tData;
		}
		trainData[dCount].jointNum = JOINT_NUM;
		trainData[dCount].joints = new double[JOINT_NUM];
		for (int i = 0; i < JOINT_NUM; i++) {
			trainData[dCount].joints[i] = readData[i];
		}
		int nDataPos = JOINT_NUM;
		trainData[dCount].accelX = readData[nDataPos++];
		trainData[dCount].accelY = readData[nDataPos++];
		trainData[dCount].accelZ = readData[nDataPos++];
		trainData[dCount].jointDiff = readData[nDataPos++];
		trainData[dCount].ampli = readData[nDataPos++];
		trainData[dCount].phase = readData[nDataPos++];
		trainData[dCount].speed = readData[nDataPos++];
		trainData[dCount].offset = readData[nDataPos++];
		//Reduce the gravity of the high proportion variable
		trainData[dCount].jointDiff /= JOINT_NUM;

		dCount++;
	}
	fIn.close();
	/*printf("%d\n", dCount);

	for (int it = 0; it < dCount; it++) {
		for (int i = 0; i < JOINT_NUM; i++) {
			cout << trainData[it].joints[i] << " ";
		}
		cout << trainData[it].accelX << " ";
		cout << trainData[it].accelY << " ";
		cout << trainData[it].accelZ << " ";
		cout << trainData[it].jointDiff << " ";
		cout << trainData[it].ampli << " ";
		cout << trainData[it].phase << " ";
		cout << trainData[it].speed << " ";
		cout << trainData[it].offset << endl;
	}*/

	return dCount;
}