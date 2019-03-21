#include "dataRead.h"

vector<double> clusterCenter[CENTER_NUM];
vector<double> centerChild[CENTER_NUM];

map<int, int> tStore;
LDatas trainData[MAX_DATA];
double trainParameter[MAX_DATA][4];
double readData[DATA_NUM];
vector<int> qResult;

int qAttribute[4][MAX_DATA];
int qTotal[4];

int dCount = 0;
int resultNum = 0;

void getClusterData(string fileName) {
	const char *theFile = fileName.c_str();
	ifstream fIn(theFile);
	string line;
	int lineFlag = 0;
	int nowPos = 0;
	while (getline(fIn, line)) {
		string nums;
		stringstream ss(line);
		lineFlag++;
		//cout << lineFlag << " " << nowPos << endl;
		//cout << line << endl;
        while (ss >> nums) {
			stringstream s2n(nums);
			double tData;
			s2n >> tData;
			//cout << nums << " ";
            if (lineFlag % 2 == 1)	clusterCenter[nowPos].push_back(tData);
			else					centerChild[nowPos].push_back(tData);
		}
		if (lineFlag % 2 == 0)	nowPos++;
        //cout << endl;
	}
	fIn.close();
	cout << "Read End---------------------" << endl;
	/*cout << nowPos << endl;
    for (int i = 0; i < nowpos; i++) {
		int cL = clusterCenter[i].size();
		for (int j = 0; j < cL; j++) {
			cout << clusterCenter[i][j] << " ";
		}
		cout << endl;
		cL = centerChild[i].size();
		for (int j = 0; j < cL; j++) {
			cout << centerChild[i][j] << " ";
		}
		cout << endl;
		cout << "-----------------------------------------" << endl;
	}*/
}
void getOpData(string fileName) {
	const char *theFile = fileName.c_str();
	ifstream fIn(theFile);
	string line;
	dCount = 0;
	for (int i = 0; i < MAX_DATA; i++) {
		trainParameter[i][0] = 0.0;
		trainParameter[i][1] = 0.0;
		trainParameter[i][2] = 0.0;
		trainParameter[i][3] = 0.0;
	}
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

		trainParameter[dCount][0] = trainData[dCount].ampli;
		trainParameter[dCount][1] = trainData[dCount].phase;
		trainParameter[dCount][2] = trainData[dCount].speed;
		trainParameter[dCount][3] = trainData[dCount].offset;

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
}
void quantization() {
	tStore.clear();
	resultNum = 0;
	for (int i = 0; i < dCount; i++) {
		double trainResult = trainData[i].accelZ;
		if (trainResult < 0)		trainResult -= STEP_LEN;
		trainResult /= STEP_LEN;

		int resultIndex = trainResult;
		if (!tStore[resultIndex]) {
			resultNum++;
			tStore[resultIndex] = resultNum;
		}
		qResult.push_back(tStore[resultIndex]);
	}
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < MAX_DATA; j++) {
			qAttribute[i][j] = -1;
		}
		tStore.clear();
		qTotal[i] = 0;
		for (int j = 0; j < dCount; j++) {
			int trainAttribute = 0;
			switch (i)
			{
			case 0: trainAttribute = trainData[j].ampli; break;
			case 1: trainAttribute = trainData[j].phase; break;
			case 2: trainAttribute = trainData[j].speed; break;
			case 3: trainAttribute = trainData[j].offset; break;
			default:
				cout << "Something error!" << endl;
				break;
			}
			if (!tStore[trainAttribute]) {
				qTotal[i]++;
				tStore[trainAttribute] = qTotal[i];
			}
			qAttribute[i][j] = tStore[trainAttribute];
		}
	}
}
