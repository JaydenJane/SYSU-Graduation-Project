#include "dataRead.h"
#include "preWork.h"

/**********************************************/
//Extern From Data Read
extern vector<double> clusterCenter[CENTER_NUM];
extern vector<double> centerChild[CENTER_NUM];

extern vector<int> qResult;
extern LDatas trainData[MAX_DATA];
extern int qAttribute[4][MAX_DATA];
extern int dCount;
extern int resultNum;

/**********************************************/
vector<double> np;
vector<int> pointsArray;
vector<int> tQuantity;
vector<double> entropy[4];
double deviation[4];

void getBetterPoint(LDatas *tp) {
	int theCluster = -1;
	double minDis = INIT_MAX;
	np.clear();
	for (int j = 0; j < JOINT_NUM; j++) {
		np.push_back(tp->joints[j]);
	}
	np.push_back(tp->jointDiff);
	for (int i = 0; i < CENTER_NUM; i++) {
		double theDis = getVectorDis(np, clusterCenter[i]);
		if (minDis > theDis) {
			minDis = theDis;
			theCluster = i;
		}
	}
    //cout << theCluster << endl;
	int cL = centerChild[theCluster].size();
    pointsArray.clear();
	for (int i = 0; i < cL; i++) {
		int thePos = centerChild[theCluster][i];
		if (trainData[thePos].accelZ > tp->accelZ){
			pointsArray.push_back(thePos);
            //cout << thePos << " ";
        }
	}
    /*cout << endl << endl;
    for(int i = 0; i < pointsArray.size(); i++){
        cout << pointsArray[i] << " ";
    }*/
}

void getH(vector<int> pArray) {
	/*cout << 1 <<endl;
    for(int i = 0 ; i < pArray.size(); i++){
        cout << pArray[i] << " ";
    }*/
    int pLen = pArray.size();
	for (int i = 0; i < 4; i++) {
		int aMax = 0;
		for (int j = 0; j < pLen; j++) {
			int nowPoint = pArray[j];
			if (qAttribute[i][nowPoint]>aMax)	aMax = qAttribute[i][nowPoint];
		}
		entropy[i].clear();
		for (int j = 1; j <= aMax; j++) {
			tQuantity.clear();
			for (int k = 0; k < pLen; k++) {
				int nowPoint = pArray[k];
                //cout << k << endl;
				if (qAttribute[i][nowPoint] == j)	tQuantity.push_back(nowPoint);
			}
			int qLen = tQuantity.size();
			double proH = 0.0;
            for (int rn = 1; rn <= resultNum; rn++) {
				int nowAttribute = 0;
				for (int qn = 0; qn < qLen; qn++) {
					if (qResult[tQuantity[qn]] == rn) {
						nowAttribute++;
					}
				}
				if (nowAttribute == 0)	continue;
				double nowRate = (double)nowAttribute / (double)qLen;
				proH += (-nowRate)*log(nowRate) / log(2.0);
			}
			entropy[i].push_back(proH);
		}
	}
	/*for (int i = 0; i < 4; i++) {
	    int tL = entropy[i].size();
	    for (int j = 0; j < tL; j++) {
	        cout << entropy[i][j] << " ";
	    }
	    cout << endl;
	}*/
}

void getVariance() {
	for (int i = 0; i < 4; i++) {
		int tL = entropy[i].size();
		double expect = 0.0;
		for (int j = 0; j < tL; j++) {
			expect += entropy[i][j];
		}
		expect /= tL;
		deviation[i] = 0.0;
		for (int j = 0; j < tL; j++) {
			double var = entropy[i][j] - expect;
			deviation[i] += (var*var);
		}
		deviation[i] /= tL;
		//cout << deviation[i] << endl;
	}
}

void getDistribution() {
	double dTotoal = 0.0;
	for (int i = 0; i < 4; i++) {
		dTotoal += deviation[i];
	}

	for (int i = 0; i < 4; i++) {
		//double theRate = deviation[i] / dTotoal;
		//cout << theRate << endl;
		deviation[i] /= dTotoal;
	}
}
