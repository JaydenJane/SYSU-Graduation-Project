#include "mse_regression.h"
#include "dataRead.h"
#include "preWork.h"

/***********************************************/
//From Date Read
extern LDatas trainData[MAX_DATA];
extern double trainParameter[MAX_DATA][4];
/***********************************************/
//Extern From Prework
extern vector<int> pointsArray;
extern double deviation[4];
/***********************************************/

vector<double> coefficient;
vector<double> tCoefficient;
vector<double> gradStep;

int reg = 0;
double paraControl[4];
DisData theVector[MAX_DATA];

int getAim() {
	int rAccuracy = 10000;
	int rNum = random(rAccuracy);
	double rRate = (double)rNum / (double)rAccuracy;
	int aimP = 0;
	double nowDivide = 0.0;
	for (int i = 0; i < 3; i++) {
		//cout << deviation[i] << " ";
        nowDivide += deviation[i];
		if (rRate < nowDivide)	break;
		aimP++;
	}
    //cout <<" --- " << rRate << endl;
	return aimP;
}
void getFitting() {
	//reg = getAim();
    int tLen = JOINT_NUM + 1;
	coefficient.clear();
	tCoefficient.clear();
	gradStep.clear();
	for (int i = 0; i < tLen+1; i++) {
		coefficient.push_back(0.0);
		tCoefficient.push_back(0.0);
		gradStep.push_back(0.0);
	}
	bool cFlag = true;
	int aL = pointsArray.size();
	int iterNum = 0;
    //cout << aL << endl;
	while (cFlag && iterNum < MAX_ITER)
	{
        for(int i = 0; i < tLen+1; i++){
            gradStep[i] = 0.0;
        }
		//cout << aL << endl;
        for (int i = 0; i < aL; i++) {
			double fValue = 0.0;
			fValue += coefficient[0];
            int nPoint = pointsArray[i];
			double sRate = trainData[nPoint].accelZ / 40.0;
            //cout << JOINT_NUM << endl;
            for (int j = 1; j <= JOINT_NUM; j++) {
				//cout << nPoint << endl;
                //cout << trainData[nPoint].joints[j-1] << endl;
                fValue += (coefficient[j] * trainData[nPoint].joints[j-1]);
			}
			//cout << fValue << endl;
            fValue += coefficient[JOINT_NUM+1] * trainData[nPoint].jointDiff;
            //cout << fValue << endl;
            double fDiff = (fValue - trainParameter[nPoint][reg])*sRate;
			gradStep[0] += fDiff;
			for (int j = 1; j <= JOINT_NUM; j++) {
				gradStep[j] += (fDiff * trainData[nPoint].joints[j-1]);
			}
			gradStep[JOINT_NUM + 1] += (fDiff * trainData[nPoint].jointDiff);
		    
        }
        //cout << aL << endl;
		//gradStep *= (STEP_RATE / aL);
		tCoefficient[0] = coefficient[0] - gradStep[0] / aL;
		for (int i = 1; i <= JOINT_NUM; i++) {
			tCoefficient[i] = coefficient[i] - gradStep[i] / aL;
		}
		tCoefficient[JOINT_NUM + 1] = coefficient[JOINT_NUM + 1] - gradStep[JOINT_NUM + 1] / aL;

		double vcDis = getVectorDis(coefficient, tCoefficient);
		//cout << "-------> " << vcDis << endl;
        if (vcDis < NEAR_ZERO)	cFlag = false;
		else					coefficient.assign(tCoefficient.begin(), tCoefficient.end());
		iterNum++;
	}
}

void getRegression(LDatas *tp) {
	double aimValue = 0.0;
	aimValue += coefficient[0];
	for (int i = 1; i <= JOINT_NUM; i++) {
		aimValue += (tp->joints[i - 1] * coefficient[i]);
	}
	aimValue += (tp->jointDiff*coefficient[JOINT_NUM + 1]);

	paraControl[0] = tp->ampli;
	paraControl[1] = tp->phase;
    paraControl[2] = tp->speed;
	paraControl[3] = tp->offset;
    for(int i = 0; i < 3; i++){
        cout << paraControl[i] << " ";
    }
    cout << "--- " << reg << endl;
	paraControl[reg] = aimValue;

    if(paraControl[2] > 3)
        paraControl[2] = 3.0;
}
