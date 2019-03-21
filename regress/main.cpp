#include <iostream>
#include <cstring>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdexcept>
#include <fstream>
#include <cmath>
using namespace std;
#include "op.h"
#include "extApi.h"

#include "dataRead.h"
#include "preWork.h"
#include "mse_regression.h"

/***********************************/
extern LDatas trainData[MAX_DATA];
extern vector<int> pointsArray;
extern int reg;
extern double paraControl[4];
/***********************************/

int clientID;
double joints[moduleNum];
double *outAngle;

LDatas thePoint;

int main(int argc,char* argv[])
{
    clock_t timeStart, timeEnd;
    getClusterData("originalData/cluster.txt");
    getOpData("originalData/data.txt");
    quantization();
    thePoint.jointNum = moduleNum - 1;
    thePoint.joints = new double[moduleNum - 1];

    ofstream outF("data_n.txt");
    outAngle = new double[moduleNum];
    for (int i = 0; i < moduleNum; i++){
        joints[i] = -100;
        outAngle[i] = 0;
    }
    string tESend = dArray2String(joints, moduleNum-1);
    unsigned char *EsendJoints;
    EsendJoints = (unsigned char*)tESend.c_str();

    printf("----------- Program Start -----------\n");
    try{
        clientID=simxStart((simxChar*)"127.0.0.1",19997,1,1,2000,5);
        if(clientID > -1){
            printf("Connect to Remote API server!\n");
            simxInt res = 0;
            simxFloat startTime, nowTime, theTime;
            simxUChar *transString;
            
            double ampli, speed, phase, offset;
            double *accelX, *accelY, *accelZ;
            double *nowJoints;
            double diffJoints;
            transString = new simxUChar[255];
            accelX = NULL;
            accelY = NULL;
            accelZ = NULL;
            nowJoints = NULL;
            /***********************************************/
            simxStartSimulation(clientID, simx_opmode_oneshot);
            timeDelay(5);
            simxSetStringSignal(clientID, "JointsData", EsendJoints,
                tESend.length(),simx_opmode_oneshot);
            res = simxGetFloatSignal(clientID,"timeSignal", 
                &startTime,simx_opmode_streaming);
            simxStopSimulation(clientID, simx_opmode_oneshot);
            timeDelay(3);
            /***********************************************/
            double tAmpli, tPhase, tSpeed, tOffset;
            tAmpli = 0;
            tPhase = 0;
            tSpeed = 0;
            tOffset = 0;
            theTime = 0;
            printf("A-%lf P-%lf S-%lf Offset-%lf\n",tAmpli, tPhase, tSpeed, tOffset);
            simxStartSimulation(clientID, simx_opmode_oneshot);
            timeDelay(5);
            printf("Simulation is started!\n");
            simxSetStringSignal(clientID, "JointsData", EsendJoints,tESend.length(),simx_opmode_oneshot);
            res = simxGetFloatSignal(clientID,"timeSignal", &startTime,simx_opmode_streaming);
            nowTime = startTime;
            simxClearFloatSignal(clientID,"", simx_opmode_streaming);
            /*******************************************************/
            double nTimes = 0.0;
            double outAccelX, outAccelY, outAccelZ, outDiff;
            outAccelX = outAccelY = outAccelZ = outDiff = 0.0;
            for (int i = 0; i < moduleNum; i++){
                outAngle[i] = 0.0;
            }
            /*******************************************************/
            int oldTimes = 0;
            while (theTime <= 200){
                string taX,taY,taZ,tJointString;
                int strLen;
                res = simxGetStringSignal(clientID,"A_X", 
                    &transString,&strLen,simx_opmode_streaming);
                taX = (char*)transString;
                res = simxGetStringSignal(clientID,"A_Y", 
                    &transString,&strLen,simx_opmode_streaming);
                taY = (char*)transString;
                res = simxGetStringSignal(clientID,"A_Z", 
                    &transString,&strLen,simx_opmode_streaming);
                taZ = (char*)transString;
                res = simxGetStringSignal(clientID,"T_J",
                    &transString,&strLen,simx_opmode_streaming);
                tJointString = (char*)transString;
                    
                string2array(taX, moduleNum, &accelX);
                string2array(taY, moduleNum, &accelY);
                string2array(taZ, moduleNum, &accelZ);
                string2array(tJointString, moduleNum-1,&nowJoints);
                /**********************************************/                    
                double nAccelZ = 0.0;
                double nAccelY = 0.0;
                double nAccelX = 0.0;
                for(int i = 0; i < moduleNum; i++){
                    nAccelZ += accelZ[i];
                    nAccelY += accelY[i];
                    nAccelX += accelX[i];
                }
                nAccelX /= (double)moduleNum;
                nAccelY /= (double)moduleNum;
                nAccelZ /= (double)moduleNum;
                diffJoints = 0.0;
                for (int i = 0; i < moduleNum-1; i++){
                    diffJoints += abs(nowJoints[i] - outAngle[i]);
                    //cout << abs(nowJoints[i]-outAngle[i])<<" >> ";
                }
                /**********************************************/                    
                if(theTime < 5){
                    ampli = 30;
                    phase = 0;
                    speed = 3;
                    offset = 0;
                }else if(theTime < 10){
                    ampli = 40;
                    phase = 0;
                    speed = 2;
                    offset = 0; 
                    cout << "------ Wait ------" << endl;
                    tAmpli = ampli;
                    tPhase = phase;
                    tSpeed = speed;
                    tOffset = offset;
                }else {
                    //timeStart = clock();
                    //timeEnd = clock();
                    //cout << (double)(timeEnd-timeStart)/CLOCKS_PER_SEC << endl;
                    /**********************************************/
                    //ampli = tAmpli;
                    //phase = tPhase;
                    //speed = tSpeed;
                    //offset = tOffset;
                    //cout << ampli << " " << phase << " ";
                    //cout << speed << " " << offset << endl;
                    outAccelX += nAccelX;
                    outAccelY += nAccelY;
                    outAccelZ += nAccelZ;
                    outDiff += diffJoints;
                    nTimes += 1.0;
                    int tTime = theTime;
                    if(tTime % 2 == 0 && oldTimes != tTime){
                        timeStart = clock();
                        oldTimes = tTime;
                        //cout << tTime<< " "<<tTime%5 <<">>>>";
                        outAccelX /= nTimes;
                        outAccelY /= nTimes;
                        outAccelZ /= nTimes;
                        outDiff /= nTimes;
                        for(int j = 0; j < moduleNum-1; j++){
                            outF << nowJoints[j] << " ";
                        }
                        cout << endl;
                        outF << outAccelX << " ";
                        outF << outAccelY << " ";
                        outF << outAccelZ << " ";
                        outF << outDiff << " ";
                        outF << ampli << " " << phase << " ";
                        outF << speed << " " << offset << endl;
                        nTimes =0; 

                        for(int i = 0; i < moduleNum - 1; i++){
                            thePoint.joints[i] = nowJoints[i];
                        }
                        thePoint.accelX = outAccelX;
                        thePoint.accelY = outAccelY;
                        thePoint.accelZ = outAccelZ;
                        thePoint.jointDiff = outDiff/(moduleNum-1);
                        thePoint.ampli = ampli;
                        thePoint.phase = phase;
                        thePoint.speed = speed;
                        thePoint.offset = offset;
                   	cout << "AccelX: " << outAccelX << " ";
					cout << "AccelY: " << outAccelY << " ";
					cout << "AccelZ: " << outAccelZ << endl;
					if(outAccelX >=1 || outAccelY >=1) continue;
					if(outAccelZ <= -1) {
						tSpeed = 2.0;
						thePoint.accelZ = -1*outAccelZ;			
					}
                        getBetterPoint(&thePoint);
                        //cout << 1 << endl;
                        getH(pointsArray);
                        //cout << 2 << endl;
                        getVariance();
                        //cout << 3 << endl;
                        getDistribution();
                        //cout << 4 << endl;

                        reg = getAim();
                        if(pointsArray.size() > moduleNum){
                            getFitting();
                            getRegression(&thePoint);
                            tAmpli = paraControl[0];
                            tPhase = paraControl[1];
                            tSpeed = paraControl[2];
                            tOffset = paraControl[3];
                        }else{
                            int tpLen = pointsArray.size();
                            for(int i = 0; i < tpLen; i++){
                                tAmpli += ampli;
                                tPhase += phase;
                                tSpeed += speed;
                                tOffset += offset;
                                switch(reg){
                                    case 0:
                                        tAmpli += (trainData[i].ampli - ampli);
                                        break;
                                    case 1:
                                        tPhase += (trainData[i].phase - phase);
                                        break;
                                    case 2:
                                        tSpeed += (trainData[i].speed - speed);
                                        break;
                                    case 3:
                                        tOffset += (trainData[i].offset - offset);
                                        break;
                                    default: break;
                                }
                            }
                            if(tpLen == 0){
                                tAmpli = ampli;
                                tPhase = phase;
                                tSpeed = speed;
                                tOffset = offset;
                            }else{
                                tAmpli /= tpLen;
                                tPhase /= tpLen;
                                tSpeed /= tpLen;
                                tOffset /= tpLen;
                            }
                            if(tSpeed > 3.0) tSpeed = 3.0;
                        }
                        timeEnd = clock();
                        cout << (double)(timeEnd-timeStart)/CLOCKS_PER_SEC << endl;
                    }
                    ampli = tAmpli;
                    phase = tPhase;
                    speed = tSpeed;
                    offset = 0;
                }
                /**********************************************/
                string tSend = getRolling(ampli,phase,speed,offset,theTime,joints,&outAngle);
                unsigned char *sendJoints;
                sendJoints = (unsigned char*)tSend.c_str();
                //cout << sendJoints << endl;
                simxSetStringSignal(clientID, "JointsData", sendJoints,
                    tSend.length(),simx_opmode_oneshot);
                /**********************************************/                    
                res = simxGetFloatSignal(clientID,"timeSignal", 
                    &nowTime,simx_opmode_streaming);
                theTime = nowTime - startTime;
            }
                                
            simxStopSimulation(clientID, simx_opmode_oneshot);
            timeDelay(3);

            transString = NULL;
            delete transString;
        }else{
            throw "Connect Failed";
        }
    }catch(const char* &err){
        printf("-------------------------------------\n");
        if(err != NULL){
           printf("%s\n", err);
        }else{
            printf("Unkown Error\n");
        } 
    }
    simxFinish(clientID);
    outF.close();
    printf("Program Ended\n");
    return(0);
}
