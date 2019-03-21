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

int clientID;
double joints[moduleNum];
double *outAngle;

int main(int argc,char* argv[])
{
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
            for (double tAmpli = 40; tAmpli <= 80; tAmpli += 5){
                for (double tPhase = 0; tPhase <= 5; tPhase += 1){
                    for (double tSpeed = 1; tSpeed <=5; tSpeed += 1){
                        double offsetEnd = 90 - tAmpli;
                        //double offsetEnd = 0;
                        for (double tOffset = 0; tOffset <= offsetEnd; tOffset += 10){
                            for (int it = 0; it < 1; it++){
                                theTime = 0;
                                printf("A-%lf P-%lf S-%lf Offset-%lf\n", 
                                        tAmpli, tPhase, tSpeed, tOffset);
                                simxStartSimulation(clientID, simx_opmode_oneshot);
                                timeDelay(5);
                                printf("Simulation is started!\n");
                                simxSetStringSignal(clientID, "JointsData", EsendJoints,
                                    tESend.length(),simx_opmode_oneshot);
                                res = simxGetFloatSignal(clientID,"timeSignal", 
                                    &startTime,simx_opmode_streaming);
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
                                while (theTime <= 40){
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
                                    }else if(theTime <= 20){
                                        ampli = 40 + (tAmpli-40)/10;
                                        phase = 0;
                                        speed = 2;
                                        offset = 0;
                                    }else {
                                        ampli = tAmpli;
                                        phase = tPhase;
                                        speed = tSpeed;
                                        offset = 0;//tOffset;
                                        
                                        outAccelX += nAccelX;
                                        outAccelY += nAccelY;
                                        outAccelZ += nAccelZ;
                                        outDiff += diffJoints;
                                        nTimes += 1.0;
                                        int tTime = theTime;
                                        if(tTime % 2 == 0 && oldTimes != tTime){
                                            oldTimes = tTime;
                                            //cout << tTime<< " "<<tTime%5 <<">>>>";
                                            outAccelX /= nTimes;
                                            outAccelY /= nTimes;
                                            outAccelZ /= nTimes;
                                            outDiff /= nTimes;
                                            for(int j = 0; j < moduleNum-1; j++){
                                                outF << nowJoints[j] << " ";
                                            }
                                            outF << outAccelX << " ";
                                            outF << outAccelY << " ";
                                            outF << outAccelZ << " ";
                                            outF << outDiff << " ";
                                            outF << ampli << " " << phase << " ";
                                            outF << speed << " " << offset << endl;
                                            nTimes =0; 
                                        }
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
                                printf("Times: %d is ended! ---------- \n", it);
                                timeDelay(3);
                            }
                        }
                    }
                }
            }
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
