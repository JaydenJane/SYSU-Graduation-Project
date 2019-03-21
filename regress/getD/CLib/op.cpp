#include "op.h"

void timeDelay(double theSecond){
    int micro_seconds = theSecond*1000*1000;
    usleep(micro_seconds);
}

void string2array(string ts, int sizeA, double **rData){
    double *temp = new double[sizeA];
    stringstream ss(ts);

    int sPos = 0;
    double tData;
    while(1){
        ss >> tData;
        if(ss.fail()) break;
        if(sPos < sizeA){
            temp[sPos++] = tData;
        }
    }
    if(*rData != NULL){
        delete *rData;
        *rData = NULL;
    }
    *rData = temp;
}

string dArray2String(double *dArray, int len){
    string toBack = "";
    char str[256];

    sprintf(str, "%04lf", dArray[0]);
    toBack += str;
    for(int i = 1; i < len; i++){
        sprintf(str, ",%04lf", dArray[i]);
        toBack += str;
    }
    return toBack;
}

/* Parameter Unit: degree */
string getRolling(double ampli, double phase, double speed, double offset,
        double nTime, double *joints, double **outAngle) {
    double tAmpli = PI*ampli/180;
    double tPhase = PI*phase/180;
    double tOffset = PI*offset/180;

    for (int i = 0; i < moduleNum; i++){
        if(i%2 == 0) {
            joints[i] = tAmpli*sin(nTime*speed+i*tPhase)+tOffset;
        }else{
            joints[i] = tAmpli*cos(nTime*speed+i*tPhase)+tOffset;
        }
        //*outAngle[i] = joints[i];
    }
    string tSend = dArray2String(joints, moduleNum-1);
    return tSend;
}