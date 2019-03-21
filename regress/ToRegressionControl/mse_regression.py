import preWork
import dataRead
import numpy
import definition

class DisData:
	pass

trainData = dataRead.trainData
trainParameter = dataRead.trainParameter

pointsArray = preWork.pointsArray
deviation = preWork.deviation

coefficient = []
tCoefficient = []
gradStep = []

reg = 0
paraControl = [0 for i in range(4)]

theVector = DisData()

def getAim():
	rAccuracy = 10000;
	rNum = random(rAccuracy)
	rRate = 1.0*rNum / rAccuracy
	aimP = 0
	nowDivide = 0.0
	for i in range(3):
		nowDivide += deviation[i]
		if(rRate < nowDivide):
			break
		aimP += 1
	return aimP

def getFitting():
	tLen = definition.JOINT_NUM + 1
	for i in range(tLen+1):
		coefficient.append(0.0)
		tCoefficient.append(0.0)
		gradStep.append(0.0)
	cFlag = True
	pointsArraytmp = numpy.array(pointsArray)
	aL = pointsArraytmp.shape
	iterNum = 0

	while(cFlag && iterNum < definition.MAX_ITER):
		for i in range(tLen+1):
			gradStep[i] = 0.0
		for i in range(aL):
			fValue = 0.0
			fValue += coefficient[0]
			nPoint = preWork.pointsArray[i]
			sRate = trainData[nPoint].accelZ / 40.0

			for j in range(definition..JOINT_NUM+1):
				fValue += (coefficient[j] * trainData[nPoint].joints[j-1])
			fValue += coefficient[definition.JOINT_NUM+1]*trainData[nPoint].jointDiff

			fDiff = (fValue - trainParameter[nPoint][reg])*sRate
			gradStep[0] += fDiff

			for j in range(definition.JOINT_NUM):
				gradStep[j] += (fDiff*trainData[nPoint].joints[j-1])
			gradStep[definition.JOINT_NUM+1] += (fDiff * trainData[nPoint].jointDiff)

		tCoefficient[0] = coefficient[0] - gradStep[0] / aL
		for i in range(definition.JOINT_NUM+1):
			tCoefficient[i] = coefficient[i] - gradStep[i] / aL
		tCoefficient[definition.JOINT_NUM+1] = coefficient[definition.JOINT_NUM+1]-gradStep[definition.JOINT_NUM+1]/aL

		vcDis = definition.getVectorDis(coefficient,tCoefficient)

		if(vcDis < definition.NEAR_ZERO):
			cFlag = False
		else:
			coefficient = tCoefficient
		iterNum++

def getRegression(tp):
	aimValue = 0.0
	aimValue += coefficient[0]
	for i in range(definition.JOINT_NUM+1)：
		aimValue += tp.joints[i-1]*coefficient[i]

	aimValue += tp.jointDiff*coefficient[definition.JOINT_NUM+1]

	paraControl[0] = tp.ampli
	paraControl[1] = tp.phase
	paraControl[2] = tp.speed
	paraControl[3] = tp.offset

	for i in range(3):
		printf(paraControl[i])
	paraControl[reg] = aimValue

	if(paraControl[2] > 3):
		paraControl[2] = 3.0