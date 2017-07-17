/**
  ******************************************************************************
  ******************************************************************************
  * @file    Util.cpp
  * @author  HDT Robotics, Inc.
  * @brief   Utility functions.
  ******************************************************************************
  ******************************************************************************
  */

#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "Util.h"

/**
 ******************************************************************************
 * @brief lookup table initialization
 ******************************************************************************
 */;
int Relay::Init(double switch_on, double switch_off, double val_on, double val_off) {
	s_on = switch_on;
	s_off = switch_off;
	v_on = val_on;
	v_off = val_off;
	Output = v_off;
	Change = 0;
	State = RELAY_OFF;

	return 1;
}

/**
 ******************************************************************************
 * @brief lookup table compute function
 ******************************************************************************
 */
void Relay::Compute(double x) {
	Change = 0;

	switch(State) {
	case(RELAY_OFF):
		if(x >= s_on) {
			State = RELAY_ON;
			Change = 1;
		}
		break;
	case(RELAY_ON):
		if(x <= s_off) {
			State = RELAY_OFF;
			Change = 1;
		}
		break;
	default:
		break;
	}

	if(State == RELAY_ON) {
		Output = v_on;
	}
	else {
		Output = v_off;
	}
}

/**
 ******************************************************************************
 * @brief  PID initialization
 ******************************************************************************
 */
int Pid::Init(char dir, double kp, double ki, double kd, double wc, double max, double min, double imax, double imin, double rate) {
	// set internal variables
	mDir = dir;
	mImax = imax;
	mImin = imin;
	mMax = max;
	mMin = min;
	mRate = rate;
	Kp = kp;
	Ki = ki;
	Kd = kd;

	Fault = 0;
	mPrevError = 0;

//	mEmax = emax;
//	TIC->Request(&mErrorTimer, etime);

	mDerivFof = new Fof();
	mDerivFof->Init(wc, rate);

	return 1;
}

/**
 ******************************************************************************
 * @brief  PID step function
 ******************************************************************************
 */
double Pid::Step(double des, double meas, double ff) {
	double Out;

	// clear faults
	Fault = 0;

	// set-point error
	Error = (double)mDir * (des - meas);

	// update integral only if saturation is not in direction of error
	if (((Error > 0) && (mSat > 0)) || ((Error < 0) && (mSat < 0))) {
		// don't update integral
	}
	else {
		// update integral, check for error conditions
		mImem += Ki * Error / mRate;
		if (mImem > mImax) {
			Fault = INTEGRALSAT;
			mImem = mImax;
		}
		else if (mImem < mImin) {
			Fault = INTEGRALSAT;
			mImem = mImin;
		}
	}

	// set output
	Out = Kp*Error + mImem + Kd*mDerivFof->Step((Error - mPrevError)*mRate) + ff;

	// check for saturation, generate fault and update flags as required
	if (Out > mMax) {
		Fault = SATURATION;
		mSat = 1;
		Out = mMax;
	}
	else if (Out < mMin) {
		Fault = SATURATION;
		mSat = -1;
		Out = mMin;
	}
	else {
		mSat = 0;
	}

	mPrevError = Error;

	return Out;
}

/**
 ******************************************************************************
 * @brief lookup table initialization
 ******************************************************************************
 */
int Lut::Init(unsigned int size, double *xt, double *yt, Mode m) {
	// set internal variables
	mSize = size;
	mMode = m;

	// allocate memory for tables
	mXt = (double *)malloc(mSize*sizeof(double));
	mYt = (double *)malloc(mSize*sizeof(double));

	// copy table data
	memcpy(mXt, xt, mSize*(sizeof(double)));
	memcpy(mYt, yt, mSize*(sizeof(double)));

	return 1;
}

/**
 ******************************************************************************
 * @brief lookup table compute function
 ******************************************************************************
 */
double Lut::Compute(double x) {
	// make sure it's a table
	if (mSize < 2) {
		return 0;
	}

	// check if x is below lower bound, take appropriate action
	if (x <= mXt[0]) {
		switch (mMode) {
		case (LIMIT):
			return mYt[0];
		case (INTERPOLATE):
			return (mYt[0] + (x - mXt[0]) / (mXt[1] - mXt[0]) * (mYt[1] - mYt[0]));
		}
	}
	// check if x is above upper bound, take appropriate action
	else if (x >= mXt[mSize - 1]) {
		switch (mMode) {
		case (LIMIT):
			return mYt[mSize - 1];
		case (INTERPOLATE):
			return (mYt[mSize - 1] + (x - mXt[mSize - 1]) / (mXt[mSize - 1] - mXt[mSize - 2]) * (mYt[mSize - 1] - mYt[mSize - 2]));
		}
	}
	// x is within bounds
	else {
		for (unsigned int i = 1; i < mSize; i++) {
			// find indexes that surround X
			if (x > mXt[i]) {
				continue;
			}
			// interpolate between them
			else {
				return (mYt[i - 1] + (x - mXt[i - 1]) / (mXt[i] - mXt[i - 1]) * (mYt[i] - mYt[i - 1]));
			}
		}
	}

	// should never reach this
	return 0;
}

/**
 ******************************************************************************
 * @brief first order filter initialization
 ******************************************************************************
 */
int Fof::Init(double wc, double rate) {
	double RC;
	int ret;

	if ((wc > 0) && (rate > 0)) {
		// calculate filter's equivalent RC value
		RC = 1 / (2 * (double) M_PI * wc);

		// set alpha, init memory
		mAlpha = (1/rate)/(RC + (1/rate));

		ret = 1;
	}
	else {
		// filter parameters invalid, don't filter input
		mAlpha = 1.0;
		ret = 0;
	}

	mMem = 0;

	return ret;
}

/**
 ******************************************************************************
 * @brief first order filter step function
 ******************************************************************************
 */
double Fof::Step(double x) {
	double Out;

	// calculate output, update memory
	Out = x*mAlpha + mMem*(1 - mAlpha);
	mMem = Out;

	return Out;
}

/**
 ******************************************************************************
 * @brief moving average initialization
 ******************************************************************************
 */
int Mavg::Init(unsigned int num) {
	// initialize data
	mNum = num;
	mInd = 0;
	mSum = 0;
	mBuffer = (double *)malloc(mNum * sizeof(double));

	return 1;
}

/**
 ******************************************************************************
 * @brief moving average step
 ******************************************************************************
 */
double Mavg::Step(double x) {
	double Out;

	// update sum, set new value in buffer
	mSum += x - mBuffer[mInd];
	mBuffer[mInd] = x;

	// divide by num to average
	Out = mSum/mNum;

	// update index
	mInd = (mInd + 1)%mNum;

	return Out;
}


/**
 ******************************************************************************
 * @brief deadband function
 ******************************************************************************
 */
void deadband(double *val, double db){
	if(*val > db){
		*val -= db;
	}
	else if(*val < -db){
		*val += db;
	}
	else{
		*val = 0.0;
	}
}

/**
 ******************************************************************************
 * @brief limit function
 ******************************************************************************
 */
int limit(double *val, double min, double max){
	if(min > max)
		return 0;

		if(*val > max){
			*val = max;
			return 1;
		}
		else if(*val < min){
			*val = min;
			return -1;
		}
	return 0;
}

/**
 ******************************************************************************
 * @brief limit function
 ******************************************************************************
 */
int limit(float *val, float min, float max){
	if(min > max)
		return 0;

		if(*val > max){
			*val = max;
			return 1;
		}
		else if(*val < min){
			*val = min;
			return -1;
		}
	return 0;
}

/**
 ******************************************************************************
 * @brief limit function
 ******************************************************************************
 */
int limit(int *val, int min, int max){
	if(min > max)
		return 0;

		if(*val > max){
			*val = max;
			return 1;
		}
		else if(*val < min){
			*val = min;
			return -1;
		}
	return 0;
}

/**
 ******************************************************************************
 * @brief limit with excess
 ******************************************************************************
 */
double limitAndExcess(double *val, double min, double max){
	double excess;
	
	if(min > max)
		return 0.;

	if(*val > max){
		excess = *val - max;
		*val = max;
		return excess;
	}
	else if(*val < min){
		excess = *val - min;
		*val = min;
		return excess;
	}
	
	return 0.;
}

/**
 ******************************************************************************
 * @brief 0/2pi rollover
 ******************************************************************************
 */
double plusMinusPiValue(double pos){
	double v = pos - floor(pos / TWO_PI) * TWO_PI;

	if ( v <= - PI){
    v += TWO_PI;
	}
	else if ( v > PI ){
    v -= TWO_PI;
	}

	return v;
}

/**
 ******************************************************************************
 * @brief -pi/pi rollover
 ******************************************************************************
 */
double zeroTwoPiValue(double pos){
	return (pos - floor(pos / TWO_PI) * TWO_PI);
}

int virtualStop(double pos, double minPos, double maxPos, double *vCmd){
	if( ((pos > maxPos) && (*vCmd > 0.)) ){
		*vCmd = 0.;
		return 1;
	}
	else if( ((pos < minPos) && (*vCmd < 0.)) ) {
		*vCmd = 0.;
		return -1;
	}

	return 0;
}

/**
 ******************************************************************************
 * @brief normalized 0 to 1
 ******************************************************************************
 */
double normVirtualStopDistance(double pos, double minPos, double maxPos, double maxDist){
	double normDist = 0; // range 0-1, 0: more than maxDist away from stop, 1: at or beyond stop
	

	if(pos >= maxPos){
		normDist = 1.;
	}
	else if(pos > (maxPos - maxDist)){
		normDist = (maxDist - (maxPos - pos)) / maxDist;
	}

	if(pos <= minPos){
		normDist = 1.;
	}
	else if(pos < (minPos + maxDist)){
		normDist = (maxDist - (pos - minPos)) / maxDist;

	}

	return normDist;
}

/**
 ******************************************************************************
 * @brief x = v^2/(2*a)
 ******************************************************************************
 */
int virtualStopWithDecel(double pos, double minPos, double maxPos, double vMax, double decelDist, double *vCmd){
	if(*vCmd > 0.){
		if(pos >= maxPos){
			*vCmd = 0.;
		}
		else if(pos > (maxPos - decelDist)){
			limit(vCmd, 0., vMax * sqrt((maxPos - pos) / decelDist));

		}
	}
	else if(*vCmd < 0.){
		if(pos <= minPos){
			*vCmd = 0.;
		}
		else if(pos < (minPos + decelDist)){
			limit(vCmd, -vMax * sqrt((pos - minPos) / decelDist), 0.);

		}
	}
	return 0;
}

/**
 ******************************************************************************
 * @brief rate limit
 ******************************************************************************
 */
int rateLimit(double maxRate, double sigPrev, double *sig, double sampleRate){
	if(fabs(*sig - sigPrev) * sampleRate > maxRate){
		if(*sig > sigPrev){
			*sig = sigPrev + maxRate / sampleRate;
		}
		else{
			*sig = sigPrev - maxRate / sampleRate;
		}
	}

	return 1;
}

/**
 ******************************************************************************
 * @brief rate limit for rotational
 ******************************************************************************
 */
int rateLimitRot(double maxRate, double sigPrev, double *sig, double sampleRate){
	if(fabs(plusMinusPiValue(*sig - sigPrev)) * sampleRate > maxRate){
		if(*sig > sigPrev){
			*sig = sigPrev + maxRate / sampleRate;
		}
		else{
			*sig = sigPrev - maxRate / sampleRate;
		}
	}

	return 1;
}

/**
 ******************************************************************************
 * @brief change velocity with max acceleration
 ******************************************************************************
 */
int accLimit(double maxAcc, double xpp, double xp, double *x, double sampleRate){
	double a;
	double v;
	double vp;

	v = (*x - xp) * sampleRate;
	vp= (xp - xpp) * sampleRate;

	a = (v - vp) * sampleRate;

	if(fabs(a)>maxAcc){
		if(a>0.){
			v = maxAcc/sampleRate + vp;
			*x = (v/sampleRate + xp);
		}
		else{
			v = -maxAcc/sampleRate + vp;
			*x = (v/sampleRate + xp);
		}

		return 1;
	}
	return 0;
}

/**
 ******************************************************************************
 * @brief saturate output to threshold
 ******************************************************************************
 */
double adaptiveLimit(double inputThreshold, double input){
	if( fabs(input) <= inputThreshold){
		return( 1./ inputThreshold * fabs(input));
	}
	else{
		return 1.;
	}
}

/**
 ******************************************************************************
 * @brief saturating 2DOF lookup table
 ******************************************************************************
 */
double linearMap(double inMin, double inMax, double outMin, double outMax, double in){
	if(in < inMin){
		return outMin;
	}
	else if(in > inMax){
		return outMax;
	}
	else{
		return ( (outMax - outMin) / (inMax - inMin) * (in - inMin) + outMin );
	}
}

/**
 ******************************************************************************
 * @brief cubed output
 ******************************************************************************
 */
double qubicScale(double in, double max){
	if(in > max){
		return 1.;
	}
	else if(in < -max){
		return -1.;
	}
	else{
		return (in*in*in) / (max*max*max);
	}
}

/**
 ******************************************************************************
 * @brief signed quadratic
 ******************************************************************************
 */
double signedQuadratic(double in){
	if(in < 0.){
		return -(in * in);
	}
	else{
		return (in * in);
	}
}

/**
 ******************************************************************************
 * @brief returns sign
 ******************************************************************************
 */
double sign(double n){
	if(n > 0.0){
		return 1;
	}
	else if(n < 0.0) {
		return -1;
	}
	else {
		return 0;
	}
}

/**
 ******************************************************************************
 * @brief returns max of two values
 ******************************************************************************
 */
double max(double a, double b){
	if(a > b){
		return a;
	}
	else{
		return b;
	}
}

/**
 ******************************************************************************
 * @brief returns min of two values
 ******************************************************************************
 */
double min(double a, double b){
  if(a < b){
    return a;
  }
  else{
    return b;
  }
}

/**
 ******************************************************************************
 * @brief returns magnitude of vector
 ******************************************************************************
 */
double vectMag(double x, double y){
  return sqrt(x*x + y*y);
}

/**
 ******************************************************************************
 * @brief return saturated vector magnitude
 ******************************************************************************
 */
int limitVectMag(double *x, double *y, double maxMag){
  double m = vectMag(*x, *y);
  
  if(m > maxMag){   
    *x *= maxMag/m;
    *y *= maxMag/m;

    return 1;
  }

  return 0;
}

/**
 ******************************************************************************
 * @brief return elliptically limited vector
 ******************************************************************************
 */
int limitVectMagElliptical(double *x, double *y, double maxMagX, double maxMagY){
	double th, r;
	double xL, yL;
	
	// find vector orientation
	th = atan2(*y, *x);
	
	// find radius of the ellipse at the vector orientation
	r = (maxMagX * maxMagY) / sqrt( pow(maxMagY*cos(th), 2.) + pow(maxMagX*sin(th), 2.) );
	
	// convert from polar coords (these are the limits)
	xL = fabs(r*cos(th));
	yL = fabs(r*sin(th));
	
	// limit
	limit(x, -xL, xL);
	limit(y, -yL, yL);
	
	return 1;
}

