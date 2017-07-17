/**
 ******************************************************************************
 ******************************************************************************
 * @file    Util.h
 * @brief   Utility functions.
 ******************************************************************************
 ******************************************************************************
 */

#ifndef Util_h
#define Util_h

#include <stdlib.h>
#include "Timer.h"

#define PI 			3.1415926536
#define TWO_PI 		6.2831853072

class Pid;
class Lut;
class Fof;
class Mavg;

// PID control loop class
class Pid {
public:
	enum FaultType {
		NOFAULT			= 0x00,
		SATURATION		= 0x01,
		INTEGRALSAT		= 0x02,
		TRACKINGERROR	= 0x04
	};
	Pid() {};
	~Pid() {};
	int Init(char dir, double kp, double ki, double kd, double wc, double max, double min, double imax, double imin, double rate);
	double Step(double des, double meas, double ff);
	void Reset(double x) { mImem = x; Fault = NOFAULT; Error = 0;};

	int Fault; 					// error bitmask
	double Kp; 					// proportional gain
	double Ki; 					// integral gain
	double Kd; 					// derivative gain
	double mImem; 				// integral memory
	double mImax; 				// integral saturation max
	double mImin; 				// integral saturation min
	double mMax; 				// saturation max
	double mMin;				// saturation min
	double Error;				// error
private:
	Fof	*mDerivFof;				// filter for derivative
	double mPrevError;			// for derivative calculation

	char mDir; 					// control direction
	double mRate; 				// rate of function call
	char mSat; 					// saturation and direction (+1, 0, -1)

//	double mEmax;				// error limit before starting timer
//	Tic::Timer mErrorTimer;		// error timer
};

// simulink style relay class
class Relay {
public:
	enum RelayState { RELAY_OFF, RELAY_ON };
	Relay() {};
	~Relay() {};
	int Init(double switch_on, double switch_off, double val_on, double val_off);
	void Compute(double x);
	double Output;
	int Change;
private:
	double s_on;					// switch on point
	double s_off;					// switch off point
	double v_on;					// value when on
	double v_off;					// value when off
	RelayState State;				// current state
};

// 1D lookup table class
class Lut {
public:
	enum Mode { LIMIT, INTERPOLATE 	};
	Lut() {};
	~Lut() { free(mXt); free(mYt); };
	int Init(unsigned int size, double *xt, double *yt, Mode m);
	double Compute(double x);

private:
	unsigned int mSize;			// lookup table size
	double *mXt;				// input table
	double *mYt;				// output table
	Mode mMode;					// mode
};

// first order filter class
class Fof {
public:
	Fof() {};
	~Fof() {} ;
	int Init(double wc, double rate);
	void Reset(double x) { mMem = x; };
	double Step(double x);

private:
	double mAlpha;			// FOF Alpha
	double mMem;			// memory of last output
};

// moving average class
class Mavg {
public:
	Mavg() {};
	~Mavg() { free(mBuffer); };
	int Init(unsigned int num);
	double Step(double x);

private:
	unsigned int mNum;		// number of averages
	unsigned int mInd;		// current index
	double *mBuffer;			// buffer of current data points to be averaged
	double mSum;				// sum of current buffer
};

void deadband(double *val, double db);
int limit(double *val, double min, double max);
int limit(float *val, float min, float max);
int limit(int *val, int min, int max);
double plusMinusPiValue(double pos);
double zeroTwoPiValue(double pos);
double normVirtualStopDistance(double pos, double minPos, double maxPos, double maxDist);
int virtualStop(double pos, double minPos, double maxPos, double *vCmd);
int virtualStopWithDecel(double pos, double minPos, double maxPos, double vMax, double decelDist, double *vCmd);
int rateLimit(double maxRate, double sigPrev, double *sig, double sampleRate);
int rateLimitRot(double maxRate, double sigPrev, double *sig, double sampleRate);
int accLimit(double maxAcc, double sigPrevPrev, double sigPrev, double *sig, double sampleRate);
double adaptiveLimit(double inputThreshold, double input);
double linearMap(double inMin, double inMax, double outMin, double outMax, double in);
double qubicScale(double in, double max);
double signedQuadratic(double in);
double sign(double n);
double max(double a, double b);
double min(double a, double b);
double vectMag(double x, double y);
int limitVectMag(double *x, double *y, double maxMag);
double limitAndExcess(double *val, double min, double max);
int limitVectMagElliptical(double *x, double *y, double maxMagX, double maxMagY);

#endif // Util_h
