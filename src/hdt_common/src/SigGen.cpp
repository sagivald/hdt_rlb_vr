#include<stdio.h>
#include <math.h>
#include "SigGen.h"
#include "Ekg.h"

SigGen::SigGen() {

}

SigGen::~SigGen() {

}

int SigGen::Init(char *name, double amp, double f, SignalType type, double rate) {
	mName = name;
	mSampleRate = rate;
	mAmplitude = amp;
	mFrequency = f;
	mSignalType = type;
	mSign = 1.;
	mOffset = 0.;
	Output = 0.;
	OutputV = 0.;
	OutputA = 0.;

	EKG->AddSignal((char *)mName, (char *)"Output", &Output);

	return 1;
}

void SigGen::ResetTime() {
	mCount = 0;
	t = 0.;
}

int SigGen::StartChirp(double amp, double f0, double ff, double duration) {
	mAmplitude = amp;
	mFrequency = f0;
	mFrequency2 = ff;
	tf = duration;
	t = 0.;
	mSignalType = CHIRP;

	return 1;
}

int SigGen::StartMultiSine(double amp, double f0, double ff, double numSteps, double duration) {
	mAmplitude = amp;
	mFrequency = f0;
	mFrequency2 = ff;
	tf = duration;
	mSignalType = MULTI_SINE;
	//deltaB = (log10(ff) - log10(f0)) / (numSteps - 1);
	deltaB = (ff - f0) / (numSteps - 1);
	deltaT = duration / numSteps;
	t = 0.;

	return 1;
}

void SigGen::Process() {
	OutputV = 0.;
	OutputA = 0.;

	if (mSignalType == SigGen::SQUARE) {
		if ((mCount % (int) (1. / mFrequency * mSampleRate)) == 0) {
//			mSign = -mSign;
			mSign = 1.0 - mSign;
			Output = mAmplitude * mSign + mOffset;
		}
	} else if (mSignalType == SigGen::SINE) {
		Output = mAmplitude * sin(2. * 3.1415926536 * mFrequency * (double) (mCount / mSampleRate)) + mOffset;
		OutputV = (2. * 3.1415926536 * mFrequency) * mAmplitude * cos(2. * 3.1415926536 * mFrequency * (double) (mCount
				/ mSampleRate));
		OutputA = -(2. * 3.1415926536 * mFrequency) * (2. * 3.1415926536 * mFrequency) * mAmplitude * sin(2.
				* 3.1415926536 * mFrequency * (double) (mCount / mSampleRate));
	} else if (mSignalType == SigGen::CHIRP) {

		//chirpFreq = mFrequency + (mFrequency2 - mFrequency) / tf * t;
		double b = log10(mFrequency2 - mFrequency) / tf;
		curFreq = mFrequency + pow(10., b * t);
		Output = mAmplitude * sin(2 * 3.1415926536 * curFreq * t);
		if (t > tf) {
			Output = 0.;
		}
		t += 1. / mSampleRate;
	} else if (mSignalType == SigGen::MULTI_SINE) {
		if (t < tf) {
			//curFreq = pow(10, log10(mFrequency) + (floor(t / deltaT) * deltaB));
			curFreq = mFrequency + (floor(t / deltaT)) * deltaB;
			Output = mAmplitude * sin(2 * 3.1415926536 * curFreq * t);
		} else {
			Output = 0.;
		}
		t += 1. / mSampleRate;

	}

	mCount++;
}
