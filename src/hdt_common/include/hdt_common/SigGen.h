#ifndef SigGen_h
#define SigGen_h

class SigGen;

//extern SigGen *SIGSOURCE;

class SigGen {
public:
	// Constuctor
	SigGen();
	// Destructor
	~SigGen();

	enum SignalType {
		SQUARE, SINE, CHIRP, MULTI_SINE
	};

	int Init(char *name, double amp, double f, SignalType type, double rate);
	void Process();
	void ResetTime();
	int StartChirp(double amp, double f0, double ff, double duration);
	int StartMultiSine(double amp, double f0, double ff, double numSteps, double duration);


	char *mName;
	int mSignalType;
	double mAmplitude;
	double mFrequency;
	double mOffset;
	double Output;
	double OutputV;
	double OutputA;
	double mFrequency2;
	double t;
	double tf;
	double deltaB;
	double deltaT;
	double curFreq;

private:
	double mSampleRate;
	long mCount;
	double mSign;
};

#endif // SigGen_h

