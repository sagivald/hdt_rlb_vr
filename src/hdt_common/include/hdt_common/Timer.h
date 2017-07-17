#ifndef Timer_h
#define Timer_h

class Timer;

extern Timer *TMR;

class Timer{
    public:
		enum StartState {
			START_OFF				= 0,
			START_ON				= 1
						};
        // Constuctor
        Timer();
        // Destructor
        ~Timer();

		  int Init(double rate);

		  class TimerData{
			public:
				TimerData(){stopTime = 0; armed = 0; timerSec = 0.0;}
				unsigned long long stopTime;
				unsigned long long startTime;
				int armed;
				double timerSec;

		  };

		  int Process();
		  int Set(TimerData *td, double timeInSec);	// Set timer for timeInSec in the future
		  int Set(TimerData *td, double timeInSec, StartState state);	//Set timer for timeInSec but determine if initially on or off
		  int Armed(TimerData *td);					// Check if timer is armed
		  int TimeUp(TimerData *td);                // Check if time is up
		  double Overtime(TimerData *td);           // Check how long ago timer expired
		  double Elapsed(TimerData *td);
		  int Disarm(TimerData *td);				// Disable the timer, Set() arms it
		  int Reset(TimerData *td);					// Reset timer to stored value
    private:
		  double sampleRate;
		  unsigned long long ticks;
		  double ElapsedTime_Sec;
};

#endif // Timer_h



