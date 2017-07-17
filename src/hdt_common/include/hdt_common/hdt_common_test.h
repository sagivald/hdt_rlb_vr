#ifndef hdt_common_test_h
#define hdt_common_test_h

#include "AperiodicTask.h"
#include "PeriodicTask.h"

class AperiodicTest: public AperiodicTask {
	public:
		AperiodicTest();
		~AperiodicTest();

		int Init(char *name, int priority);
	private:
		void Task(void);
};

class PeriodicTest: public PeriodicTask {
	public:
		PeriodicTest();
		~PeriodicTest();

		int Init(char *name, double rate, int priority);
	private:
		void Task(void);
};

#endif // hdt_common_test_h
