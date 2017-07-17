#ifndef hdt_adroit_console_h
#define hdt_adroit_console_h

#include <vector>
#include <string>

#include "AdroitComs.h"
#include "AdroitParams.h"

#include <boost/thread/thread.hpp>

// adroit console class
class AdroitConsole {
public:
	AdroitConsole() {};
	~AdroitConsole() {};

	int Init(AdroitComs *coms, AdroitComs::AdroitDrive **drives);
private:
	void MainMenu(void);
	void MainProcessCommand(std::vector<std::string> lines);
	unsigned int Split(const std::string &s, std::vector<std::string> &elems, char delim);

	AdroitParams *PARAMS;
	AdroitComs *COMS;
	AdroitComs::AdroitDrive **Drives;

	// thread for console loop
	boost::thread *LoopThread;
	void Loop(void);
};

#endif // hdt_adroit_console_h
