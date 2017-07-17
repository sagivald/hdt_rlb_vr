#ifndef hdt_adroit_coms_h
#define hdt_adroit_coms_h

#include "AdroitComs.h"

#define ROS_NAME				"hdt_adroit_coms"
#define SAMPLE_RATE				50.0
#define SAMPLE_PERIOD 			1.0/SAMPLE_RATE
#define DEBUG_MSG_RATE                   2*500/SAMPLE_RATE
#define HDT_TELEM_TOPIC		 	"/hdt_adroit_coms/hdt_telem"
#define JOINT_TELEM_TOPIC 		"/hdt_adroit_coms/joint_telem"
#define JOINT_CMD_TOPIC			"/hdt_adroit_coms/joint_cmd"
#define HDT_JOINT_CMD_TOPIC             "/hdt_adroit_coms/hdt_joint_cmd"
#define DRIVE_ERROR_TOPIC	 	"/hdt_adroit_coms/drive_error"
#define MAX_DRIVE_CURRENT		13.5
#define COMS_READY_SERVICE		"/hdt_adroit_coms/coms_ready"
#define WRITE_PARAM_SERVICE             "/hdt_adroit_coms/write_drive_param"
#define READ_PARAM_SERVICE              "/hdt_adroit_coms/read_drive_param"
#define APP_LOAD_SERVICE                "/hdt_adroit_coms/app_load"
#define STATE_SELECT_SERVICE            "/hdt_adroit_coms/state_select"
#define GET_STATUS_SERVICE              "/hdt_adroit_coms/get_status"
#define COMMIT_SERVICE                  "/hdt_adroit_coms/commit"
#define INIT_WAIT_TIME			1000000
#define JOINT_CMD_TIMEOUT		0.5

// coms interface class
class AdroitInterface: public AdroitComs {
public:
	AdroitInterface() : AdroitComs(0) {};
	~AdroitInterface() {};

	virtual int Init(void) = 0;
	virtual int SendMsg(AdroitMsg *adroit_msg) = 0;
	virtual int ReceiveMsg(AdroitMsg *msg) = 0;
	virtual int Write(void) = 0;
private:
};

#endif // hdt_adroit_coms_h

