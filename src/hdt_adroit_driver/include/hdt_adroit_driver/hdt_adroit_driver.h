#ifndef hdt_adroit_driver_h
#define hdt_adroit_driver_h

#include "ros/ros.h"

#include "sensor_msgs/JointState.h"
#include "hdt_adroit_driver/HDTJointState.h"
#include "hdt_adroit_driver/HDTDebugTelem.h"
#include "hdt_adroit_driver/HDTLSTelem.h"
#include "hdt_adroit_driver/HDTMSTelem.h"
#include "hdt_adroit_driver/HDTHSTelem.h"
#include "hdt_adroit_driver/HDTErrorTelem.h"
#include "hdt_adroit_driver/HDTParameterTelem.h"
#include "hdt_adroit_driver/HDTStatusTelem.h"
#include "hdt_adroit_driver/HDTControlCmdTelem.h"

//#include "hdt_adroit_driver/DriveError.h"

#include "hdt_adroit_driver/ComsReady.h"
#include "hdt_adroit_driver/ReadDriveParam.h"
#include "hdt_adroit_driver/WriteDriveParam.h"
#include "hdt_adroit_driver/AppLoad.h"
#include "hdt_adroit_driver/StateSelect.h"
#include "hdt_adroit_driver/GetStatus.h"
#include "hdt_adroit_driver/Commit.h"

#include "hdt_adroit_coms.h"
#include "hdt_adroit_console.h"

#define ROBOT_DESCRIPTION	"robot_description"
#define PARAM_WAIT_DURATION	1.0

typedef std::map<std::string, int>::iterator it_type;
typedef std::map<int,ros::Publisher> pub_map;
typedef std::map<int,std::string> topic_map;
// coms interface class
class AdroitDriver {
public:
	AdroitDriver() {};
	~AdroitDriver() {};

	int Init(AdroitInterface *coms_interface);
private:

	int msg_count;

	// functions
	int ProcessMessages(void);
	void TimerCallback(const ros::TimerEvent &event);
	void JointCmdCallback(const sensor_msgs::JointState& msg);
	void HDTJointCmdCallback(const hdt_adroit_driver::HDTJointState& msg);
	
	bool ComsReadyCallback(hdt_adroit_driver::ComsReady::Request&req, hdt_adroit_driver::ComsReady::Response& res);
        bool WriteParamServiceCallback(hdt_adroit_driver::WriteDriveParam::Request&req, hdt_adroit_driver::WriteDriveParam::Response& res);

	bool ReadParamServiceCallback(hdt_adroit_driver::ReadDriveParam::Request&req, hdt_adroit_driver::ReadDriveParam::Response& res);

	bool AppLoadServiceCallback(hdt_adroit_driver::AppLoad::Request&req, hdt_adroit_driver::AppLoad::Response& res);
	bool StateSelectServiceCallback(hdt_adroit_driver::StateSelect::Request&req, hdt_adroit_driver::StateSelect::Response& res);

	bool GetStatusServiceCallback(hdt_adroit_driver::GetStatus::Request&req, hdt_adroit_driver::GetStatus::Response& res);

	bool CommitServiceCallback(hdt_adroit_driver::Commit::Request&req, hdt_adroit_driver::Commit::Response& res);

	void SendImpedanceCommands(double damping_ratio, double stiffness_ratio, bool stiffness_enabled = true);
	void ResetDrives(void);

	// adroit drives
	AdroitComs::AdroitDrive *adroit_drives[MAX_DRIVES];
	std::map<std::string, int> active_drives;

	//adroit params
	AdroitParams *PARAMS;

	// coms interface
	AdroitInterface *COMS;

	// joint state publisher
	ros::Publisher joint_telem_pub;
	ros::Publisher hdt_telem_pub;
	pub_map hdt_debug_telem_pub;
	pub_map hdt_ls_telem_pub;
	pub_map hdt_ms_telem_pub;
	pub_map hdt_hs_telem_pub;
	pub_map hdt_error_telem_pub;
	pub_map hdt_param_telem_pub;
	pub_map hdt_status_telem_pub;
	pub_map hdt_can_control_cmd_telem_pub;
	pub_map hdt_control_cmd_telem_pub;
	//ros::Publisher drive_error_pub;

	// joint command subscriber
	ros::Subscriber joint_cmd_sub;
	ros::Subscriber hdt_joint_cmd_sub;

	// joint message holder
	sensor_msgs::JointState joint_telem_msg;
	hdt_adroit_driver::HDTJointState hdt_telem_msg;
	std::map<int,hdt_adroit_driver::HDTDebugTelem> hdt_debug_telem_msg;
	std::map<int,hdt_adroit_driver::HDTLSTelem> hdt_ls_telem_msg;
	std::map<int,hdt_adroit_driver::HDTMSTelem> hdt_ms_telem_msg;
	std::map<int,hdt_adroit_driver::HDTHSTelem> hdt_hs_telem_msg;
	std::map<int,hdt_adroit_driver::HDTErrorTelem> hdt_error_telem_msg;
	std::map<int,hdt_adroit_driver::HDTParameterTelem> hdt_param_telem_msg;
	std::map<int,hdt_adroit_driver::HDTStatusTelem> hdt_status_telem_msg;
	std::map<int,hdt_adroit_driver::HDTControlCmdTelem> hdt_can_control_cmd_telem_msg;
	std::map<int,hdt_adroit_driver::HDTControlCmdTelem> hdt_control_cmd_telem_msg;

	// topic names
	topic_map hdt_debug_telem_topics;
	topic_map hdt_ls_telem_topics;
	topic_map hdt_ms_telem_topics;
	topic_map hdt_hs_telem_topics;
	topic_map hdt_error_telem_topics;
	topic_map hdt_param_telem_topics;
	topic_map hdt_status_telem_topics;
	topic_map hdt_can_control_cmd_telem_topics;
	topic_map hdt_control_cmd_telem_topics;

	// coms ready service
	ros::ServiceServer coms_ready;

	// write parameter service
	ros::ServiceServer write_param_srv;
	
	// read parameter service
	ros::ServiceServer read_param_srv;
	
	// application loader service
	ros::ServiceServer app_load_srv;

	// state select service
	ros::ServiceServer state_select_srv;

	// status service
	ros::ServiceServer get_status_srv;

	// commit service
	ros::ServiceServer commit_srv;

	// adroit console
	AdroitConsole adroit_console;

	// params
	bool control_enabled;
	double stiffness_ratio, damping_ratio;
	bool impedance_enabled, console_enabled;

	// timers
	ros::Time joint_cmd_time;
	std::vector<ros::Time> drive_timers;
};

#endif // hdt_adroit_driver_h
