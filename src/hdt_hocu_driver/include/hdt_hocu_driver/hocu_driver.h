// Header for HOCU Driver

// If the header is not defined then define it
#ifndef hocu_driver_h
#define hocu_driver_h

// Including Used Libraries

// ROS Libraries
#include "ros/ros.h"

// Sensor Messages Libraries
#include "sensor_msgs/Joy.h"

// Communication Libraries
#include "hdt_common/ComsInterface.h"
#include "hdt_common/TcpInterface.h"
#include "hdt_common/SlipProtocol.h"

// HOCU Libraries
#include "hdt_hocu_driver/ClearScreen.h"
#include "hdt_hocu_driver/UpdateScreen.h"
#include "hdt_hocu_driver/SetPageColumn.h"
#include "hdt_hocu_driver/SetFont.h"
#include "hdt_hocu_driver/WriteString.h"

// Defining Global Variables

// general
#define HOCU_NAME				"hocu_driver"
#define HOCU_TOPIC				"/hocu/joy"
#define CLEAR_SCREEN_TOPIC		"/hocu/clear_screen"
#define UPDATE_SCREEN_TOPIC		"/hocu/update_screen"
#define SET_PAGE_COLUMN_TOPIC	"/hocu/set_page_column"
#define SET_FONT_TOPIC			"/hocu/set_font"
#define WRITE_STRING_TOPIC		"/hocu/write_string"
#define	HOCU_NUM_AXES			10
#define HOCU_NUM_BUTTONS		5

// HOCU Driver Class
class HocuDriver {
public:
	// setup
	HocuDriver(int port) { Port = port; };
	~HocuDriver() {};

	//	hocu data structure definition
	typedef struct HocuDataType{
		float axisX;
		float axisY;
		float axisZ;
		float rotX;
		float rotY;
		float rotZ;
		float thumbRoll;
		float thumbYaw;
		float indexRoll;
		float ringRoll;
		uint32_t handTrigger;
		uint32_t buttons;
		uint32_t timestamp;
	} HocuDataType;

	//	Defining HOCU Messages ID
	enum HocuMsgIdType {
	  	HOCU_INVALID_ID			= 0x00,
	  	HOCU_HEARTBEAT_CMD		= 0x01,
	  	HOCU_HEARTBEAT_RES		= 0x02,
	  	HOCU_DATA				= 0x51,
	  	HOCU_MODE				= 0x52,
	  	HOCU_UPDATE_CAL			= 0x53,
	  	HOCU_CLEAR_SCREEN		= 0x54,
	  	HOCU_CLEAR_PAGES		= 0x55,
	  	HOCU_SET_PAGE_COLUMN	= 0x56,
	  	HOCU_SET_FONT			= 0x57,
	  	HOCU_WRITE_STRING		= 0x58,
	  	HOCU_BACKLIGHT			= 0x59,
	  	HOCU_UPDATE_SCREEN		= 0x60
	};

	int Init(void);
private:
	// CallBacks functions
	void TimerCb(const ros::TimerEvent &event);

	int Step(void);
	int CheckMsgId(uint8_t id);

	void ProcessHocuDataCmd(uint8_t *data);
	void SendHeartbeatCmd(void);
	void SendHocuClearScreen(const hdt_hocu_driver::ClearScreen& msg);
	void SendHocuUpdateScreen(const hdt_hocu_driver::UpdateScreen& msg);
	void SendHocuSetPageColumn(const hdt_hocu_driver::SetPageColumn& msg);
	void SendHocuSetFont(const hdt_hocu_driver::SetFont& msg);
	void SendHocuWriteString(const hdt_hocu_driver::WriteString& msg);

	// Variables

	int Port;

	// TCP Communication Variables
	ComsInterface *ComsServer;

	// Timer Variables
	ros::Timer sample_loop_timer;

	// Subscribers
	ros::Subscriber clear_screen_sub;
	ros::Subscriber update_screen_sub;
	ros::Subscriber set_page_column_sub;
	ros::Subscriber set_font_sub;
	ros::Subscriber write_string_sub;

	// Publishers
	ros::Publisher joy_pub;

	// Messages
	sensor_msgs::Joy joy_msg;
};

#endif // hocu_driver_h

