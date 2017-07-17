#ifndef Config_h
#define Config_h

class Config;
extern Config *CONF;

#define CONFIG_LABEL_LEN 50
#define CONFIG_VALBUF_LEN 20
#define CONFIG_TABLE_SIZE 256

class Config {
public:
	// Constuctor
	Config();
	// Destructor
	~Config();
	int Init(char *fname);
	int Read();
	int Write();

	typedef enum {
		NONE, INT, DOUBLE, FLOAT, STRING,
	} ValueType;
	typedef enum {
		NOERROR = 0,
		FILEMISSING = -1,
		CHECKSUMERROR = -2,
		INCOMPLETE = -3,
		TABLESIZE = -4
	} ErrorType;

	void LinkAndSet(char *name, float *val, float defaultVal);
	void LinkAndSet(char *name, double *val, double defaultVal);
	void LinkAndSet(char *name, int *val, int defaultVal);
	void LinkAndSetString(char *name, char *val, char* defaultVal);
	void Dump();

	void GetTypeAndIndexByName(char *name, int *type, int *index);
	void ChangeValueByIndex(int index, float val);
	void ChangeValueByIndex(int index, double val);
	void ChangeValueByIndex(int index, int val);
	double GetDoubleValueByIndex(int index);
	int GetIntValueByIndex(int index);
	float GetFloatValueByIndex(int index);
	int GetError();

private:
	char *filename;
	int configError;
	unsigned int checkSum;

	typedef struct {
		char label[CONFIG_LABEL_LEN];
		char valueBuf[CONFIG_VALBUF_LEN];
		void *valuePtr;
		ValueType type;
	} TableEntry;

	TableEntry configTable[CONFIG_TABLE_SIZE];
	int numEntries;

	int updateChecksum(unsigned int *chksum, const char *str, int len);
	int searchLabel(const char *searchLabel);

};

#endif // Config_h

