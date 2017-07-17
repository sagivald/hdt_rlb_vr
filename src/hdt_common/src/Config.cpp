#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "Config.h"

Config::Config(){
}

Config::~Config(){
}


int Config::Init(char *fname){
  filename = fname;

	numEntries = 0;
	for(int i = 0; i < CONFIG_TABLE_SIZE; i++){
		configTable[i].type = NONE;
	}
	configError = NOERROR;
	return Read();

}

int Config::Read(){
	FILE *fp;
	int stat;
	unsigned int recCheckSum;
	numEntries = 0;
	fp = fopen(filename, "r");

	if(fp == NULL){
		printf("Config::File does not exist\n");
		configError = FILEMISSING;
		return configError;
	}

	checkSum = 0;
	while(1){
		stat = fscanf(fp, "%s = %s", configTable[numEntries].label, configTable[numEntries].valueBuf);
		if(stat == EOF){
			break;
		}

		// don't compute checksum on the checksum entry in the file
		if(strcmp(configTable[numEntries].label, "checksum") != 0){
			updateChecksum(&checkSum, configTable[numEntries].label, strlen(configTable[numEntries].label));
			updateChecksum(&checkSum, configTable[numEntries].valueBuf, strlen(configTable[numEntries].valueBuf));
			numEntries++;
		}

		if(numEntries >= (CONFIG_TABLE_SIZE - 1)){
			printf("Config::Table size exceeded\n");
			fclose(fp);
			configError = TABLESIZE;
			return configError;
		}

	}

	if(strcmp(configTable[numEntries].label, "checksum") != 0){
		printf("Config::File does not contain a checksum %d\n", checkSum);
		fclose(fp);
		configError = CHECKSUMERROR;
		return configError;
	}

	sscanf(configTable[numEntries].valueBuf, "%d", &recCheckSum);
	if(recCheckSum != checkSum){
		printf("Config::Checksum error %d != %d\n", checkSum, recCheckSum);
		fclose(fp);
		configError = CHECKSUMERROR;
		return configError;
	}

//	printf("Config::Successfully read %d entries from the file\n", numEntries);
	fclose(fp);

	return 1;
}


int Config::updateChecksum(unsigned int *chksum, const char *str, int len){
	for(int i = 0; i < len; i++){
		*chksum += str[i];
	}

	return 1;
}

int Config::searchLabel(const char *searchLabel){
	for(int i = 0; i < numEntries; i++){
		if(strcmp(searchLabel, configTable[i].label) == 0){
			return i;
		}
	}

	return -1;
}
#if 1
void Config::LinkAndSetString(char *name, char *val, char* defaultVal){
	int index;

	index = searchLabel(name);

	if(index == -1){
		if(numEntries >= (CONFIG_TABLE_SIZE - 1)){
			printf("Config::Table size exceeded\n");
			configError = TABLESIZE;
			return;
		}
		printf("Config::Can't match variable %s with anything in the file\n", name);
		configError = INCOMPLETE;
		strcpy(configTable[numEntries].label, name);
		configTable[numEntries].valuePtr = val;
		strncpy(val,defaultVal, 19);
		configTable[numEntries].type = STRING;
		numEntries++;
	}
	else{

		configTable[index].valuePtr = val;
		//sscanf(configTable[index].valueBuf, "%f", val);
		strncpy( val, configTable[index].valueBuf, 19);
		configTable[index].type = STRING;
	}
}
#endif

void Config::LinkAndSet(char *name, float *val, float defaultVal){
	int index;

	index = searchLabel(name);

	if(index == -1){
		if(numEntries >= (CONFIG_TABLE_SIZE - 1)){
			printf("Config::Table size exceeded\n");
			configError = TABLESIZE;
			return;
		}
		printf("Config::Can't match variable %s with anything in the file\n", name);
		configError = INCOMPLETE;
		strcpy(configTable[numEntries].label, name);
		configTable[numEntries].valuePtr = val;
		*val = defaultVal;
		configTable[numEntries].type = FLOAT;
		numEntries++;
	}
	else{
		configTable[index].valuePtr = val;
		sscanf(configTable[index].valueBuf, "%f", val);
		configTable[index].type = FLOAT;
	}
}

void Config::LinkAndSet(char *name, double *val, double defaultVal){
	int index;

	index = searchLabel(name);

	if(index == -1){
		if(numEntries >= (CONFIG_TABLE_SIZE - 1)){
			printf("Config::Table size exceeded\n");
			configError = TABLESIZE;
			return;
		}
		printf("Config::Can't match variable %s with anything in the file\n", name);
		configError = INCOMPLETE;
		strcpy(configTable[numEntries].label, name);
		configTable[numEntries].valuePtr = val;
		*val = defaultVal;
		configTable[numEntries].type = DOUBLE;
		numEntries++;
	}
	else{
		configTable[index].valuePtr = val;
		sscanf(configTable[index].valueBuf, "%lf", val);
		configTable[index].type = DOUBLE;
	}
}

void Config::LinkAndSet(char *name, int *val, int defaultVal){
	int index;

	index = searchLabel(name);

	if(index == -1){
		if(numEntries >= (CONFIG_TABLE_SIZE - 1)){
			printf("Config::Table size exceeded\n");
			configError = TABLESIZE;
			return;
		}

		printf("Config::Can't match variable %s with anything in the file\n", name);
		configError = INCOMPLETE;
		strcpy(configTable[numEntries].label, name);
		configTable[numEntries].valuePtr = val;
		*val = defaultVal;
		configTable[numEntries].type = INT;
		numEntries++;
	}
	else{
		configTable[index].valuePtr = val;
		sscanf(configTable[index].valueBuf, "%d", val);
		configTable[index].type = INT;
	}

}

int Config::GetError(){
	return configError;
}

int Config::Write(){
	FILE *fp;
	char buf[CONFIG_VALBUF_LEN];
	int i;
	int numEntriesWritten = 0;

	fp = fopen(filename, "w");

	if(fp == NULL){
		printf("Config::Error opening file\n");
		return -1;
	}

	checkSum = 0;
	for(i = 0; i < numEntries; i++, numEntriesWritten++){
		if(configTable[i].type == INT){
			sprintf(buf, "%d", *(int *)configTable[i].valuePtr);
		}
		else if(configTable[i].type == DOUBLE){
			sprintf(buf, "%lf", *(double *)configTable[i].valuePtr);
		}
		else if(configTable[i].type == FLOAT){
			sprintf(buf, "%lf", *(float *)configTable[i].valuePtr);
		}else if(configTable[i].type == STRING){
			sprintf(buf, "%s", (char *)configTable[i].valuePtr);
		}
		else{ // this entry is not linked with a variable, skip it
			numEntriesWritten--;
			continue;
		}

		fprintf(fp, "%s = %s\n", configTable[i].label, buf);

		updateChecksum(&checkSum, configTable[i].label, strlen(configTable[i].label));
		updateChecksum(&checkSum, buf, strlen(buf));

	}

	// write out the checksum
	fprintf(fp, "checksum = %d\n", checkSum);

	fflush(fp);

//	printf("Config::Successfully wrote %d entries to the file\n", numEntriesWritten);
	fclose(fp);

	return 1;

}

void Config::Dump(){
	for(int i=0; i < numEntries; i++){
		if(configTable[i].type == INT){
			printf("%s = %d [INT] file = %s\n", configTable[i].label,
												 *(int *)configTable[i].valuePtr,
												 		   configTable[i].valueBuf);
		}
		else if(configTable[i].type == DOUBLE){
			printf("%s = %lf [DOUBLE] file = %s\n", configTable[i].label,
												 *(double *)configTable[i].valuePtr,
												 		   configTable[i].valueBuf);
		}
		else if(configTable[i].type == FLOAT){
			printf("%s = %lf [FLOAT] file = %s\n", configTable[i].label,
												 *(float *)configTable[i].valuePtr,
												 		   configTable[i].valueBuf);
		}
		else{
		  	printf("%s [UNLINKED] file = %s\n", configTable[i].label,
												 		   configTable[i].valueBuf);
		}

	}
}


void Config::GetTypeAndIndexByName(char *name, int *type, int *index){

 	*index = searchLabel(name);
	if(*index != -1){
		*type = configTable[*index].type;
		return;
	}
	else{
		return;
	}
}

void Config::ChangeValueByIndex(int index, float val){
	*(float *)configTable[index].valuePtr = val;
}

void Config::ChangeValueByIndex(int index, double val){
	*(double *)configTable[index].valuePtr = val;
}

void Config::ChangeValueByIndex(int index, int val){
	*(int *)configTable[index].valuePtr = val;
}

float Config::GetFloatValueByIndex(int index){
	return *(float *)configTable[index].valuePtr;
}

double Config::GetDoubleValueByIndex(int index){
	return *(double *)configTable[index].valuePtr;
}

int Config::GetIntValueByIndex(int index){
	return *(int *)configTable[index].valuePtr;
}

