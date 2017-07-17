/*----------------------------------------------------------------------------
 * Name:    AdroitApp.h
 * Purpose: application handling
 * Note(s):
 *----------------------------------------------------------------------------*/

#pragma once

#ifndef AdroitApp_h
#define AdroitApp_h

#include <string>
#include <vector>
#include <fstream>
#include <stdint.h>

#define NUM_BYTES_IN_WORD		sizeof(uint32_t)
#define	APP_HEADER_SIZE			0x100
#define APP_HEADER_ENABLE

class AdroitApp {
public:
	AdroitApp();
	~AdroitApp();
	int read_hex_file(std::string file_name);
	int hex_to_bin(void);
	
	std::vector<unsigned char> bin_data;
private:
	char *file_name;
	std::ifstream app_file;
	std::vector<std::string> hex_data;
};

#endif // AdroitApp_h
