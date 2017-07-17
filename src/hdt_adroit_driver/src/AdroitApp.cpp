/*----------------------------------------------------------------------------
 * Name:    AdroitApplication.cpp
 * Purpose: application handling
 * Note(s):
 *----------------------------------------------------------------------------*/

#include <sstream>
#include <iostream>
#include "AdroitApp.h"

/*----------------------------------------------------------------------------
  constructor
 *----------------------------------------------------------------------------*/
AdroitApp::AdroitApp() {
}

/*----------------------------------------------------------------------------
  destructor
 *----------------------------------------------------------------------------*/
AdroitApp::~AdroitApp() {
}

/*----------------------------------------------------------------------------
  read file
 *----------------------------------------------------------------------------*/
int AdroitApp::read_hex_file(std::string file_name) {
	// open hex file
	app_file.open(file_name.c_str(), std::ios::in);
	
	// check for valid file handle
	if(!app_file) {
		return 0;
	}

	// read file into hex data vector
	std::string s;
	if(app_file.is_open()) {
		while(app_file.good()) {
			std::getline(app_file, s);
			//std::cout << s << std::endl;

			// check fo rempty line
			if(s.empty()) continue;

			// add to hex data vector
			hex_data.push_back(s);
		}
	}

	// close hex file
	app_file.close();

	// check for failures
	if(app_file.eof() == false) {
		printf("File read failure\n");
		return 0;
	}

	return 1;
}

/*----------------------------------------------------------------------------
  convert intel hex file to binary
 *----------------------------------------------------------------------------*/
int AdroitApp::hex_to_bin(void) {
	uint32_t upper_address = 0;
	uint32_t current_address = 0;
	uint32_t prev_address = 0;
	bool first_address = true;

#ifdef APP_HEADER_ENABLE
	// add application header
	for(int i = 0; i < APP_HEADER_SIZE; i++) {
		bin_data.push_back(0x00);
	}
#endif

	// parse binary data from hex file line by line
	for(std::vector<int>::size_type i = 0; i != hex_data.size(); i++) {
		int checksum_result = 0;
		int byte_count, record_type, address;
		std::vector<unsigned char> line_data;

		//std::cout << hex_data[i]<< std::endl;

		// check first character for :
		if(hex_data[i].at(0) != ':') continue;

		// copy data from line
		for(unsigned int j = 1; j < hex_data[i].size()-1; j=j+2) {
			std::stringstream ss_data;
			int byte_data;

			// read data
			ss_data << std::hex << hex_data[i].substr(j, 2);
			ss_data >> byte_data;
			//std::cout << byte_data << std::endl;
			
			// add byte to vector
			line_data.push_back((unsigned char)byte_data);
			
			// update checksum
			checksum_result += byte_data;
		}
		//std::cout<<checksum_result<<std::endl;
		// check checksum
		if((checksum_result & 0xFF) != 0x00) {
			std::cout << "Checksum mismatch, check input file" << std::endl;
			return 0;
		}

		// set data
		byte_count = line_data[0];
		address = (line_data[1] << 8) | line_data[2];
		record_type = line_data[3];

		// handle record type
		switch(record_type) {
		case(0):
			current_address = upper_address | address;
			// check if first address
			if(first_address == false) {
				// check for new memory address
				if(prev_address != current_address) {
					// pad binary data as necessary
					for(uint32_t j = 0; j < (current_address - prev_address); j++) {
						bin_data.push_back((unsigned char)0xFF);
					}
				}
			}
			else {
				first_address = false;
			}

			// add to binary data
			for(int j = 0; j < byte_count; j++) {
				bin_data.push_back(line_data[4 + j]);
			}
			prev_address = current_address + byte_count;
			break;
		case(4):
			// update upper address
			upper_address = (line_data[4] << 8 | line_data[5]) << 16;
			break;
		default:
			break;
		}
	}

	// pad to be evenly divisible by NUM_BYTES_IN_WORD
	uint32_t rem = bin_data.size() % NUM_BYTES_IN_WORD;
	if(rem > 0) {
		for(uint32_t i = 0; i < NUM_BYTES_IN_WORD - rem; i++) {
			bin_data.push_back((unsigned char)0xFF);
		}
	}

	return 1;
}
