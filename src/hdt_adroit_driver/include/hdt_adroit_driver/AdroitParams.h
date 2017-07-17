/*----------------------------------------------------------------------------
 * Name:    AdroitParams.h
 * Purpose: parameter handling
 * Note(s):
 *----------------------------------------------------------------------------*/

#pragma once

#ifndef AdroitParams_h
#define AdroitParams_h

#include <string>
#include <vector>
#include <stdint.h>
#include "tinyxml.h"

#define NUM_FLASH_PARAMETERS	512

int get_param_type(std::string name);
std::string param_to_string(uint32_t *data, uint8_t type);

typedef enum ParameterType {
	INT_TYPE		= 0,
	FLOAT_TYPE		= 1,
	ARRAY_TYPE		= 2
} ParameterType;

class AdroitParams {
public:
	AdroitParams();
	~AdroitParams();

	bool load_file(std::string params_file);
	int get_index_by_name(std::string name);
	int get_type_by_name(std::string name);
	int get_type_by_index(int index);
	std::string get_name_by_index(int index);
	bool params_to_xml(std::vector<uint32_t> param_data, std::string file_name);
	bool xml_to_params(std::vector<uint32_t> &param_data, std::string file_name);

	std::vector<std::string> param_name;
	std::vector<std::string> param_type;
private:
};

#endif // AdroitParams_h
