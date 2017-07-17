/*----------------------------------------------------------------------------
 * Name:    AdroitParameters.cpp
 * Purpose: parameter handling
 * Note(s):
 *----------------------------------------------------------------------------*/

#include <sstream>
#include "AdroitParams.h"
#include "AdroitCrc.h"

/*----------------------------------------------------------------------------
  get parameter type from string
 *----------------------------------------------------------------------------*/
int get_param_type(std::string name) {
	int type = INT_TYPE;

	if(name == "INT") {
		type = INT_TYPE;
	}
	else if(name == "FLOAT") {
		type = FLOAT_TYPE;
	}

	return type;
}

/*----------------------------------------------------------------------------
  display parameter
 *----------------------------------------------------------------------------*/
std::string param_to_string(uint32_t *data, uint8_t type) {
	std::stringstream ss;

	switch(type) {
	case(INT_TYPE): {
		int int_data;
		memcpy(&int_data, data, sizeof(int_data));
		ss << int_data;
		//printf("%d", int_data);
		break;
		}
	//case(HEX_TYPE): {
	//	uint32_t hex_data;
	//	memcpy(&hex_data, data, sizeof(hex_data));
	//	//printf("%08X", hex_data);
	//	break;
	//	}
	case(FLOAT_TYPE): {
		float float_data;
		memcpy(&float_data, data, sizeof(float_data));
		//ss.precision(7);
		ss << std::fixed << float_data;
		//printf("%f", float_data);
		break;
		}
	default:
		break;
	}

	return ss.str();
}

/*----------------------------------------------------------------------------
  read params from xml file
 *----------------------------------------------------------------------------*/
bool AdroitParams::xml_to_params(std::vector<uint32_t> &param_data, std::string file_name) {
	TiXmlDocument doc(file_name.c_str());
	bool loadOkay = doc.LoadFile();

	// parse parameters xml file
	if(loadOkay) {
		TiXmlElement *param;
		param = doc.FirstChildElement("ParameterConfiguration")->FirstChildElement("Parameter");
		for(param; param; param = param->NextSiblingElement("Parameter")) {
			int index = atoi(param->Attribute("Index"));
			//printf("index = %d\r\n", index);
			std::string name = param->FirstChildElement("Name")->GetText();
			std::string type = param->FirstChildElement("Type")->GetText();
			std::string val = param->FirstChildElement("Value")->GetText();

			// check that xml data matches params.xml
			if((name.compare(param_name[index]) == 0) && (type.compare(param_type[index]) == 0)) {
				uint32_t data;
				// get param data by type
				switch(get_param_type(type)) {
				case(INT_TYPE):
				case(ARRAY_TYPE): {
					int int_data;
					std::istringstream(val) >> int_data;
					memcpy(&data, &int_data, sizeof(data));
					param_data.push_back(data);
					break;
					}
				case(FLOAT_TYPE): {
					float float_data;
					std::istringstream(val) >> float_data;
					memcpy(&data, &float_data, sizeof(data));
					param_data.push_back(data);
					break;
					}
				// not a valid type
				default:
					return false;
					break;
				}
			}
			// name or type did not match
			else {
				return false;
			}
		}
	}

	return true;
}

/*----------------------------------------------------------------------------
  write params to xml file
 *----------------------------------------------------------------------------*/
bool AdroitParams::params_to_xml(std::vector<uint32_t> param_data, std::string file_name) {
	TiXmlDocument doc;  
	TiXmlDeclaration *decl = new TiXmlDeclaration("1.0", "UTF-8", "yes");  
	doc.LinkEndChild(decl);  
 
	TiXmlElement *root = new TiXmlElement("ParameterConfiguration");
	root->SetAttribute("xmlns:xsi", "http://www.w3.org/2001/XMLSchema-instance");
	doc.LinkEndChild(root);

	// write all parameters
	for(int i = 0; i < NUM_FLASH_PARAMETERS; i++) {
		std::string data = param_to_string(&param_data[i], get_param_type(param_type[i]));
		std::stringstream ss;
		ss << i;

		// don't write if name or type unknown
		if(param_name[i].empty() || param_type[i].empty()) continue;

		// set parameter element
		TiXmlElement *param = new TiXmlElement("Parameter");
		param->SetAttribute("Index", ss.str().c_str());
		root->LinkEndChild(param);

		// set name element
		TiXmlElement *name = new TiXmlElement("Name");
		name->LinkEndChild(new TiXmlText(param_name[i].c_str()));
		param->LinkEndChild(name);

		// set type element
		TiXmlElement *type = new TiXmlElement("Type");
		type->LinkEndChild(new TiXmlText(param_type[i].c_str()));
		param->LinkEndChild(type);

		// set value element
		ss.str(std::string());
		ss << param_data[i];
		TiXmlElement *val = new TiXmlElement("Value");
		val->LinkEndChild(new TiXmlText(data.c_str()));
		param->LinkEndChild(val);
	}

	doc.SaveFile(file_name.c_str()); 

	return true;
}

/*----------------------------------------------------------------------------
  constructor
 *----------------------------------------------------------------------------*/
AdroitParams::AdroitParams() {
}

/*----------------------------------------------------------------------------
  load params
 *----------------------------------------------------------------------------*/
bool AdroitParams::load_file(std::string params_file) {
	TiXmlDocument doc(params_file);
	bool loadOkay = doc.LoadFile();

	param_name.assign(NUM_FLASH_PARAMETERS, "");
	param_type.assign(NUM_FLASH_PARAMETERS, "");

	// parse parameters xml file
	if(loadOkay) {
		TiXmlElement *param;
		param = doc.FirstChildElement("ParameterConfiguration")->FirstChildElement("Parameter");
		for(param; param; param = param->NextSiblingElement("Parameter")) {
			int index = atoi(param->Attribute("Index"));
			std::string name = param->FirstChildElement("Name")->GetText();
			std::string type = param->FirstChildElement("Type")->GetText();

			param_name[index] = name;
			param_type[index] = type;
		}
	}

	return loadOkay;
}

/*----------------------------------------------------------------------------
  destructor
 *----------------------------------------------------------------------------*/
AdroitParams::~AdroitParams() {
}

/*----------------------------------------------------------------------------
  get index by name
 *----------------------------------------------------------------------------*/
int AdroitParams::get_index_by_name(std::string name) {
	for(int i = 0; i < NUM_FLASH_PARAMETERS; i++) {
		if(name.compare(param_name[i]) == 0) {
			return i;
		}
	}
	
	return -1;
}

/*----------------------------------------------------------------------------
  get type by name
 *----------------------------------------------------------------------------*/
int AdroitParams::get_type_by_name(std::string name) {
	for(int i = 0; i < NUM_FLASH_PARAMETERS; i++) {
		if(name.compare(param_name[i]) == 0) {
			return get_param_type(param_type[i]);
		}
	}
	
	return INT_TYPE;
}

/*----------------------------------------------------------------------------
  get type by index
 *----------------------------------------------------------------------------*/
int AdroitParams::get_type_by_index(int index) {
	if((index >= 0) && (index < NUM_FLASH_PARAMETERS)) {
		return get_param_type(param_type[index]);
	}

	return INT_TYPE;
}

/*----------------------------------------------------------------------------
  get name by index
 *----------------------------------------------------------------------------*/
std::string AdroitParams::get_name_by_index(int index) {
	if((index >= 0) && (index < NUM_FLASH_PARAMETERS)) {
		return param_name[index];
	}

	return "";
}
