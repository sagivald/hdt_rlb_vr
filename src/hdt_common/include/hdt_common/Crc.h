#ifndef __CRC_H
#define __CRC_H

#include <stdint.h>
#include <vector>

// crc class
class Crc {
public:
	Crc();
	~Crc() {};
	
	uint32_t crc32(uint8_t *buf, uint32_t len);
	uint32_t crc32(std::vector<unsigned char> buf);
private:
	uint32_t crc_table[256];
};

#endif

