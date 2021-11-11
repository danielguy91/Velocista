/*
 * Util.h
 *
 *  Created on: 10 nov. 2021
 *      Author: dany_
 */

#ifndef UTIL_H_
#define UTIL_H_
typedef struct {
	uint8_t *buf;
	uint8_t iW;
	uint8_t iR;
	uint8_t header;
	uint16_t timeout;
	uint8_t nBytes;
	uint8_t iData;
	uint8_t cks;
}__attribute__((packed, aligned(1))) _rx;

typedef struct {
	uint8_t *buf;
	uint8_t iW;
	uint8_t iR;
	uint8_t cks;
}__attribute__((packed, aligned(1))) _tx;

typedef union {
	uint8_t u8[4];
	int8_t i8[4];
	uint16_t u16[2];
	int16_t  i16[2];
	uint32_t u32;
	int32_t i32;
	float   f;
}_sWork;

// typedef
typedef union{
	struct{
		uint8_t b0: 1;
		uint8_t b1: 1;
		uint8_t b2: 1;
		uint8_t b3: 1;
		uint8_t b4: 1;
		uint8_t b5: 1;
		uint8_t b6: 1;
		uint8_t b7: 1;
	}bit;
	uint8_t byte;
}_sFlag;

typedef union{
    uint16_t value;
    uint8_t v[2];
}_unionNd;



#endif /* UTIL_H_ */
