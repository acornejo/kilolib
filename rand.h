#ifndef __RAND_H__
#define __RAND_H__

static uint32_t x=132456789, y=362436069, z=521288629;

uint32_t xorshift96()
{
    uint32_t t;

	x ^= x << 16;
	x ^= x >> 5;
	x ^= x << 1;

	t = x;
	x = y;
	y = z;
	z = t ^ x ^ y;

	return z;
}

void randSeed(uint32_t seed) {
	x=seed;
}

uint8_t irand8(uint8_t minval, uint8_t maxval) {
	return (uint8_t) ((((uint16_t)(xorshift96()&0xFF)) * (maxval-minval))/256) + minval;
}

uint16_t irand16(uint16_t minval, uint16_t maxval) {
	return (uint16_t) ((((xorshift96()>>16) * (maxval-minval))>>16) + minval);
}

uint8_t urand8(uint8_t maxval) {
	return (uint8_t) ((((uint16_t) (xorshift96()&0xFF)) * maxval)/256);
}

uint16_t urand16(uint16_t maxval) {
	return (uint16_t) (((xorshift96()>>16) * maxval)>>16);
}

uint8_t rand8() {
	return (uint8_t)(xorshift96()&0xFF);
}

uint16_t rand16() {
	return (uint16_t)(xorshift96()>>16);
}

// static uint8_t seed=0xaa, accumulator = 0;
//
// uint8_t rnd(void) {
//         seed ^= seed<<3;
//         seed ^= seed>>5;
//         seed ^= accumulator++>>2;
//         return seed;
// }
#endif//__RAND_H__
