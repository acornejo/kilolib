#ifndef __RAND_H__
#define __RAND_H__

static uint8_t seed=0xaa, accumulator = 0;

uint8_t rand() {
    seed ^= seed<<3;
    seed ^= seed>>5;
    seed ^= accumulator++>>2;
    return seed;
}

#endif//__RAND_H__
