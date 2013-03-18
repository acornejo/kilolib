#ifndef __BITFIELD_H__
#define __BITFIELD_H__

#define BF_create(NAME, SIZE)\
    unsigned char NAME##_data[(SIZE>>3)] = {0};\
    static const unsigned int NAME##_size = (SIZE>>3)

#define BF_init(NAME) {\
    int i=0;\
    for (i=0; i<NAME##_size; i++) NAME##_data[i]=0;\
}

#define BF_set(NAME, IDX) NAME##_data[IDX>>3] |= (1<<(IDX&7))

#define BF_clr(NAME, IDX) NAME##_data[IDX>>3] &= ~(1<<(IDX&7))

#define BF_get(NAME, IDX) (NAME##_data[IDX>>3] & (1<<(IDX&7)))

#define BF_capacity(NAME) NAME##_size

#define BF_clear(NAME) {\
    int i=0;\
    for (i=0; i<NAME##_size; i++) NAME##_data[i]=0;\
}

#endif//__BITFIELD_H__
