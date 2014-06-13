/** @internal */

typedef struct {
    uint8_t page_address;
    uint8_t page_offset;
    uint16_t word1;
    uint16_t word2;
    uint16_t word3;
    uint8_t unused;
} bootmsg_t;
