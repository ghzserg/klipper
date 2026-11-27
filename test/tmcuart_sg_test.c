#include <assert.h>
#include <string.h>
#include "tmcuart_sg.h"

int main(void)
{
    static const uint8_t requests[4][5] = {
        {0xea, 0x03, 0x28, 0x28, 0xc4},
        {0xea, 0x0b, 0x28, 0x28, 0x9f},
        {0xea, 0x13, 0x28, 0xa8, 0xe9},
        {0xea, 0x1b, 0x28, 0xa8, 0xb2}
    };
    static const uint8_t replies[5][10] = {
        {0x0a, 0xfa, 0x2f, 0x28, 0x80, 0x00, 0x02, 0x08, 0xa0, 0xeb},
        {0x0a, 0xfa, 0x2f, 0x28, 0x80, 0x00, 0x02, 0x28, 0x20, 0xaf},
        {0x0a, 0xfa, 0x2f, 0x28, 0x80, 0x00, 0x02, 0xe8, 0x3f, 0x92},
        {0x0a, 0xfa, 0x2f, 0x28, 0x80, 0x00, 0x0a, 0x08, 0xa0, 0xb0},
        {0x0a, 0xfa, 0x2f, 0x28, 0x80, 0x00, 0x1a, 0xe8, 0xbf, 0xe4}
    };
    static const uint16_t values[] = {0, 1, 255, 256, 1023};
    uint8_t data[10];
    for (unsigned i = 0; i < 4; i++) {
        tmcuart_sg_request(i, data);
        assert(!memcmp(data, requests[i], 5));
    }
    for (unsigned i = 0; i < 5; i++) {
        assert(tmcuart_sg_decode(replies[i], 10) == values[i]);
        for (unsigned len = 0; len < 10; len++)
            assert(tmcuart_sg_decode(replies[i], len) == TU_SG_INVALID);
        for (unsigned bit = 0; bit < 80; bit++) {
            memcpy(data, replies[i], 10);
            data[bit/8] ^= 1 << (bit%8);
            assert(tmcuart_sg_decode(data, 10) == TU_SG_INVALID);
        }
    }
    uint8_t record[8] = {0x55};
    record[7] = 0xaa;
    tmcuart_sg_store(&record[1], 0xfffffff0u, 0x03ff);
    const uint8_t expected[] = {0x55, 0xf0, 0xff, 0xff, 0xff, 0xff, 3, 0xaa};
    assert(!memcmp(record, expected, sizeof(record)));
    tmcuart_sg_store(&record[1], 0, TU_SG_INVALID);
    assert(record[5] == 0xff && record[6] == 0xff);
    return 0;
}
