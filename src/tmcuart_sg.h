#ifndef __TMCUART_SG_H
#define __TMCUART_SG_H

#include <stdint.h>

#define TU_SG_INVALID 0xffff

static inline uint8_t
tmcuart_sg_crc(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint_fast8_t i = 0; i < len; i++) {
        uint8_t b = data[i];
        for (uint_fast8_t j = 0; j < 8; j++, b >>= 1)
            crc = (crc << 1) ^ (((crc >> 7) ^ (b & 1)) ? 0x07 : 0);
    }
    return crc;
}

static inline void
tmcuart_sg_request(uint8_t addr, uint8_t *out)
{
    uint8_t frame[4] = {0xf5, addr, 0x41, 0};
    frame[3] = tmcuart_sg_crc(frame, 3);
    uint64_t encoded = 0;
    for (uint_fast8_t i = 0; i < 4; i++)
        encoded |= (uint64_t)((frame[i] << 1) | 0x200) << (10 * i);
    for (uint_fast8_t i = 0; i < 5; i++)
        out[i] = encoded >> (8 * i);
}

// Проверяем длину, старт/стоп, заголовок, диапазон SG и CRC.
static inline uint16_t
tmcuart_sg_decode(const uint8_t *data, uint8_t len)
{
    if (len != 10)
        return TU_SG_INVALID;
    uint8_t frame[8];
    for (uint_fast8_t i = 0; i < 8; i++) {
        uint_fast8_t bit = i * 10, pos = bit / 8;
        uint16_t word = data[pos] | ((uint16_t)data[pos+1] << 8);
        word >>= bit & 7;
        if ((word & 0x201) != 0x200)
            return TU_SG_INVALID;
        frame[i] = word >> 1;
    }
    if (frame[0] != 0x05 || frame[1] != 0xff || frame[2] != 0x41
        || frame[3] || frame[4] || frame[5] > 3
        || tmcuart_sg_crc(frame, 7) != frame[7])
        return TU_SG_INVALID;
    return ((uint16_t)frame[5] << 8) | frame[6];
}

// Шесть байт в little endian, без требований к выравниванию.
static inline void
tmcuart_sg_store(uint8_t *data, uint32_t clock, uint16_t sg)
{
    for (uint_fast8_t i = 0; i < 4; i++)
        data[i] = clock >> (8 * i);
    data[4] = sg;
    data[5] = sg >> 8;
}

#endif
