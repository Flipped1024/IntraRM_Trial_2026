#ifndef PC_PROTOCOL_H
#define PC_PROTOCOL_H
#include <stdint.h>
#include <string.h>
enum { KEY_W, KEY_A, KEY_S, KEY_D, KEY_UP, KEY_DOWN, KEY_LEFT,
       KEY_RIGHT, KEY_FIRE, KEY_FRICTION, KEY_Q, KEY_E, KEY_COUNT };
static uint8_t pc_crc8(const uint8_t *p, unsigned n)
{
    uint8_t crc = 0;
    while (n--) {
        crc ^= *p++;
        for (unsigned b = 0; b < 8; b++)
            crc = (uint8_t)((crc & 128) ? (crc << 1) ^ 7 : crc << 1);
    }
    return crc;
}
static void pc_build_frame(uint8_t f[14], uint8_t sequence, int enabled,
                           int amplitude, const int keys[KEY_COUNT])
{
    memset(f, 0, 14);
    f[0] = 0xC3; f[1] = 0x3C; f[2] = sequence;
    if (enabled) {
        f[3] = 3;
        f[4] = (uint8_t)((keys[KEY_D] - keys[KEY_A]) * amplitude);
        f[5] = (uint8_t)((keys[KEY_S] - keys[KEY_W]) * amplitude);
        f[6] = (uint8_t)((keys[KEY_RIGHT] - keys[KEY_LEFT]) * 127);
        f[7] = (uint8_t)((keys[KEY_DOWN] - keys[KEY_UP]) * 127);
        f[9] = keys[KEY_FIRE] ? 255 : 0;
        f[11] = (keys[KEY_FRICTION] ? 1 : 0) |
                (keys[KEY_Q] ? 0x40 : 0) | (keys[KEY_E] ? 0x80 : 0);
    }
    f[13] = pc_crc8(f, 13);
}
#endif
