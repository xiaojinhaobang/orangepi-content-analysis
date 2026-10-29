#ifndef HEIGHT_DIAG_CODEC_H
#define HEIGHT_DIAG_CODEC_H
#include <stdint.h>
#include <stddef.h>

static int32_t height_diag_s32(const uint8_t *p)
{
    uint32_t u = (uint32_t)p[0] | ((uint32_t)p[1] << 8)
               | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    return (u & 0x80000000u) ? (int32_t)((int64_t)u - 4294967296LL) : (int32_t)u;
}

static int decode_height_diag(const uint8_t *payload, size_t size, int32_t out[3])
{
    if (!payload || !out || size != 9) return 0;
    out[0] = height_diag_s32(payload);
    out[1] = height_diag_s32(payload + 4);
    out[2] = payload[8];
    return 1;
}
#endif
