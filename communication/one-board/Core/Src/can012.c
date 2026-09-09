/*********************************************************/
/***** CAN Communication practice - frame 0x012/0x0A2 *****/
/*********************************************************/

#include "can012.h"
#include <string.h>

uint8_t Can012_Checksum(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00u;

    for (int i = (int)len - 1; i >= 0; i--)
    {
        crc ^= data[i];
        for (uint8_t b = 0u; b < 8u; b++)
        {
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x1Du)
                                : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

void Can012_BuildFrame(uint8_t value0, uint8_t value1, uint8_t out[CAN012_DLC])
{
    memset(out, 0x00, CAN012_DLC);

    out[0] = value0;
    out[1] = value1;
    out[2] = (uint8_t)(value0 + value1);          /* wraps modulo 256 by design */
    out[6] = Can012_Checksum(out, 6u);             /* covers byte0..5 */
}
