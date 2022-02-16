#include "common.h"

bool hex_to_uint8(uint8_t a, uint8_t *res)
{
    uint8_t nibble_low = a & 0xf;
    switch (a & 0xf0) {
        case 0x30: // 0
            if (nibble_low > 9) {
                return false;
            }
            *res = nibble_low;
            break;
        case 0x40:  // A-
        case 0x60: // a-
            if (nibble_low == 0 || nibble_low > 6) {
                return false;
            }
            *res = nibble_low + 9;
            break;
        default:
            return false;
    }
    return true;

}
