#include "calc_cycles.h"

int32_t calculate_cycles(int16_t *buffer, uint32_t length) {
    int32_t crossings = 0;
    if (length < 2) {
        return 0;
    }
    for (uint32_t i = 1; i < length; i++) {
        if ((buffer[i-1] < 0 && buffer[i] >= 0) || (buffer[i-1] > 0 && buffer[i] <= 0)) {
            crossings++;
        }
    }
    return (crossings + 1) / 2; // Half-crossings to full cycles
}