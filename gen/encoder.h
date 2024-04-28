
/* 16 bit value:
31 - on/off
30-27 - power (4 bit)

7-4 - go (4 bit)
3-0 - turn (4 bit)
*/
#include <stdbool.h>


uint16_t encode(bool power);
