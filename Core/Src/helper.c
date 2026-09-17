#include "helper.h"

/* num2buff_LE():
FUNCTION:
    Takes a 32 bit number and parses the bytes into a given buffer in Little Endian format. 
    i.e. index 0 of the buffer is LSB of the number.
DEFS:
    num:      32 bit number
    buff:     byte buffer of >=4 bytes
RETURNS:
    original pointer to buffer
*/
// uint8_t* num2buff_LE(uint32_t num, uint8_t* buff){
//     *(uint32_t*)buff = num;
//     return buff;
// }

/* num2buff_BE():
FUNCTION:
    Takes a 32 bit number and parses the bytes into a given buffer in Big Endian format. 
   i.e. index 0 of the buffer is MSB of the number.
DEFS:
    num:      32 bit number
    buff:     byte buffer of >=4 bytes
RETURNS:
    original pointer to buffer
*/
uint8_t* num2buff_BE(uint32_t num, uint8_t* buff){
    for (int i = 3U; i >= 0U; i--)
        *buff++ = (num >> i*8U) & 0xFF;
    return buff - 4U;
}

/* buff2num_LE():
FUNCTION:
    Takes a 4 byte buffer in Little Endian format and concatenates the bytes into a 32 bit number.
    i.e. index 0 of the buffer is LSB of the number.
DEFS:
    buff:     byte buffer of >=4 bytes

*/
// uint32_t buff2num_LE(uint8_t* buff){
//     return *(uint32_t*)buff;
// }

/* buff2num_BE():
FUNCTION:
    Takes a 4 byte buffer in Big Endian format and concatenates the bytes into a 32 bit number.
    i.e. index 0 of the buffer is MSB of the number.
DEFS:
    buff:     byte buffer of >=4 bytes
*/
uint32_t buff2num_BE(uint8_t* buff){
    uint32_t result = 0U;
    for (int i = 3U; i >= 0U; i--)
        result |= *buff++ << i*8U;
    return result;
}

/* time_add():
FUNCTION:
    Adds together two time structs, a and b, with wrap around (mod 2^40)
    Returns (a + b) % 2^40
DEFS:
    a:      TIME object a, struct containing an upper uint8_t and a lower uint32_t
    b:      TIME object b, struct containing an upper uint8_t and a lower uint32_t
*/
TIME time_add(TIME a, TIME b) {
    TIME result;
    uint8_t carry_one = 0U;
    result.BOT4 = a.BOT4 + b.BOT4;
    carry_one   = result.BOT4 < MAX(a.BOT4, b.BOT4);
    result.TOP1 = a.TOP1 + b.TOP1 + carry_one;
    return result;
}

/* time_sub():
FUNCTION:
    Subtracts time b from time a with wrap around (mod 2^40)
    Returns (a - b) % 2^40
DEFS:
    a:      TIME object a, struct containing an upper uint8_t and a lower uint32_t
    b:      TIME object b, struct containing an upper uint8_t and a lower uint32_t
*/
TIME time_sub(TIME a, TIME b) {
    TIME one;
      b.TOP1 ^=  0xFF; b.BOT4 ^= ~0U; // not
    one.TOP1  =  0U; one.BOT4  =  1U;
    b = time_add(b, one);
    return time_add(a, b);
}

/* argMedian()
FUNCTION:
    Returns the index of the median of three input values
DEFS:
    a:      float at index 0
    b:      float at index 1
    c:      float at index 2
*/
uint8_t argMedian(float a, float b, float c) {
    if ((a > b) ^ (a > c))
        return 0;
    else if ((b < a) ^ (b < c))
        return 1;
    else
        return 2;
}