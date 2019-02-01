#ifndef SHARED_UTILS
#define SHARED_UTILS

static byte const first1BitperNib[] = {4,0,1,0,2,0,1,0,3,0,1,0,2,0,3,0};
// gets the index of the least significant high bit of a byte
#define FIRST1BIT(x) ((x&0x0F) == 0 ? first1BitperNib[(x)>>4] + 4 : first1BitperNib[(x)&0xF])
#define FIRST0BIT(x) ((x&0x0F) == 0x0F ? first1BitperNib[(^x)>>4] + 4 : first1BitperNib[(^x)&0xF])

static byte const bitsPerNib[] = {0,1,1,2,1,2,2,3,1,2,2,3,2,3,3,4};
#define BITCOUNT(x) (bitsPerNib[(x)&0xF] + bitsPerNib[(x)>>4])

static byte const bitMask[] = { 1, 2, 4, 8, 16, 32, 64, 128 };
#define GET_BITMASK(x) bitMask[x]

// swaps the nibbles of a byte e.g turns 0xF1 into 0x1F
#define swap(value) asm("swap %0" : "=r" (value) : "0" (value)) 

#endif
