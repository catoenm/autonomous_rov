#include "crc32b.h"
#include <stdint.h>
#include <stddef.h>

// http://www.hackersdelight.org/hdcodetxt/crc.c.txt
uint32_t crc32b(uint8_t *message, size_t size) {
   int i, j;
   uint8_t byte;
   uint32_t crc, mask;

   i = 0;
   crc = 0xFFFFFFFF;
   while (i < size) {
      byte = message[i];            // Get next byte.
      crc = crc ^ byte;
      for (j = 7; j >= 0; j--) {    // Do eight times.
         mask = -(crc & 1);
         crc = (crc >> 1) ^ (0xEDB88320UL & mask);
      }
      i = i + 1;
   }
   return ~crc;
}
