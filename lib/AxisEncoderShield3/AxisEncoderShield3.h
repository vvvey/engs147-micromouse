// Refactoring of Robogaia.com encoder example
// for inclusion in ENGS 147 labs and projects
// Original: Robogaia.com (now defunct)
// Modified by: M. Kokko
// 27-Mar-2025
//
#include <SPI.h>

#define CHIP_SEL_PIN_1 10
#define CHIP_SEL_PIN_2 9
#define CHIP_SEL_PIN_3 8

void initEncoderShield(void);
long getEncoderValue(int encoder);
void selectEncoder(int encoder);
void deselectEncoder(int encoder);
void LS7366_Init(void);
