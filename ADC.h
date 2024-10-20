#include "i2c.h"

void ADC_setChannel (uint8_t channel);      // Set the ADV input (0-5)
void ADC_setReference(uint8_t Vref);        // Set the ADV voltage reference
void ADC_Init(void);                        // Initialize the ADC
uint16_t ADC_getValue(void);                // Initiate conversion, return result

#define Vref01 1.1