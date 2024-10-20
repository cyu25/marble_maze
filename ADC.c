#include "ADC.h"

void ADC_setChannel (uint8_t channel){                  // Set the ADV input (0-5)
    ADMUX  &= 0xF0;  
    ADMUX |= channel;
}    

void ADC_setReference(uint8_t Vref){                    // Set the ADV voltage reference
    if (Vref == 5){
        ADMUX &= ~((1<<REFS0)|(1<< REFS1));
        ADMUX |= (1 << REFS0);  
    }
    else if (Vref == Vref01){
        ADMUX &= ~((1<<REFS0)|(1<< REFS1));
        ADMUX |= ((1 << REFS0) | (1 << REFS1));  
    }
}  

void ADC_Init(void){                                    // Initialize the ADC
    ADC_setReference(5);       
    ADCSRA |= (1 << ADPS0)|(1 << ADPS1)|(1 << ADPS2);   // Prescale by 128
    ADCSRA |= (1 << ADEN);                              // Enable ADC
}        



uint16_t ADC_getValue(void){                            // Initiate conversion, return result
    uint16_t value;
    ADCSRA |= (1 << ADSC);                               // Start conversion
    while ((ADCSRA & (1 << ADSC)) != 0){}                // Wait for completion
    value = ADC;                                         // Read the result
    return value;
}              