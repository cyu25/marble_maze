/* Engs 28, Embedded Systems
*
* Name: Claire Yu and Ava Rosenbaum
* Assignment: Lab9
*
* Program name: Lab9
* Date created: February 29, 2024
* Description: An embedded controller is designed for a two axis marble maze. 
*
* Dependencies: ioE28.c, SevenSeg.c, ADC.c, i2c.c and makefile
*
* I/O pins: 
* Potentiometer A0 and A1 INPUT
* Seven-Seg display - OUTPUT (SDA/SCL)
* Joystick button - D2 INPUT
* Start/Stop Photosensor B4 B5 INPUT 
* Motor control B0 B1 B2 B3 OUTPUT
* LEDS D3 D4 OUTPUT 
*
* Revisions: (use this to track the evolution of your program)
* Developed a paper design
* Implemented design into code
* Debugged:
* Stepper Motor functions created
* 
*/
#include <avr/io.h>
#include "SevenSeg.h"
#include "i2c.h"
#include "ADC.h"
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>				// for abs()
#include <math.h>				// for round()

#define MID 					512  		// middle for the ADC value
#define FORWARD 				1
#define BACKWARD				-1
#define MAX_TIMER_VALUE       	60         // 60s timer
#define MAX_ADC 				1024
#define LIMIT					80
#define DEADZONE				12

void timer1_init(uint16_t timeout); // pings every second
void greenLED(void);
void redLED(void);
void clearLEDs(void);
void initPinChangeInterrupt(void); // joystick button pushed
void stepperMoveX(int16_t steps, int8_t direction);
void stepperMoveY(int16_t steps, int8_t direction); 
void timer0_init(uint16_t timeout);	// Pings when it's time to update
void moveMotors(void);	// gets ADC values from joystick and moves the motors accordingly

volatile int8_t direction = FORWARD;
volatile uint8_t timerFlag1 = 0;	
volatile uint8_t timerFlag0 = 0;	
volatile uint8_t buttonPushed = 0;

typedef enum {WON, LOST, NEITHER, OFF} winstate_t ;
typedef enum {TRUE, FALSE} state_t ;
volatile state_t pushbuttonState = FALSE;
volatile winstate_t gameResult = NEITHER;
volatile state_t startState = FALSE;
volatile state_t stopState = FALSE;

const uint8_t PIN_MASK_START = (1 << PINB4);
const uint8_t PIN_MASK_STOP = (1 << PINB5);
const uint8_t PIN_MASK_JOYSTICK = (1 << PIND2);
static int16_t stepCountX = 0; // stepCount is a static to hold its previous value
static int16_t stepCountY = 0;

int main(void) {
	sei();

	i2cInit();   
  	ADC_Init();
  	SevenSeg_init();
  	timer1_init(62500);	// initialize timer 1 to every second
  	ADC_setReference(5);
	initPinChangeInterrupt();
	timer0_init(250);
	int8_t countdown = MAX_TIMER_VALUE; // 60 second countdown
	uint16_t display_buffer[HT16K33_NBUF];
	int32_t count = 1; // count needs to be 1 initially to avoid being 5 for the first run through
	
	DDRB = (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3); 	// turn on the ports of stepper motter
	DDRD |= (1 << DDD3) | (1 << DDD4);							// configure bits 3-4 as outputs for LEDs

	DDRB  &= ~(1 << DDB4);										// Configure bit 4 as input, explicitly
	DDRB  &= ~(1 << DDB5);										// Configure bit 5 as input, explicitly
  	PORTB |= (1 << PORTB4) | (1 << PORTB5);						// Turn pullups on for bits 4 and 5

	DDRD  &= ~(1 << DDD2);										// Configure bit 2 as input, explicitly
  	PORTD |= (1 << PORTD2);										// Turn pullup on

	while(1) { 
		if (buttonPushed == 1){ 								// everytime the button is pushed, change the game state
			if ((PIND & PIN_MASK_JOYSTICK) == 0){ 				// check if button is no longer pressed
				if (pushbuttonState == FALSE){ 					// change game from OFF to ON
					gameResult = NEITHER; 						// game inaction
					pushbuttonState = TRUE;
				}
				else if (pushbuttonState == TRUE){				// change game from ON to OFF
					pushbuttonState = FALSE;
				}
			}
			buttonPushed = 0;									// put the flag down
		}
		if (timerFlag0) { 										// every time the timer flag is HIGH, time is elapsed
			// to result in display not being wonky
			if (pushbuttonState == TRUE){
				if (count == 6){ 								// to have four ms being passed before taking another step
					count = 0; 									// reset count
					moveMotors(); 								// collect ADC values from joystick and move motor accordingly
				}
				count++; 										// increment count
			}
 			timerFlag0 = 0; 									// put the flag down
		
		} 
		if (timerFlag1 == 1){
			if (pushbuttonState == TRUE){ // game is on
				// check the voltage from the start sensor
      			if ( (PINB & PIN_MASK_START) == 0 ){
					startState = TRUE;
				}    
				else{
					startState = FALSE;
				} 

				// check the voltage from the stop sensor
				if ( (PINB & PIN_MASK_STOP) == 0 ){
					stopState = FALSE;
				}    
				else{
					stopState = TRUE;
				}    
			
				// check results/state of the game
				if ((startState == TRUE) && (stopState == TRUE) && (gameResult == NEITHER)){ // game has just ended
					if (countdown > 0){	// game won within the time limit
						gameResult = WON;
					}
					else {	// game not won within the time limit
						gameResult = LOST;
					}
				}
				else if ((startState == TRUE) && (stopState == FALSE) && (gameResult == NEITHER)){	// game is in action
					countdown--; // decriment countdown by one second
					if (countdown == 0) { // time is up!
						gameResult = LOST;
 					}
				} 

				// set/clear the LEDs depending on the game results
				if (gameResult == WON){
					greenLED();
				}
				else if (gameResult == LOST){
					redLED();
				}
				else {
					clearLEDs();
				}

				SevenSeg_number(abs(countdown), display_buffer); // display countdown
				
				// check countdown is greater than 0 and change pushbotton state if necessary
				if ((countdown <= 0) | (gameResult == WON) | (gameResult == LOST)){ // game countdown ended, game was won, or game was lost
					// display 'End' on SevenSeg because the game has ended
					display_buffer[0] = 0x00;
					display_buffer[1] = 0x79;
					display_buffer[3] = 0b01010100;
					display_buffer[4] = 0b01011110;
				}
				SevenSeg_write(display_buffer);
				timerFlag1 = 0;
			}
			else { // game is off
				// display 'OFF' on SevenSeg
				display_buffer[0] = 0x00;
				display_buffer[1] = 0x3F;
				display_buffer[3] = 0x71;
				display_buffer[4] = 0x71;
				SevenSeg_write(display_buffer);

				// reset countdown, start and stop states, game results, and LEDs
				countdown = MAX_TIMER_VALUE;
				startState = FALSE;
				stopState = FALSE;
				gameResult = OFF;
				clearLEDs();
			}
			
		}
	}
	return 0;
}

// Set timer for every second
void timer1_init(uint16_t timeout) {
  TCCR1B |= (1 << WGM12);										// Set mode to CTC
  TIMSK1 |= (1 << OCIE1A);										// Enable timer interrupt
  OCR1A = timeout;												// Load timeout value
  TCCR1B |= (1 << CS12);										// Set prescaler to 256
}


// Timer 1 interrupt flag
ISR(TIMER1_COMPA_vect) {				
  timerFlag1 = 1; 									            // time for another sample
}

void redLED(void){
    PORTD &= ~(1 << PORTD4);									// turn off green LED
	PORTD |= (1 << PORTD3);			    						// set bit 3 to turn upside down mode bit on
}

void greenLED(void){
    PORTD &= ~(1 << PORTD3);									// turn off red LED
	PORTD |= (1 << PORTD4);			    						// set bit 4 to turn upside down mode bit on
}

void clearLEDs(void){
    PORTD &= ~((1 << PORTD3) | (1 << PORTD4));					// clear LEDs
}

// pin change interrupt
void initPinChangeInterrupt(void) {
    PCICR |= (1 << PCIE2);                  // Enable pin-change interrupt for D-pins
    PCMSK2 |= (1 << PD2);                   // Set pin mask for bit 2 of Port D
}

// pin change interrupt flag
ISR(PCINT2_vect) {
   buttonPushed = 1; 				// Set flag to notify main
}


// step motor for X
void stepperMoveX(int16_t steps, int8_t direction){	
	uint16_t stepsTaken = 0; // keep track of step count
		
	// clearing the other port just in case
	PORTB &= ~((1 << PORTB0)|(1 << PORTB1) | (1 << PORTB2)|(1 << PORTB3));
		
	// check the direction
	if (direction == FORWARD){
		PORTB |= (1 << PORTB0);
	} else {
		PORTB &= ~(1 << PORTB0);
		steps = -steps; // because movement is currently negative
	}
		
	// how many steps to go
	while (stepsTaken < steps){
		PORTB |= (1 << PORTB1); // turn on (HIGH STATE) - AWAY
		_delay_us(500);  // wait frequency
		PORTB &= ~((1 << PORTB1)); // reset the pin back to LOW state TOWARD
		_delay_us(500);
		stepsTaken++;
	}
}
	
// step motor for Y
void stepperMoveY(int16_t steps, int8_t direction){	
	uint16_t stepsTaken = 0; // keep track of step count
	
	// clearing the other port just in case
	PORTB &= ~((1 << PORTB0)|(1 << PORTB1) | (1 << PORTB2)|(1 << PORTB3));
		
	// check the direction
	if (direction == FORWARD){
		PORTB |= (1 << PORTB2); // turn on HIGH
	} else {
		PORTB &= ~(1 << PORTB2); // turn off port
		steps = -steps;
	}
	
	// how many steps to go
	while (stepsTaken < steps){
		PORTB |= (1 << PORTB3); // turn on (HIGH STATE) - AWAY
		_delay_us(500); // delay for .5 ms
		PORTB &= ~((1 << PORTB3)); // reset the pin back to LOW state TOWARD
		_delay_us(500);
		stepsTaken++;
	}
}


// timer flag
ISR(TIMER0_COMPA_vect){
	timerFlag0 = 1;
}


// Set the ADC sampling rate. Use 125Hz to set up
void timer0_init(uint16_t timeout){
  TCCR0A |= (1 << WGM01); 					// set mode to CTC (clear timer on compare)
  TIMSK0 |= (1 << OCIE0A); 					// enable timer interrupt                         
  TCCR0B |= (1 << CS01) | (1 << CS00); 		// set to 64 for prescaler 
  // Timer clock freq / MAXCOUNT (250) = interrupt frequency (1000 Hz) & system frequency (16M) / 64 = timer frequency (250,000)
  OCR0A = timeout; 							// set it to stopped, initially
  
}

// gets ADC values from joystick and moves the motors accordingly
void moveMotors(void){
	int16_t step = 0; 						// get the step

	ADC_setChannel(1); 						// Set the channel for X- axis control
	uint16_t xValue = ADC_getValue(); 		// get the x value 
			
	ADC_setChannel(0); 						// Use analog Channel for Y-axis control
	uint16_t yValue = ADC_getValue(); 		// get the y value
	
	
	/*
		want to make sure the board does not exceed its limits (avoid rotating 360 degrees), 
		such that stepCountX/Y are used to keep track of how many steps taken in each direction
	
	*/
		
	// convert x ADC values to steps and move x axis motor accordingly
	if (xValue >= MID + DEADZONE && xValue <= MAX_ADC){ 	// ADC range: 524-1024
		if (stepCountX < LIMIT || stepCountX < -LIMIT) { 	// step count must be within the board movement range
			step = round(3*(xValue - MID - 12)/250);		// convert to range between 0 and 6 steps
			stepperMoveX(-step, BACKWARD); 					// right
			stepCountX += step; 							// increment the stepCount
		}
	} else if (xValue < MID - DEADZONE && xValue >= 0){ 	// ADC range: 0-500
		if (stepCountX > LIMIT || stepCountX > -LIMIT) {
			step = round((3*xValue/250) + 6);				// convert to range between 0 and 6 steps
			stepperMoveX(step, FORWARD); 					// left
			stepCountX -= step;
		}
		
	} 

	// convert y ADC values to steps and move y axis motor accordingly		
	if (yValue >= MID + DEADZONE && yValue <= MAX_ADC){ 	// ADC range: 524-1024
		if (stepCountY > LIMIT || stepCountY > -LIMIT) { 	// limiting the y directions
			step = round(3*(yValue - MID - 12)/250);		// convert to range between 0 and 6 steps
			stepperMoveY(step, FORWARD); 					// DOWN		
			stepCountY -= step;
		}
	} else if (yValue < MID - DEADZONE && yValue >= 0){ 	// ADC range: 0-500
		if (stepCountY < LIMIT || stepCountY < -LIMIT){
			step = round((3*yValue/250) + 6);				// convert to range between 0 and 6 steps
			stepperMoveY(-step, BACKWARD); 					// UP
			stepCountY += step;
		}
	} 
}