/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>

/*End of auto generated code by Atmel studio */

/*  *********************************************
*  SparkFun_ADXL345_Example
*  Triple Axis Accelerometer Breakout - ADXL345
*  Hook Up Guide Example
*
*  Utilizing Sparkfun's ADXL345 Library
*  Bildr ADXL345 source file modified to support
*  both I2C and SPI Communication
*
*  E.Robert @ SparkFun Electronics
*  Created: Jul 13, 2016
*  Updated: Sep 06, 2016
*
*  Development Environment Specifics:
*  Arduino 1.6.11
*
*  Hardware Specifications:
*  SparkFun ADXL345
*  Arduino Uno
*  *********************************************/

#include <SparkFun_ADXL345.h>         // SparkFun ADXL345 Library
//Beginning of Auto generated function prototypes by Atmel Studio
void ADXL_ISR();
//End of Auto generated function prototypes by Atmel Studio



/*********** COMMUNICATION SELECTION ***********/
/*    Comment Out The One You Are Not Using    */
ADXL345 adxl = ADXL345(10);           // USE FOR SPI COMMUNICATION, ADXL345(CS_PIN);
//ADXL345 adxl = ADXL345();             // USE FOR I2C COMMUNICATION

/****************** INTERRUPT ******************/
/*      Uncomment If Attaching Interrupt       */
int interruptPin = 2;                 // Setup pin 2 to be the interrupt pin (for most Arduino Boards)
int ledPin = 8; // rjw
volatile bool interruptCalled = false; // int called


/****************** bikeVibeParameters ******************/
#define DIG_FILTER 1

// Buffer for data read from sensors.
#define BUFFER_SIZE 3
int16_t m_buffer[BUFFER_SIZE];

#define NO_PTS 4096 // measurements before report a8 value

#define TWO_PI 2 * PI

#define SIMULATION	0

volatile boolean gotInterrupt = false;

/*
// calibration not needed: error < 1%
int32_t calibrationCoeffs[3][2] = {
621,103,309,105,-455,104}; // [x,y,z] (x - a0) / a1 to convert 10 bit integer to acceleration in ms-2
// measured for sum of 16 readings
*/

int32_t filterCoeffs[3][6] = {
	{2048,-3781,1752,100,9,-90},	{2048,134,353,634,1268,634},	{2048,-3980,1936,1991,-3982,1991} // fs = 1000 Hz
	//{2048, -3885, 1845, 67, 4, -63},	{2048, -5430, 8161, 1195, 2389, 1195},	{2048, -4019, 1973, 2010, -4020, 2010}  // fs = 1500 Hz; unstable
}; // [Hw,Hl,Hh][a0,a1,a2,b0,b1,b2]
// can scale these values by a fixed factor without change to the calculation
// use 2048 to enable shift divide

uint8_t nBits[3]; // used to store power of 2 of first element of filter coeffs
int32_t halfVal[3]; // used to store first element of filter coeffs divided by 2

int32_t filteredValues[3][3][4] = {
	{{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}},
	{{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}},
	{{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}};
	// [rows][columns] [x,y,z][buffer index][aa,bb,cc,dd] HwHlHh converts successively from aa->dd

	////////////////////////////////////// calibrate accelerometer measurements //////////////////////////////////////////
	void calibrate(const int32_t coeffs[3][2], int32_t calibratedValues[3][3][4], int16_t measurements[3], uint8_t last) {
		uint8_t i;
		for (i = 0; i < 3; i++) {
			calibratedValues[i][last][0] = (((int32_t)measurements[i] << 8) - coeffs[i][0]) / coeffs[i][1];
		}
	}

	////////////////////////////////////// applyFilter //////////////////////////////////////////
	void applyFilter(const int32_t filterCoeffs[3][6], int32_t filteredValues[3][3][4], uint8_t lastValue, uint8_t nBits[3], int32_t halfVal[3]) {
		uint8_t i, j, k;
		int32_t val;
		for (i = 0; i < 3; i++) {   // over x, y, z
			for (j = 0; j < 3; j++) { // over Hw, Hl, Hh
				int32_t result = 0;
				uint8_t l = lastValue;
				for (k = 0; k < 3; k++) { // over cyclic buffer
					result += filterCoeffs[j][k + 3] * filteredValues[i][l][j];
					if (k != 0)
					result -= filterCoeffs[j][k] * filteredValues[i][l][j + 1];
					if (l == 0) {
						l = 2;
						} else {
						--l;
					}
				}
				val = result + halfVal[j];
				if (val & 0x80000000)
				{
					val = val << 1;
					val = val >> (nBits[j] + 1);
					val |= 0x80000000;
				}
				else
				{
					val = val >> nBits[j];
				}
				filteredValues[i][lastValue][j + 1] = val;
			}
		}
	}

	//////////////////////////////////////flash_led()//////////////////////////////////////////

	static void flash_led(uint16_t noTimes, bool forever) {
		do {
			for (uint16_t j = 0; j < noTimes; j++) {
				digitalWrite(ledPin, 1);
				delay(100);
				digitalWrite(ledPin, 0);
				if (j < noTimes) delay(150);
			}
			if (forever) {
				delay(10000 - noTimes * 250);
			}
		} while (forever);
	}
	////////////////////////////////////// ADXL_ISR() //////////////////////////////////////////
	/* Look for Interrupts and Triggered Action    */
	void ADXL_ISR() {
		interruptCalled = true; // will be reset in main loop
	}
	////////////////////////////////////// checkPowerofTwo() //////////////////////////////////////////
	
	uint8_t checkPowerofTwo(int32_t x)
	{
		//checks whether a number is zero or not
		//only works for +ve numbers
		if (x < 1)
		return 0xff;
		
		uint8_t j = 0;

		//true till x is not equal to 1
		while( x != 1)
		{
			//checks whether a number is divisible by 2
			if(x % 2 != 0)
			return 0xff;
			x /= 2;
			++j;
		}
		return j;
	}
	////////////////////////////////////// simulate() //////////////////////////////////////////
void simulate(float frequency, float time, float amplitude, int16_t measurements[3])
{
int32_t val = (int32_t)(amplitude * sin(TWO_PI * frequency * time));
for (uint8_t i = 0; i < 3; i++) measurements[i] = val;
}


	////////////////////////////////////// ISR for TIMER1 //////////////////////////////////////////
ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{
	gotInterrupt = true;
	PORTB ^= (1 << PB1); // toggle port 9

}


	////////////////////////////////////// diagnoseInterrupt() //////////////////////////////////////////

	void diagnoseInterrupt() {
		// getInterruptSource clears all triggered actions after returning value
		// Do not call again until you need to recheck for triggered actions
		byte interrupts = adxl.getInterruptSource();

		// Free Fall Detection
		if(adxl.triggered(interrupts, ADXL345_FREE_FALL)){
			Serial.println("*** FREE FALL ***");
			//add code here to do when free fall is sensed
		}
		
		// Inactivity
		if(adxl.triggered(interrupts, ADXL345_INACTIVITY)){
			Serial.println("*** INACTIVITY ***");
			//add code here to do when inactivity is sensed
		}
		
		// Activity
		if(adxl.triggered(interrupts, ADXL345_ACTIVITY)){
			Serial.println("*** ACTIVITY ***");
			//add code here to do when activity is sensed
		}
		
		// Double Tap Detection
		if(adxl.triggered(interrupts, ADXL345_DOUBLE_TAP)){
			Serial.println("*** DOUBLE TAP ***");
			//add code here to do when a 2X tap is sensed
		}
		
		// Tap Detection
		if(adxl.triggered(interrupts, ADXL345_SINGLE_TAP)){
			Serial.println("*** TAP ***");
			//add code here to do when a tap is sensed
		}
	}
	////////////////////////////////////// setup() //////////////////////////////////////////
	/*          Configure ADXL345 Settings         */
	void setup(){
		
		Serial.begin(115200);                 // Start the serial terminal
		Serial.println("BikeVibe based on ADXL345 Accelerometer");
		Serial.println();
		
		//pinMode(ledPin, OUTPUT);
		DDRB |= (1 << DDB0) | (1 << DDB1); // pin 8, 9 output

		pinMode(interruptPin, INPUT);
		digitalWrite(ledPin,LOW);
		
		adxl.powerOn();                     // Power on the ADXL345

		adxl.setRangeSetting(16);           // Give the range settings
		// Accepted values are 2g, 4g, 8g or 16g
		// Higher Values = Wider Measurement Range
		// Lower Values = Greater Sensitivity

		adxl.setSpiBit(0);                  // Configure the device to be in 4 wire SPI mode when set to '0' or 3 wire SPI mode when set to 1
		// Default: Set to 1
		// SPI pins on the ATMega328: 11, 12 and 13 as reference in SPI Library
		
		adxl.setActivityXYZ(0, 0, 0);       // Set to activate movement detection in the axes "adxl.setActivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
		adxl.setActivityThreshold(75);      // 62.5mg per increment   // Set activity   // Inactivity thresholds (0-255)
		
		adxl.setInactivityXYZ(0, 0, 0);     // Set to detect inactivity in all the axes "adxl.setInactivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
		adxl.setInactivityThreshold(75);    // 62.5mg per increment   // Set inactivity // Inactivity thresholds (0-255)
		adxl.setTimeInactivity(10);         // How many seconds of no activity is inactive?

		adxl.setTapDetectionOnXYZ(0, 0, 0); // Detect taps in the directions turned ON "adxl.setTapDetectionOnX(X, Y, Z);" (1 == ON, 0 == OFF)
		
		// Set values for what is considered a TAP and what is a DOUBLE TAP (0-255)
		adxl.setTapThreshold(50);           // 62.5 mg per increment
		adxl.setTapDuration(15);            // 625 μs per increment
		adxl.setDoubleTapLatency(80);       // 1.25 ms per increment
		adxl.setDoubleTapWindow(200);       // 1.25 ms per increment
		
		// Set values for what is considered FREE FALL (0-255)
		adxl.setFreeFallThreshold(7);       // (5 - 9) recommended - 62.5mg per increment
		adxl.setFreeFallDuration(30);       // (20 - 70) recommended - 5ms per increment
		
		// Setting all interupts to take place on INT1 pin
		adxl.setImportantInterruptMapping(1, 1, 1, 1, 1);     // Sets "adxl.setEveryInterruptMapping(single tap, double tap, free fall, activity, inactivity);"
		// Accepts only 1 or 2 values for pins INT1 and INT2. This chooses the pin on the ADXL345 to use for Interrupts.
		// This library may have a problem using INT2 pin. Default to INT1 pin.
		
		// Set interrupts for each mode (1 == ON, 0 == OFF)
		adxl.InactivityINT(0);
		adxl.ActivityINT(0);
		adxl.FreeFallINT(0);
		adxl.doubleTapINT(0);
		adxl.singleTapINT(0);
		
		//attachInterrupt(digitalPinToInterrupt(interruptPin), ADXL_ISR, RISING);   // Attach Interrupt
		
		//check 2^N
		for (uint8_t j = 0; j < 3; j++)
		{
			nBits[j] = checkPowerofTwo(filterCoeffs[j][0]);
			if (nBits[j] == 0xff) flash_led(7, true); //exit with error 7
			halfVal[j] = filterCoeffs[j][0] >> 1;
		}
		// initialize timer1 to provide interrupts at 1ms intervals
		noInterrupts();           // disable all interrupts
		TCCR1A = 0;
		TCCR1B = 0;
		TCNT1  = 0;

		OCR1A = 16000;            // compare match register 16MHz/1000; fs = 1kHz
		//OCR1A = 10667;            // compare match register 16MHz/1000; fs = 1.5kHz
		TCCR1B |= (1 << WGM12);   // CTC mode
		TCCR1B |= (1 << CS10);    // No prescaler
		TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
		interrupts();             // enable all interrupts





		flash_led(2, false); // indicate about to start measuring

	}
	////////////////////////////////////// loop() //////////////////////////////////////////

	void loop(){
		int16_t results[256][2];
		uint32_t count = 0;
		uint32_t sum = 0; // sum of frequency-weighted acceleration for xyz axes
		TIMSK0 = 0; // turn off timer0.  Used by millis()
		//PORTB ^= (1 << PB0);
		//delayMicroseconds(100);
		//PORTB ^= (1 << PB0);
		while (!gotInterrupt) {;}
		gotInterrupt = false;
		while (count < NO_PTS) {
			uint8_t lastValue = (uint8_t)(count % 3); // index on filteredValues which is a cyclic buffer
			#if SIMULATION
			simulate(1, (float)count * 1e-3, 64.0, m_buffer);
			#else
			while (!gotInterrupt) {;}
			gotInterrupt = false;
			PORTB ^= (1 << PB0);
			adxl.readAccel((int16_t *)m_buffer);         // Read the accelerometer values and store them in variables declared above x,y,z
			PORTB ^= (1 << PB0);
			#endif
			#if DIG_FILTER
			if (count < 2) {
				//calibrate(calibrationCoeffs, filteredValues, (int16_t *)m_buffer, count);
				for (uint8_t j = 0; j < 3; j++) {
					for (uint8_t k = 1; k < 4; k++) {
						filteredValues[j][count][k] = (int32_t)(m_buffer[j]);
					}
				}
				} else {
				//calibrate(calibrationCoeffs, filteredValues, (int16_t *)m_buffer, lastValue);
				for (uint8_t j = 0; j < 3; j++) {
					filteredValues[j][lastValue][0] = (int32_t)(m_buffer[j]);
				}
				applyFilter(filterCoeffs, filteredValues, lastValue, nBits, halfVal);
			}

			++count;
			// calculate rms values and ahvSquared
			uint32_t sumThisInterval = 0;
			for (uint8_t i = 0; i < 3; i++) {
				sumThisInterval += (filteredValues[i][lastValue][3] * filteredValues[i][lastValue][3]);
			}
			sum += (sumThisInterval >> 8); //divide result by 256 to avoid overflow
			// could measure max here

			uint32_t index = (count >> 2) & 0xff;
			results[index][0] = filteredValues[0][lastValue][0];
			results[index][1] = filteredValues[0][lastValue][3];
			#endif // DIG_FILTER
		}
		//PORTB ^= (1 << PB0);
		//delayMicroseconds(100);
		//PORTB ^= (1 << PB0);
		TIMSK0 = 1; // turn on timer0.  Used by delay()


// test filter values for x axis
		Serial.print("Sum = ");
		Serial.println(sum);
		for (uint16_t i = 0; i < 256; i++)
		{
			Serial.print(results[i][0]);
			Serial.print(" ");
			Serial.println(results[i][1]);
		}
		Serial.println();

	
		delay(2000);
	}
	
	
	
	
	
	
	
	
	
	
	
	
	
	






