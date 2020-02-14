// Tremolo Waveform
#include "modulate.h"

// OLED Library
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, 4);

// Pinouts
#define BTN_PIN             8
byte btnState = 0;
byte btnCount = 0;
byte currentState = 0;
int interruptCounter = 0;

// Constants
#define PI                  3.14159265359
#define SAMPLE_RATE         38500
#define SIZE                250

//**** FLANGING *****//
#define W_FLANG             PI/SAMPLE_RATE
#define ALPHA               0.8

//***** Chorusing *****//
#define W_CHORUS            PI/SAMPLE_RATE

//**** Phasing *****//
#define BANDWIDTH           0.2*PI
// Notch 1 Parameters
#define A_1                 PI/5
#define W_1                 PI/SAMPLE_RATE
#define MIN_FREQ1           PI/5
// Alpha Stuff
#define alpha               (float)(1/cos(BANDWIDTH) - tan(BANDWIDTH))

// Flanging and Chorusing Buffer
static int buffer[SIZE] = {0};
  
// PHASING
// Buffer length of 2 as max delay is i - 2 in difference eqn
// Buffer holding inputs
static int bufferInputs[2] = {0};
// Buffer holding outputs of first notch filter 
static int bufferMiddle[2] = {0};

//***********************//

// Common among all
static int current = 0;
// Store output
float LeftOutput = 0;

//******* Tremolo Stuff ********//
int cnt = 0;
byte divider = 0;
byte sample = 0;

byte tremolo(int input){
  // Divider to only apply signal modulation to slow down effect
  if ( divider == 4 ){
   // Reset count if more than array modulate.h array size
   if ( cnt > 1023 ) cnt = 0;
   else cnt++; 
   // Reset divider count to 0     
   divider = 0;
  }
  // Increment divider count
  divider++;
  // Modulate input sample with sine waveform 
  sample = pgm_read_byte(&waveform[cnt]);
  return map(input,0,1023,0,sample);
}

//******* Distortion Effect ********//
/**
 * Effect quantizes sound amplitudes to a set amplitude ( clipping audio )
 */

byte fuzzDistortion(int input){
  if (input < 100){
    return 200;
  }
  else if( ( input > 100 ) && ( input < 150) ){
    return 230;
  }
  else {
    return 255;
  }
}

//******* Phasing Effect ********//
/**
 * Sound from moving a Notch Filter across Frequency Spectrum 
 */

byte Phasing(int input){  
  
  // Implementing sweeping feature of notch
  float varyingNotch = (float)A_1*(1 - cos(W_1*current)) + MIN_FREQ1;
  // Beta values change according to new notch positions
  float beta = (float)cos(varyingNotch);
  // Apply phasing effect to first notch filter
  if ((current - 1) <= 0 || (current - 2) <= 0){
      if ( (current - 1) > 0 ) {
          LeftOutput = ((1+alpha)/2)*bufferInputs[current] - (1+alpha)*beta*bufferInputs[current - 1] 
    + beta*(1 + alpha)*bufferMiddle[current - 1];
    } 
  else if ( (current - 2) > 0 ) {
          LeftOutput = ((1+alpha)/2)*bufferInputs[current] + (1+alpha)*bufferInputs[current - 2] 
    - alpha*bufferMiddle[current - 2];
    }
      else {
        LeftOutput = ((1+alpha)/2)*bufferInputs[current];
    }
  }
    // Apply full difference eqn
  else {
        LeftOutput = 
    ((1+alpha)/2)*bufferInputs[current] - (1+alpha)*beta*bufferInputs[current - 1]
    + (1+alpha)*bufferInputs[current - 2] + beta*(1 + alpha)*bufferMiddle[current - 1]
    - alpha*bufferMiddle[current - 2];
  }

  // Circular Buffer Implementation to store past inputs, middle and outputs
  bufferInputs[current] = input;
  bufferMiddle[current] = LeftOutput;
  current++;
  current = current % 2;

  return map((int)LeftOutput,0,1023,0,255);
}

//******* Chrousing Effect ********//
/**
 * An effect with a chorus sound ( echoes )
 */

byte Chorusing(int input) {
    
  // Getting Delays based on given equations
  int Dvarying1 = (int)( (SIZE*( 1 - cos((float)(PI/SAMPLE_RATE)*current))) );
  int Dvarying2 = (int)( (SIZE*( 1 + cos((float)(PI/SAMPLE_RATE)*current))) );

  // Check if one of them is non negative, apply the non negative term
  if ( (current - Dvarying1 <= 0 ) || (current - Dvarying2 <= 0) ){
    if ( current - Dvarying1 > 0 ) {
      LeftOutput = input + ALPHA*buffer[current - Dvarying1]; 
    }
    else if ( current - Dvarying2 > 0 ) {
      LeftOutput = input + ALPHA*buffer[current - Dvarying2]; 
    }
    else {
      // Both indices are negative
      LeftOutput = input;
    }
  }
  else {
    // Apply Chorusing Effect
    LeftOutput = input + ALPHA*buffer[current - Dvarying1] + ALPHA*buffer[current - Dvarying2];
  }

  // Circular Buffer Implementation to store past D values
  buffer[current] = input;
  current++;
  current = current % SIZE;

  return (map((int)(LeftOutput),0,1023,0,255));
}

//******* Flanger Effect ********//
/**
 * Sound effect with mixing identical signals with one delayed slightly
 */
byte Flanger(int input){
    
  // Getting Delay based on flanging delay
  int Dvarying = (int)( (SIZE*( 1 - cos((float)(W_FLANG)*current))) );

  // Apply Flanging Effect using difference equation
  LeftOutput = input + ALPHA*buffer[current - Dvarying];

  // Circular Buffer Implementation to store past D values
  buffer[current] = input;
  current++;
  current = current % SIZE;

  return map((int)LeftOutput,0,1023,0,255);
}

/**
 * OLED Display Function
 */
void displayState(byte state){
  
  // Stop all current ADC operations
  ADCSRA = 0;
  
  // Set Display Stuff
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(10,0);
  if (state == 1){
    display.println(F("Flanger"));
  }
  else if ( state == 2 ){
    display.println(F("Chorusing"));
  }
  else if ( state == 3 ) {
    display.println(F("Phasing"));
  }
  else if ( state == 4 ){
    display.setCursor(0,0);
    display.println(F("Distortion"));
  }
  else if ( state == 5 ){
    display.println(F("Tremolo"));
  }
  else if ( state == 6 ) {
    display.println(F("Bit Crush"));
  }
  else {
    display.println(F("No Effect"));
  }
  display.display(); 

  // Restart ADC
  ADCSRA |= (1 << ADPS2) | (1 << ADPS0); //set ADC clock with 32 prescaler- 16mHz/32=500kHz
  ADCSRA |= (1 << ADATE); //enabble auto trigger
  ADCSRA |= (1 << ADIE);  //enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN);  //enable ADC
  ADCSRA |= (1 << ADSC);  //start ADC measurements

  // Store current state information for future reference
  currentState = state;
}

void setup(){  

  // Setup Pinouut Stuff
  for (byte i=0;i<8;i++){
    // 8-bit DAC output
    pinMode(i,OUTPUT);
  }    
  // Button select
  pinMode(BTN_PIN,INPUT_PULLUP);

  // Setup I2C OLED Display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  // Initial Screen Display Routine
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(10, 0);
  display.println(F("UNO Audio Processor!"));
  display.display();
  delay(2000);
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(10, 0);
  display.println(F("Developed by"));
  display.setCursor(10, 10);
  display.println(F("Matthew Yong"));
  display.setCursor(10, 20);
  display.println(F("2019"));
  display.display();   
  delay(2000);  

  cli();//disable interrupts
   
  // Set up continuous sampling of analog pin 0
  
  //clear ADCSRA and ADCSRB registers
  ADCSRA = 0;
  ADCSRB = 0;
  
  // ADC Multiplexer Settings
  ADMUX |= (1 << REFS0); //set reference voltage
  ADMUX |= (1 << ADLAR); //left align the ADC value- so we can read highest 8 bits from ADCH register only

  // ADC Settings
  ADCSRA |= (1 << ADPS2) | (1 << ADPS0); //set ADC clock with 32 prescaler- 16mHz/32=500kHz
  ADCSRA |= (1 << ADATE); //enabble auto trigger
  ADCSRA |= (1 << ADIE);  //enable interrupts when measurement complete
  // Start ADC
  ADCSRA |= (1 << ADEN);  //enable ADC
  ADCSRA |= (1 << ADSC);  //start ADC measurements
  
  sei(); //enable interrupts

  // Show first effect in use
  displayState(btnState);    
}

// ISR that plays audio every 38.5 KHz
ISR(ADC_vect) {
  // Button selects different effect to apply to current sample
  if ( btnState == 1 ){
    PORTD = Flanger(ADCH);    
  }
  else if ( btnState == 2 ) {
    PORTD = Chorusing(ADCH);
  }
  else if ( btnState == 3 ){
    PORTD = Phasing(ADCH);
  }
  else if ( btnState == 4) {
    PORTD = fuzzDistortion(ADCH);
  }
  else if ( btnState == 5 ){
    PORTD = tremolo(ADCH);
  }
  else if ( btnState == 6 ){
    // Bit Crusher
    byte bitCrushBit = 4;
    PORTD = ADCH << bitCrushBit;
  }
  else {
    PORTD = ADCH;
  }
  // check button after isr called 4000 times 
  if (interruptCounter == 4000){
    if(digitalRead(BTN_PIN) == LOW){
      if ( btnState > 7 ){
        btnState = 0;
      }
      else {
        btnState++;
      }
    }
    interruptCounter = 0;    
  }
  interruptCounter++;
}

void loop(){
  // Only change display OLED when recognise a state change 
  if ( currentState!= btnState ){
    displayState(btnState);    
  }
}














