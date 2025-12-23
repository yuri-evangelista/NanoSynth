/*
 * Project: Mozzi Synth with FSLP and OLED (SSD1306Ascii version)
 * Description: A monophonic synthesizer using an FSLP sensor for pitch/volume 
 * and rotary encoders for Scale/Key selection.
 * * REQUIRED LIBRARIES:
 * 1. Mozzi
 * 2. SSD1306Ascii (by Bill Greiman)
 */

#include <Mozzi.h>
#include <Oscil.h>
#include <FixMath.h> 
#include <EventDelay.h>
#include <ADSR.h>
#include <Ead.h> 
#include <ResonantFilter.h>
#include <RollingAverage.h>
#include <tables/saw2048_int8.h>
#include <tables/triangle_dist_squared_2048_int8.h>
#include <tables/triangle_valve_2_2048_int8.h>
#include <tables/cos1024_int8.h>
#include <tables/square_analogue512_int8.h> //added
#include <mozzi_rand.h>
#include <mozzi_midi.h>

// DISPLAY LIBRARIES
// Use AvrI2c to avoid conflicts with Mozzi's interrupt usage
#include <SSD1306Ascii.h>
#include <SSD1306AsciiAvrI2c.h>

// Custom Headers
#include "scales.h"
#include "fslp.h"

// --------------------------------------------------------------------------
// HARDWARE CONFIGURATION & PINS
// --------------------------------------------------------------------------
// FSLP Sensor Pins
const byte PIN_FSLP_SENSE  = A0; // Sense Line (SL)
const byte PIN_FSLP_DRIVE2 = A1; // Drive Line 2 (D2)
const byte PIN_FSLP_DRIVE1 = 4;  // Drive Line 1 (D1)
const byte PIN_FSLP_BOT_R0 = 5;  // Resistor R0 (D)

// Potentiometers
const byte PIN_POT_INTENSITY = A2;
const byte PIN_POT_VIB_FREQ  = A3;

// Rotary Encoder
const byte PIN_ENC_A      =  2;   // Interrupt Pin
const byte PIN_ENC_B      =  3;   // Interrupt Pin
const byte PIN_ENC_BUTTON =  8;   // Push button

//Buttons
const byte PIN_SOUND_BUTTON     = 12;   // Push button for sound change

// OLED Display Settings
#define I2C_ADDRESS 0x3C
#define OLED_RESET -1 // Not used

// --------------------------------------------------------------------------
// AUDIO SETTINGS
// --------------------------------------------------------------------------
#define MOZZI_AUDIO_MODE MOZZI_OUTPUT_2PIN_PWM 
#define MOZZI_AUDIO_BITS_PER_CHANNEL 7

// Envelope Timing (Attack, Decay, Sustain, Release)
const int ENV_LVL_ATTACK  = 180;
const int ENV_LVL_SUSTAIN = 32;
const int FILT_LVL_ATTACK = 127;
const int FILT_LVL_SUSTAIN = 32;

// --------------------------------------------------------------------------
// GLOBAL OBJECTS
// --------------------------------------------------------------------------
// Display Object (AvrI2c version is lighter and interrupt-safe)
SSD1306AsciiAvrI2c oled;

// Oscillators
Oscil<SAW2048_NUM_CELLS, MOZZI_AUDIO_RATE> aOscil(SAW2048_DATA);
Oscil<TRIANGLE_DIST_SQUARED_2048_NUM_CELLS, MOZZI_AUDIO_RATE> bOscil(TRIANGLE_DIST_SQUARED_2048_NUM_CELLS);
Oscil<SQUARE_ANALOGUE512_NUM_CELLS, MOZZI_AUDIO_RATE> cOscil(SQUARE_ANALOGUE512_DATA);

Oscil<COS1024_NUM_CELLS, MOZZI_AUDIO_RATE> aVibrato(COS1024_DATA);

// Envelopes & Filters
ADSR<MOZZI_CONTROL_RATE, MOZZI_CONTROL_RATE> envelope_filter;
ADSR<MOZZI_AUDIO_RATE, MOZZI_AUDIO_RATE> envelope;

Ead ead_envelope_filter(MOZZI_CONTROL_RATE); // resolution will be MOZZI_CONTROL_RATE

MultiResonantFilter<uint8_t> multiResFilt;
RollingAverage <int, 16> kAverage; // Smooths FSLP pressure readings

EventDelay kFslpDelay; // Throttle FSLP reads
EventDelay kPotDelay;  // Throttle Potentiometer reads
EventDelay kDisplayDelay; // Throttle Display Update

// --------------------------------------------------------------------------
// STATE VARIABLES
// --------------------------------------------------------------------------
// Musical State
volatile byte ROOT_NOTE = 57;   // Externally accessed by scales.h
volatile byte SCALE_INDEX = 0;
float vibFreq = 5.0f;
UFix<0,8> intensity = 0.5;
uint8_t resonance = 230;

byte SOUND_NUMBER = 0;
// 1. Define the strings in Flash memory to save RAM
const char sound0[] PROGMEM = "SOUND 0";
const char sound1[] PROGMEM = "SOUND 1";
const char sound2[] PROGMEM = "SOUND 2";
const char sound3[] PROGMEM = "SOUND 3";
// 2. Create a table of pointers to those strings
const char* const SOUND_NAMES[] PROGMEM = {sound0, sound1, sound2, sound3};

// Encoder State
volatile uint32_t rootEncoderPos = 2147483640;
volatile uint32_t scaleEncoderPos = 2147483640;
volatile uint32_t rootOldEncPos = 2147483640;
volatile uint32_t scaleOldEncPos = 2147483640;

 //Button
static int oldCountButtonPressed;

// Interrupt Flags
volatile byte aFlag = 0;
volatile byte bFlag = 0;
volatile byte encoderReading = 0;

// FSLP State
int fslpPressure = 0;
int fslpPosition = 0;
int fslpGrade = 0;
int fslpGain = 128;

// --------------------------------------------------------------------------
// INTERRUPT SERVICE ROUTINES (Encoder)
// --------------------------------------------------------------------------
void PinA() {
  cli();
  encoderReading = PIND & 0xC; // Read Pins 2 & 3 directly
  bool btnState = digitalRead(PIN_ENC_BUTTON);

  if (encoderReading == B00001100 && aFlag) {
    if (btnState == LOW) rootEncoderPos--;
    else scaleEncoderPos--;
    
    bFlag = 0;
    aFlag = 0;
  }
  if (encoderReading == B00000100) bFlag = 1;
  sei();
}

void PinB() {
  cli();
  encoderReading = PIND & 0xC;
  bool btnState = digitalRead(PIN_ENC_BUTTON);

  if (encoderReading == B00001100 && bFlag) {
    if (btnState == LOW) rootEncoderPos++;
    else scaleEncoderPos++;

    bFlag = 0;
    aFlag = 0;
  }
  if (encoderReading == B00001000) aFlag = 1;
  sei();
}

// --------------------------------------------------------------------------
// HELPER FUNCTIONS
// --------------------------------------------------------------------------
void updateOledDisplay() {
  // Clearing the screen can be slow, but essential for variable text lengths
  oled.clear(); 
  
  // To be modified, should read an enum variable
  oled.setFont(System5x7);
  oled.set1X();
  oled.setCursor(80, SOUND_NUMBER);
  //oled.print((char*)SOUND_NAMES[SOUND_NUMBER]);
  oled.print((__FlashStringHelper*)pgm_read_word(&SOUND_NAMES[SOUND_NUMBER]));

  // -- Row 0/1: Root Note --
  oled.setFont(ZevvPeep8x16);
  oled.set1X();
  oled.setCursor(0, 1);
  oled.print("ROOT");


  oled.set2X(); // Double size for visibility
  oled.setCursor(44, 0); 
  oled.print(getNoteNames(ROOT_NOTE));
  

  // -- Row 4/5: Scale Name --
  oled.set1X();
  oled.setCursor(0, 5);
  oled.print("SCALE");
  
  oled.set2X(); 
  oled.setCursor(44, 4); // Below the label
  oled.print(getScaleNames(SCALE_INDEX));
  oled.set1X(); // Reset to normal
}

/**
 * Monitors a pin and maintains an internal count of presses.
 * Returns the current total count.
 */
int countButtonPress(int pin) {
   // Static variables persist between function calls
  static int internalCounter = 0;
  static int lastButtonState = HIGH;
  static int stableButtonState = HIGH;
  static unsigned long lastDebounceTime = 0;
  const unsigned long DEBOUNCE_DELAY = 25;

  int reading = digitalRead(pin);

  // Debounce logic: reset timer if the pin flickers
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    // If the state has been stable and is different from our last confirmed state
    if (reading != stableButtonState) {
      stableButtonState = reading;

      // Increment count on transition to LOW (pressed)
      if (stableButtonState == LOW) {
        internalCounter++;
      }
    }
  }

  lastButtonState = reading;
  return internalCounter; // Always return the current total
}

// --------------------------------------------------------------------------
// SETUP
// --------------------------------------------------------------------------
void setup() {
  // 1. Hardware Init
  pinMode(PIN_ENC_BUTTON, INPUT_PULLUP);
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);

  pinMode(PIN_SOUND_BUTTON, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), PinA, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), PinB, RISING);

  // 2. Display Init (SSD1306Ascii)
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.displayRemap(true); //Rotate 180°
  oled.setFont(System5x7);
  updateOledDisplay();

  // 3. Audio Init
  aOscil.setPhase(113);
  bOscil.setPhase(223);
  cOscil.setPhase(179);
  
  aVibrato.setFreq(vibFreq);

  envelope.setADLevels(ENV_LVL_ATTACK, ENV_LVL_SUSTAIN);
  envelope.setTimes(1, 100, 512, 1000);

  envelope_filter.setADLevels(FILT_LVL_ATTACK, FILT_LVL_SUSTAIN);
  envelope_filter.setTimes(50, 256, 2400, 1024); //100,100

  ead_envelope_filter.set(50, 128); //attack ms, decay ms
  
  multiResFilt.setCutoffFreqAndResonance(255, resonance);
  //Serial.begin(115200);

   // Initialize Timers
  kFslpDelay.set(15); // Check FSLP every ~15ms (approx 66Hz)
  kFslpDelay.start();

  kPotDelay.set(250); // Check pots every 100ms
  kPotDelay.start();

  kDisplayDelay.set(250);
  kDisplayDelay.start(100);

  startMozzi();
}

// --------------------------------------------------------------------------
// MAIN CONTROL LOOP (Runs at MOZZI_CONTROL_RATE)
// --------------------------------------------------------------------------
void updateControl() {

  if (kDisplayDelay.ready()) {
  // --- 1. Handle Encoder Changes ---
  if (rootOldEncPos != rootEncoderPos) {
    ROOT_NOTE = (rootEncoderPos % 12) + 57;
    updateOledDisplay();
    rootOldEncPos = rootEncoderPos;
  }

  if (scaleOldEncPos != scaleEncoderPos) {
    SCALE_INDEX = scaleEncoderPos % 10;
    updateOledDisplay();
    scaleOldEncPos = scaleEncoderPos;
  } 
  kDisplayDelay.start();
  }
  // --- 2. Handle Push Button for sound change ---
  int currentCountButtonPressed = countButtonPress(PIN_SOUND_BUTTON) % 4;
  if (oldCountButtonPressed != currentCountButtonPressed) {
    SOUND_NUMBER = currentCountButtonPressed ;
    oldCountButtonPressed = currentCountButtonPressed;
    updateOledDisplay();
    envelope.noteOff();
    envelope_filter.noteOff();
  }

  // --- 3. Handle Potentiometers (Throttled) ---
  if (kPotDelay.ready()) { 
    intensity = analogRead(PIN_POT_INTENSITY) / 1024.f;
    vibFreq = analogRead(PIN_POT_VIB_FREQ) / 100.f + 1;
    aVibrato.setFreq(vibFreq);
    kPotDelay.start();
    
  }

 
  // --- 4. Switch sounds ---
  

    int cutOff;
    int val;
    switch(SOUND_NUMBER) {
    case 0: 
      val = FILT_LVL_ATTACK - envelope_filter.next();
      cutOff = max(48, val);
      break;

    case 1:   
      cutOff = envelope_filter.next(); 
      break;

    case 2: 
      val = (  envelope_filter.next() - ead_envelope_filter.next()) + 127;
      val = max(48, val);
      val = min(val, 127);
      cutOff = ( val * envelope_filter.next() ) >> 7; 
      break;

    case 3: 
      //cutOff = ( ((uint16_t)(sq(ead_envelope_filter.next() ) ) ) >> 8 );
      //cutOff = ((uint16_t)(ead_envelope_filter.next()+120) ) >> 2 ;
      cutOff = ((uint16_t)(ead_envelope_filter.next())+64 ) >> 2 ;
      break;
    
    default: cutOff = 255 - envelope_filter.next(); break;
  }
 
  multiResFilt.setCutoffFreqAndResonance( cutOff, resonance);


  // --- 5. Handle FSLP Sensor ---
 if(kFslpDelay.ready()) {
 
  bool isNoteActive = getFSLP(PIN_FSLP_SENSE, PIN_FSLP_DRIVE1, PIN_FSLP_DRIVE2, PIN_FSLP_BOT_R0, 
                              fslpPressure, fslpPosition, fslpGrade, fslpGain);
 
  if (isNoteActive) {
    // Calculate Frequency
    Q16n16 s_freq = getFreq(SCALE_INDEX, fslpGrade, ROOT_NOTE);

    // Smooth gain
    int averagedGain = kAverage.next(fslpGain);
    
    // Update Envelopes
    envelope.noteOff(); 
    envelope_filter.noteOff();
    envelope.setADLevels(averagedGain, 64);

    // Update Oscillators
    aOscil.setFreq_Q16n16(s_freq);
    bOscil.setFreq_Q16n16(s_freq /3.0f + 0.3f);
    cOscil.setFreq_Q16n16(s_freq /3.0f  + 0.2f);

    envelope.noteOn();
    envelope_filter.noteOn();
    ead_envelope_filter.start();

  } else {
    // Release Note
    envelope.noteOff();
    envelope_filter.noteOff();
  } 
     kFslpDelay.start();
  }
   
  // --- 6. Update Filter Envelope ---
  envelope_filter.update();
}

// --------------------------------------------------------------------------
// AUDIO LOOP (Runs at MOZZI_AUDIO_RATE)
// --------------------------------------------------------------------------
AudioOutput updateAudio() {
  AudioOutput filtered;
  envelope.update();
   
  // Calculate Vibrato
  auto vibratoVal = intensity * toSFraction(aVibrato.next());
  
  // Mix Oscillators
  int oscMix = (aOscil.phMod(vibratoVal)) + 
               (bOscil.next() >> 2) + 
               (cOscil.next() >> 1);
  

  uint16_t wave = (envelope.next() * oscMix) >> 8;
  

  multiResFilt.next((uint16_t)wave);


  // Apply Low Pass or band pass Filter
  switch (SOUND_NUMBER){
    case 2:
      filtered = multiResFilt.notch();
      break;
    
    case 3:
      filtered = multiResFilt.band()<<1;
      break;
    
    default: //sounds 0 and 1
      filtered =  multiResFilt.low();
      break;
  }


  return MonoOutput::fromNBit(9, filtered);
}

void loop() {
  audioHook(); 
}