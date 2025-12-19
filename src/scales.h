#ifndef SCALES_H_
#define SCALES_H_

#include <Arduino.h>
#include <FixMath.h> 

#define NUM_KEYS 12

// Note: ROOT_NOTE must be defined in the main .ino file
extern volatile byte ROOT_NOTE; 

// --------------------------------------------------------------------------
// SCALE DEFINITIONS
// --------------------------------------------------------------------------
// Arrays store semitone intervals relative to 0, or raw MIDI offsets
const byte grades_pentatonic_minor[NUM_KEYS] = {0,3,5,7,10,12+0,12+3,12+5,12+7,12+10,12+12,12+12+3};
const byte grades_pentatonic_major[NUM_KEYS] = {0,2,4,7, 9,12+0,12+2,12+4,12+7, 12+9,12+12,12+12+2};
const byte      grades_blues_minor[NUM_KEYS] = {0,3,5,6,7,10,12+0,12+3,12+5,12+6,12+7,12+10};
const byte            grades_major[NUM_KEYS] = {0,2,4,5,7,9,11,12+0,12+2,12+4,12+5,12+7};
const byte            grades_minor[NUM_KEYS] = {0,2,3,5,7,8,10,12+0,12+2,12+3,12+5,12+7};
const byte   grades_minor_harmonic[NUM_KEYS] = {0,2,3,5,7,8,11,12+0,12+2,12+3,12+5,12+7};
const byte    grades_minor_melodic[NUM_KEYS] = {0,2,3,5,7,9,11,12+0,12+2,12+3,12+5,12+7};
const byte   grades_bebop_dominant[NUM_KEYS] = {0,2,4,5,7,9,10,11,12+0,12+2,12+4,12+5};
const byte      grades_bebop_major[NUM_KEYS] = {0,2,4,5,7,8, 9,11,12+0,12+2,12+4,12+5};
const byte   grade_half_diminished[NUM_KEYS] = {0,1,3,4,6,7, 9,10,12+0,12+1,12+3,12+4};

// Display Names
char* noteNames[12] = {"C ", "C#","D ","Eb", "E ", "F ", "F#", "G ", "Ab", "A ", "B", "Bb"};
char* scaleNames[10] = {"P min", "P Maj", "Blues", "Maj", "min", "Harmo", "Melod", "BeB d" ,"BeB M", "h Dim"}; 

// --------------------------------------------------------------------------
// HELPERS
// --------------------------------------------------------------------------

inline char* getNoteNames(int midinote) {
  // Ensure positive modulo for safety
  return noteNames[(midinote % 48) % 12];
}

inline char* getScaleNames(int index) {
  if(index < 0 || index > 9) return scaleNames[9];
  return scaleNames[index];
}

// Returns the Frequency in Mozzi Q16n16 format
inline Q16n16 getFreq(int scale, int grade, byte currentRoot) {
  int midi_note_offset = 0;
  
  // Boundary check for grade
  if (grade >= NUM_KEYS) grade = NUM_KEYS - 1;
  if (grade < 0) grade = 0;

  switch(scale) {
    case 0: midi_note_offset = grades_pentatonic_minor[grade]; break;
    case 1: midi_note_offset = grades_pentatonic_major[grade]; break;
    case 2: midi_note_offset = grades_blues_minor[grade]; break;
    case 3: midi_note_offset = grades_major[grade]; break;
    case 4: midi_note_offset = grades_minor[grade]; break;
    case 5: midi_note_offset = grades_minor_harmonic[grade]; break;
    case 6: midi_note_offset = grades_minor_melodic[grade]; break;
    case 7: midi_note_offset = grades_bebop_dominant[grade]; break;
    case 8: midi_note_offset = grades_bebop_major[grade]; break;
    case 9: midi_note_offset = grade_half_diminished[grade]; break;
    default: midi_note_offset = 0; break;
  }
  
  return Q16n16_mtof(Q16n0_to_Q16n16(midi_note_offset + currentRoot));
}

#endif /* SCALES_H_ */