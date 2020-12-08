/*
#include <midi_Defs.h>
#include <midi_Message.h>
#include <midi_Namespace.h>
#include <midi_Platform.h>
#include <midi_Settings.h>
#include <MIDI.h>
#include <serialMIDI.h>
*/

// Definition for NOP x n;
// https://forum.arduino.cc/index.php?topic=481251.0
__asm__ __volatile__(
  ".macro NOPX      P1          \n\t"
  ".rept &P1                    \n\t"
  "NOP                          \n\t"
  ".endr                        \n\t"   // End of Repeat
  ".endm                        \n\t"   // End of macro
);
#define NOPlength   12

uint8_t nano2NOP (uint8_t nanoseconds) {
    // adequate for min timings, potentially a problem for max
    return ( nanoseconds % NOPlength ) + 1;
}

// https://arduino.stackexchange.com/questions/16698/arduino-constant-clock-output
// clock business

/*  
TODO: Investigate a kind of deltaTime situation for the delays,
particularly for the mode-switch waiting times. Possible use of the timers?
They scare me :(
*/

#define instructionMask (byte)B111
#define write    (byte)B00010
#define data     (byte)B10000
#define address  (byte)B00000
#define bank0    (byte)B00000
#define bank1    (byte)B01000
#define read     (byte)B00100
#define inactive (byte)B00111

#define CS  78
#define RD  77
#define WR  76
#define A1  75
#define A0  74
#define D0  53

void setup() {
    pinMode(CS, OUTPUT);
    pinMode(A0, OUTPUT);
    pinMode(A1, OUTPUT);
}

void loop() {

}

void something (byte instruction, byte dataBus) {
    if ( instruction % 2 | bitRead(instruction,1) ^ bitRead(instruction,2)) {
        // undefined behaviour (low) or inactive (high); go for inactive
    }
    else {
        // active
        if(bitRead(instruction,1)) {
            // should be in write mode
            if (bitRead(instruction, 2)) {
                // this shouldn't happen.
            }
            writeBus(instruction, dataBus);
        } else {
            // should be in read mode
            if (bitRead(instruction, 1)) {
                // this shouldn't happen.
            }
            readBus();
        }
    };
};

void writeBus (byte instruction, byte dataBus) {
    digitalWrite(A0, bitRead(instruction,4));
    digitalWrite(A1, bitRead(instruction,3));

    __asm__ __volatile__("NOPX 1"); // tAS, min 10ns
    digitalWrite(CS, LOW);
    digitalWrite(WR, LOW);
    for ( int i = 0; i < 8; i++ ) {
        pinMode(D0 + i, OUTPUT);
        digitalWrite( D0 + i, bitRead(dataBus,i));
    }

    __asm__ __volatile__("NOPX 17"); // tCSW & tWW, min 200ns (kind of)
    digitalWrite(WR,HIGH);
    digitalWrite(CS,HIGH);
    
    __asm__ __volatile__("NOPX 1"); // tAH, min 10ns
    digitalWrite(A0, LOW);
    digitalWrite(A1, LOW);

    __asm__ __volatile__("NOPX 1"); // tWDH - tAH, min 10ns
    for ( int i = 0; i < 8; i++ ) {
        pinMode(D0 + i, INPUT);
    }
}

byte readBus () {
    digitalWrite(A0, LOW);
    digitalWrite(A1, LOW);

    __asm__ __volatile__("NOPX 1"); // tAS, min 10ns
    digitalWrite(CS, LOW);
    digitalWrite(WR, LOW);
    
    __asm__ __volatile__("NOPX 21"); // tACC, min 250ns (I think)

    for ( int i = 0; i < 8; i++ ) {
        // do the read
    }

    __asm__ __volatile__("NOPX 9"); // tCSR & tRW, min 100ns (kind of)
    digitalWrite(WR,HIGH);
    digitalWrite(CS,HIGH);
    
    __asm__ __volatile__("NOPX 1"); // tAH & tDH, min 10ns
    for ( int i = 0; i < 8; i++ ) {
        pinMode(D0 + i, INPUT);
    }
}

void rawSetPins (byte instruction) {
    digitalWrite(CS, bitRead(instruction,0));
    digitalWrite(RD, bitRead(instruction,1));
    digitalWrite(WR, bitRead(instruction,2));
    digitalWrite(A1, bitRead(instruction,3));
    digitalWrite(A0, bitRead(instruction,4));
}
