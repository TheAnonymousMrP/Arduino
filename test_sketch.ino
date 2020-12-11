/*
#include <MIDI.h>
#include <midi_Defs.h>
#include <midi_Message.h>
#include <midi_Namespace.h>
#include <midi_Platform.h>
#include <midi_Settings.h>
#include <serialMIDI.h>

#ifndef ARDUINO_MAIN
#include <wiring_private.h>
#endif

*/

#ifndef _digitalWriteFast_h_
#include <digitalWriteFast.h>
#endif

// Definition for NOP x n;
// https://forum.arduino.cc/index.php?topic=481251.0
__asm__ __volatile__(".macro NOPX      P1          \n\t"
                     ".rept &P1                    \n\t"
                     "NOP                          \n\t"
                     ".endr                        \n\t" // End of Repeat
                     ".endm                        \n\t" // End of macro
);

// https://arduino.stackexchange.com/questions/16698/arduino-constant-clock-output
// clock business

/*
TODO: Investigate a kind of deltaTime situation for the delays,
particularly for the mode-switch waiting times. Possible use of the timers?
They scare me :(
*/

#define pIC             PORTA + PA5 // 73
#define pIRQ            PORTA + PA6 // 72
#define pCS             PORTA + PA0 // 78
#define pRD             PORTA + PA1 // 77
#define pWR             PORTA + PA2 // 76
#define pA1             PORTA + PA3 // 75
#define pA0             PORTA + PA4 // 74
#define pD0             PORTC + PC0 // 53
#define DataBus         PORTC       // 53-60

struct Register {
    byte address;
    byte sizeAndOffset; // Offset << 4
};

// I think the read versions are unnecessary?? 
// Keep for reading from files
byte readRegister(byte a, Register r) {
    if ((r.sizeAndOffset & B00001111) == 8) {
        return a;
    } else if ((r.sizeAndOffset & B00001111) > 8) {
        return 0;
    } else {
        byte temp = a >> (r.sizeAndOffset >> 4);
        return temp | (0xFF >> (r.sizeAndOffset & B00001111));
    }
}
byte writeRegister(byte a, Register r) {
    if ((r.sizeAndOffset & B00001111) == 8) {
        return a;
    } else if ((r.sizeAndOffset & B00001111) > 8) {
        return 0;
    } else {
        return a << (r.sizeAndOffset >> 4);
    }
}
// TODO: Figure out if these are backwards?? Endianness etc.
word readTimerA(byte a, byte b) { return ((word)a << 2) | (b & B00000011); }
word writeTimerA(word a) {
    byte b = (byte)a & B00000011;
    return ((a >> 2) << 8 | b);
}
word readFNum(byte a, byte b) { return a | ((b & B00000111) << 8); }
word writeFNum(word a) {
    word b = (a >> 8) & B00000111;
    return (a << 8 | b);
}

const Register Test1       = {0x21, 8 | (0 << 4)};
const Register LFO         = {0x22, 4 | (0 << 4)};
const Register TimerA      = {0x24, 10 | (0 << 4)};
const Register TimerB      = {0x26, 8 | (0 << 4)};
const Register TimerLoad   = {0x27, 2 | (0 << 4)};
const Register TimerEnable = {0x27, 2 | (2 << 4)};
const Register TimerReset  = {0x27, 2 | (4 << 4)};
const Register TimerMode   = {0x27, 2 | (6 << 4)};
const Register Channel     = {0x28, 3 | (0 << 4)};
const Register Slot        = {0x28, 4 | (4 << 4)};
const Register DACData     = {0x2A, 8 | (0 << 4)};
const Register DACSelect   = {0x2B, 1 | (7 << 4)};
const Register Test2       = {0x2C, 8 | (0 << 4)};
// These next few have a byte for each operator in each channel.
// TODO: Figure out how to factor in an offset
const Register DT    = {0x30, 3 | (4 << 4)};
const Register Multi = {0x30, 4 | (0 << 4)};
const Register TL    = {0x40, 7 | (0 << 4)};
const Register KS    = {0x50, 2 | (6 << 4)};
const Register AR    = {0x50, 5 | (0 << 4)};
const Register AM    = {0x60, 1 | (7 << 4)};
const Register DR    = {0x60, 5 | (0 << 4)};
const Register SR    = {0x70, 5 | (0 << 4)};
const Register SL    = {0x80, 4 | (4 << 4)};
const Register RR    = {0x80, 4 | (0 << 4)};
const Register SSGEG = {0x90, 4 | (0 << 4)};
const Register FNum1 = {0xA0, 8 | (0 << 4)};
const Register FNum2 = {0xA4, 3 | (0 << 4)};
const Register Block = {0xA4, 3 | (3 << 4)};
// These 3 have a byte for each operator.
// TODO: Figure out how to factor in an offset.
const Register Channel3FNum1 = {0xA8, 8 | (0 << 4)};
const Register Channel3FNum2 = {0xAC, 3 | (0 << 4)};
const Register Channel3Block = {0xAC, 3 | (3 << 4)};

const Register FB   = {0xB0, 3 | (3 << 4)};
const Register Algo = {0xB0, 3 | (0 << 4)};
const Register LR   = {0xB4, 2 | (6 << 4)};
const Register AMS  = {0xB4, 3 | (4 << 4)};
const Register PMS  = {0xB4, 3 | (0 << 4)};

void setupPins() {
    pinMode(pIC, OUTPUT);
    pinMode(pIRQ, OUTPUT);
    pinMode(pCS, OUTPUT);
    pinMode(pRD, OUTPUT);
    pinMode(pWR, OUTPUT);
    pinMode(pA0, OUTPUT);
    pinMode(pA1, OUTPUT);

    for (int i = 0; i < 8; i++) {
        pinMode(DataBus + i, INPUT);
    }
    
    digitalWrite(pIC, HIGH);
    digitalWrite(pIRQ, HIGH);
    digitalWrite(pCS, HIGH);
    digitalWrite(pRD, HIGH);
    digitalWrite(pWR, HIGH);
    digitalWrite(pA0, HIGH);
    digitalWrite(pA1, HIGH);
}

void setup() {
    setupPins();
}

void loop() {}

void something(byte instruction, byte d07) {
    
    static const byte address  = B10000;
    static const byte data     = B10000;
    static const byte bank0    = B00000;
    static const byte bank1    = B01000;
    static const byte write    = B00010;
    static const byte read     = B00100;
    static const byte inactive = B00111;
    static const byte invalid  = B00110;

    if (instruction % 2 | bitRead(instruction, 1) ^ bitRead(instruction, 2)) {
        // undefined behaviour (low) or inactive (high); go for inactive
        inactiveDataBus();
    } else {
        // active
        if (instruction & write) {
            // should be in write mode
            if (instruction & read) {
                // this shouldn't happen.
            }
            writeDataBus(bitRead(instruction, 3), bitRead(instruction, 4), d07);
        } else {
            // should be in read mode
            if (instruction & write) {
                // this shouldn't happen.
            }
            readDataBus();
        }
    }
}

void writeDataToAddress(bool bank, byte address, byte data) {
    // set address
    writeDataBus(bank, false, address);
    // wait?
    writeWaitWidth(false, address);
    // write data
    writeDataBus(bank, true, data);
}

void writeWaitWidth(bool fromAddressOrData, byte address) {
    if (fromAddressOrData) {
        if(address < 0x21 || address > 0xB6 ) {
            // undefined behaviour, panic!!!
        } else if (address < 0xA0) {
            // 83 cycles
        } else {
            // 47 cycles
        }
    } else {
        // 17 cycles
    }
}

void writeDataBus(bool bankA1, bool addressOrDataA0, byte input) {
    digitalWriteFast(pA0, A0);
    digitalWriteFast(pA1, A1);

    __asm__ __volatile__("NOPX 1"); // tAS, min 10ns
    digitalWriteFast(pCS, LOW);
    digitalWriteFast(pWR, LOW);
    for (int i = 0; i < 8; i++) {
        pinMode(pD0 + i, OUTPUT);
    }

    DataBus |= input;

    __asm__ __volatile__("NOPX 17"); // tCSW & tWW, min 200ns (kind of)
    digitalWriteFast(pWR, HIGH);
    digitalWriteFast(pCS, HIGH);

    __asm__ __volatile__("NOPX 1"); // tAH, min 10ns
    digitalWriteFast(pA0, LOW);
    digitalWriteFast(pA1, LOW);

    __asm__ __volatile__("NOPX 1"); // tWDH - tAH, min 10ns
    for (int i = 0; i < 8; i++) {
        pinMode(pD0 + i, INPUT);
    }
}

byte readDataBus() {
    byte status;
    digitalWriteFast(pA0, LOW);
    digitalWriteFast(pA1, LOW);

    __asm__ __volatile__("NOPX 1"); // tAS, min 10ns
    digitalWriteFast(pCS, LOW);
    digitalWriteFast(pRD, LOW);
    // no need to change mode for pins

    __asm__ __volatile__("NOPX 21"); // tACC, min 250ns (I think)

    status = DataBus;

    __asm__ __volatile__("NOPX 9"); // tCSR & tRW, min 100ns (kind of)
    digitalWriteFast(pRD, HIGH);
    digitalWriteFast(pCS, HIGH);

    __asm__ __volatile__("NOPX 1"); // tAH & tDH, min 10ns
    // no need to change mode for pins

    return status;
}

void inactiveDataBus() {
    if (portModeRegister(DataBus) || portOutputRegister(DataBus)) {
        for (int i = 0; i < 8; i++) {
            pinMode(DataBus + i, INPUT);
        }
    }

    digitalWriteFast(pCS, HIGH);
    digitalWriteFast(pRD, HIGH);
    digitalWriteFast(pWR, HIGH);
    digitalWriteFast(pA0, HIGH);
    digitalWriteFast(pA1, HIGH);
}

void reset() {
    digitalWriteFast(pIC, LOW);
    // Wait 192 cycles
    digitalWriteFast(pIC, HIGH);
    setupPins();
}

/*
void digitalWritePort(uint8_t pin, uint8_t val)
{
        uint8_t timer = digitalPinToTimer(pin);
        uint8_t port = digitalPinToPort(pin);
        volatile uint8_t *out;

        if (port == NOT_A_PIN) return;

        // If the pin that support PWM output, we need to turn it off
        // before doing a digital write.
        if (timer != NOT_ON_TIMER) turnOffPWM(timer);

        out = portOutputRegister(port);

        uint8_t oldSREG = SREG;
        cli();

        *out |= val;

        SREG = oldSREG;
}
*/

uint8_t digitalReadPort(uint8_t pin) {
    uint8_t timer = digitalPinToTimer(pin);
    uint8_t port  = digitalPinToPort(pin);

    if (port == NOT_A_PIN)
        return LOW;

    // If the pin that support PWM output, we need to turn it off
    // before getting a digital reading.
    // if (timer != NOT_ON_TIMER) turnOffPWM(timer);

    return *portInputRegister(port);
}

// https://arduino.stackexchange.com/questions/13165/how-to-read-pinmode-for-digital-pin
int pinMode(uint8_t pin) {
    if (pin >= NUM_DIGITAL_PINS)
        return (-1);

    uint8_t           bit  = digitalPinToBitMask(pin);
    uint8_t           port = digitalPinToPort(pin);
    volatile uint8_t *reg  = portModeRegister(port);
    if (*reg & bit)
        return (OUTPUT);

    volatile uint8_t *out = portOutputRegister(port);
    return ((*out & bit) ? INPUT_PULLUP : INPUT);
}