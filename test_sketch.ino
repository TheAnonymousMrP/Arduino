#include <SD.h>

#include "TestData/testdata.h"


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

#define pIC     PORTA + PA5 // 73
#define pIRQ    PORTA + PA6 // 72
#define pCS     PORTA + PA0 // 78
#define pRD     PORTA + PA1 // 77
#define pWR     PORTA + PA2 // 76
#define pA1     PORTA + PA3 // 75
#define pA0     PORTA + PA4 // 74
#define pD0     PORTC + PC0 // 53
#define DataBus PORTC       // 53-60

class PointerHelper {

    uint32_t       _length = 0;
    const uint8_t *_pointerSOF;
    const uint8_t *_pointer8;
    const uint8_t *_pointerEOF = _pointerSOF;

  public:
    PointerHelper(void){};
    // PointerHelper(PointerHelper* c) { return PointerHelper(c._pointerSOF, c._pointerEOF)};
    PointerHelper(const uint8_t *start, const uint8_t *end) {
        _pointerSOF = start;
        _pointer8   = start;
        _pointerEOF = end;
        _length     = end - start;
    };
    uint32_t length() { return _length; }
             operator bool() { return (_length > 0) && (_pointer8 < _pointerEOF); }

    const uint8_t * position() { return _pointer8; }

    bool readByte(uint8_t &buffer) {
        if (bool()) {
            buffer = *_pointer8++;
            return true;
        } else {
            return false;
        }
    }
    uint8_t readBytes(uint8_t &buffer, uint16_t nBytes) {
        uint8_t returned = 0U;
        for (returned; returned < nBytes; returned++) {
            if (!readByte(buffer)) {
                break;
            }
        }
        return returned;
    }

    bool skipBytes(uint16_t nBytes) {
        if (_pointer8 + nBytes < _pointerEOF) {
            _pointer8 += nBytes;
            return true;
        } else {
            return false;
        }
    }
} currentFile;

namespace VGM {
    const uint32_t SIXTY_SAMPLES  = 735;
    const uint32_t FIFTY_SAMPLES  = 882;
    const uint32_t OFFSET_DEFAULT = 0x0C;
    enum class Type {
        null,
        dataBlock, // is this not just a long sequence of Instructions??
        header,
        instruction,
        wait
    };

    struct Header {
        uint32_t fileID;                    // 0x00
        uint32_t offsetEOF;                 // 0x04
        uint32_t offsetStart;               // 0x34 
        uint32_t fileVersion;               // 0x08
        uint32_t sampleTotal;               // 0x18
        uint32_t loopOffset;                // 0x1C
        uint32_t loopTotal;                 // 0x20
        uint32_t frameRate;                 // 0x24
        uint32_t SN76489clock;              // 0x0C
        uint16_t SN76489feedback;           // 0x28
        uint16_t SN76489shiftRegisterWidth; // 0x2A
        uint8_t  SN76489flags;              // 0x2B
        uint32_t YM2612clock;               // 0x2C
    } currentHeader;
} // namespace VGM

namespace YM2612 {
    struct Write {
        // consider merging to a word?
        uint8_t address;
        uint8_t data;
        bool bank;
    } currentWrite;

    struct Register {
        // consider merging to a word?
        uint8_t address;
        uint8_t sizeAndOffset; // Offset << 4
    };

    struct Command {
        enum class Type { inactive, read, wait, write };
        Type type = Type::inactive;
        union {
            uint32_t wait;
            Write    write;
        };
    } currentCommand;

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
    // These next few have a uint8_t for each operator in each channel.
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
    // These 3 have a uint8_t for each operator.
    // TODO: Figure out how to factor in an offset.
    const Register Channel3FNum1 = {0xA8, 8 | (0 << 4)};
    const Register Channel3FNum2 = {0xAC, 3 | (0 << 4)};
    const Register Channel3Block = {0xAC, 3 | (3 << 4)};
    const Register FB            = {0xB0, 3 | (3 << 4)};
    const Register Algo          = {0xB0, 3 | (0 << 4)};
    const Register LR            = {0xB4, 2 | (6 << 4)};
    const Register AMS           = {0xB4, 3 | (4 << 4)};
    const Register PMS           = {0xB4, 3 | (0 << 4)};
}; // namespace YM2612





// I think the read versions are unnecessary??
// Keep for reading from files
uint8_t readRegister(uint8_t a, YM2612::Register r) {
    if ((r.sizeAndOffset & B00001111) == 8) {
        return a;
    } else if ((r.sizeAndOffset & B00001111) > 8) {
        return 0;
    } else {
        uint8_t temp = a >> (r.sizeAndOffset >> 4);
        return temp | (0xFF >> (r.sizeAndOffset & B00001111));
    }
}
uint8_t writeRegister(uint8_t a, YM2612::Register r) {
    if ((r.sizeAndOffset & B00001111) == 8) {
        return a;
    } else if ((r.sizeAndOffset & B00001111) > 8) {
        return 0;
    } else {
        return a << (r.sizeAndOffset >> 4);
    }
}
// TODO: Figure out if these are backwards?? Endianness etc.
word readTimerA(uint8_t a, uint8_t b) { return ((word)a << 2) | (b & B00000011); }
word writeTimerA(word a) {
    uint8_t b = (uint8_t)a & B00000011;
    return ((a >> 2) << 8 | b);
}
word readFNum(uint8_t a, uint8_t b) { return a | ((b & B00000111) << 8); }
word writeFNum(word a) {
    word b = (a >> 8) & B00000111;
    return (a << 8 | b);
}

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
    // TODO: Remove file-setup from arduino-setup
    currentFile = PointerHelper(testFile.data, testFile.data + sizeof(testFile.data));
    VGM::currentHeader = readVGMHeader((const uint32_t *)currentFile.position());
    currentFile.skipBytes(0x34 + VGM::currentHeader.offsetStart);
}

void loop() {


}

/* void something(uint8_t instruction, uint8_t d07) {
    static const uint8_t address  = B10000;
    static const uint8_t data     = B10000;
    static const uint8_t bank0    = B00000;
    static const uint8_t bank1    = B01000;
    static const uint8_t write    = B00010;
    static const uint8_t read     = B00100;
    static const uint8_t inactive = B00111;
    static const uint8_t invalid  = B00110;

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
*/

void writeDataToAddress(YM2612::Command newCommand) {
    bool processAddress = true;
    // check address against last instruction if appropriate
    if (YM2612::currentCommand.type == YM2612::Command::Type::write) {
        processAddress != newCommand.write.bank ==
            YM2612::currentCommand.write.bank &&newCommand.write.address ==
            YM2612::currentCommand.write.address;
    } else if (YM2612::currentCommand.type == YM2612::Command::Type::wait) {
        processAddress != newCommand.write.bank ==
            YM2612::currentWrite.bank &&newCommand.write.address ==
            YM2612::currentWrite.address;
    }

    if (processAddress) {
        writeDataBus(newCommand.write.bank, false, newCommand.write.address);
        writeWaitWidth(false, true, newCommand.write.address);
    }
    writeDataBus(newCommand.write.bank, true, newCommand.write.data);
    YM2612::currentCommand = newCommand;
    YM2612::currentWrite   = newCommand.write;
}

void writeDataBus(bool bankA1, bool addressOrDataA0, uint8_t input) {
    digitalWriteFast(pA0, addressOrDataA0);
    digitalWriteFast(pA1, bankA1);

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

void writeWaitWidth(bool fromAddressOrData, bool toAddressOrData,
                    uint8_t address) {
    if (fromAddressOrData) {
        if (address < 0x21 || address > 0xB6) {
            // undefined behaviour, panic!!!
        } else if (address < 0xA0) {
            // 83 cycles
        } else {
            // 47 cycles
        }
    } else if (!fromAddressOrData && !toAddressOrData) {
        // undefined behaviour, panic!!!
    } else {
        // 17 cycles
    }
}

uint8_t readDataBus() {
    uint8_t status;
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
 
VGM::Header readVGMHeader(const uint32_t *pointerToFileAddress) {
    VGM::Header temp;
    temp.fileID       = *pointerToFileAddress++;    // 0x00++
    temp.offsetEOF    = *pointerToFileAddress++;    // 0x04++
    temp.fileVersion  = *pointerToFileAddress++;    // 0x08++
    temp.SN76489clock = *pointerToFileAddress++;    // 0x0C++
    pointerToFileAddress += 2;                      // 0x10+8
    temp.sampleTotal     = *pointerToFileAddress++; // 0x18++
    temp.loopOffset      = *pointerToFileAddress++; // 0x1C++
    temp.loopTotal       = *pointerToFileAddress++; // 0x20++
    temp.frameRate       = *pointerToFileAddress++; // 0x24++
    uint32_t SN76478temp = *pointerToFileAddress++; // 0x28++
    temp.YM2612clock     = *pointerToFileAddress++; // 0x2C++
    pointerToFileAddress++;                         // 0x30+4

    if (temp.fileVersion < 0x00000150) {
        temp.offsetStart = VGM::OFFSET_DEFAULT;
    } else {
        temp.offsetStart = *pointerToFileAddress; // 0x34
    }

    temp.SN76489feedback           = SN76478temp >> 16;
    temp.SN76489shiftRegisterWidth = SN76478temp >> 8;
    temp.SN76489flags              = SN76478temp;

    return temp;
}

YM2612::Command readVGMCommand(PointerHelper fileP,
                               VGM::Header header = VGM::currentHeader) {
    YM2612::Command tempCommand;
    bool success = false;
    uint8_t tempUINT8;
    uint32_t        tempUINT32;
    enum CommandType : uint8_t {
        writeYM26120   = 0x52,
        writeYM26121   = 0x53,
        waitN          = 0x61,
        wait60th       = 0x62,
        wait50th       = 0x63,
        endofSoundData = 0x66,
        writeDataBlock = 0x67,
        waitNplus1     = 0x70,
        writeYM2612PCM = 0x80,
        offsetPCM      = 0xE0
    };
    uint8_t test;
    if (VGM::readByte(test, fileP)) {
        // this all seems dodgy as hell. At least no data can be changed??
        switch (test) {
        case writeYM26120:
        case writeYM26121:
            tempCommand.type = YM2612::Command::Type::write;

            if (VGM::readByte(tempUINT8, fileP) ||
                VGM::readByte(tempCommand.write.address, fileP) ||
                VGM::readByte(tempCommand.write.data, fileP)) {
                success = true;
                tempCommand.write.bank = tempUINT8 - 0x52;
            }
            break;
        case waitN ... wait50th:
            if (test == waitN) {
                VGM::readByte(tempUINT8, fileP);
                tempUINT32 = tempUINT8;
                VGM::readByte(tempUINT8, fileP);
                tempUINT32 += (uint32_t)tempUINT8 << 8;
            } else if (test == wait60th) {
                tempUINT32 = VGM::SIXTY_SAMPLES;
            } else if (test == wait50th) {
                tempUINT32 = VGM::FIFTY_SAMPLES;
            }
            tempCommand.type = YM2612::Command::Type::wait;
            tempCommand.wait = tempUINT32;
            break;
        case waitNplus1 ... waitNplus1 + 0x0F:
            tempCommand.type = YM2612::Command::Type::wait;
            VGM::readByte(tempUINT8, fileP);
            tempCommand.wait = tempUINT8 - waitNplus1;
            break;
        case writeDataBlock: {
            // No idea how to handle 4mB potential data.
            fileP.pointer8 += 2;
            uint8_t  typeOfDataBlock = *fileP.pointer8++;
            uint32_t blockLength;
            blockLength += *fileP.pointer8++;
            blockLength += (uint32_t)*fileP.pointer8++ << 8;
            blockLength += (uint32_t)*fileP.pointer8++ << 16;
            blockLength += (uint32_t)*fileP.pointer8++ << 24;

            // if(file.pointer8 + blockLength >=)

            if (typeOfDataBlock == 0x00) {
                // PANIC!!!
                fileP.pointer8 += blockLength; // not somebody else's problem :(
            } else {
                // can't ignore it :(
                fileP.pointer8 += blockLength;
            }
            break;
        }
        case offsetPCM:
            // Panic!!!
            // First four uint8_ts define offset from PCM block (same as DAC
            // Data??) Each subsequent 0x8n plays a sample and steps ahead n
            // sample
        case endofSoundData:
            // some kind of flag?
        default:
            // VGMType::null;
            break;
        }
    };

    // if we've overflowed the file the command can't be trusted to be valid
    if(fileP.pointer8 >= currentFile.pointerEOF || success == false) {
        tempCommand.type = YM2612::Command::Type::inactive;
    }

    return tempCommand;
}