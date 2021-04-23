#include <MIDI.h>
#include <SD.h>

#include "Test/testdata.h"

#ifndef _ym2612_h_
#include "ym2612.h"
#endif

#ifndef _sn76489_h_
#include "sn76489.h"
#endif

#ifndef _digitalWriteFast_h_
#include <digitalWriteFast.h>
#endif

#define SOMETHING true

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

// Physical Control Pins
#define CON_MEM_PORT PORTK
#define CON_BIT_INT  bitRead(PCIFR,PCIF2) // not sure this is necessary
#define ENC_OFF_1A   PK0    // Also PCINT16
#define ENC_OFF_1B   PK1    // Also PCINT17
#define ENC_PIN_1A   89U    // PK0
#define ENC_PIN_1B   88U    // PK1

#define BLOCKSIZE       0x100U
#define COMMANDPILESIZE 0x80U
#define READATTEMPTS    5U

uint8_t digitalReadPort(uint8_t pin) {
    uint8_t timer = digitalPinToTimer(pin);
    uint8_t port  = digitalPinToPort(pin);

    if (port == NOT_A_PIN) return LOW;

    // If the pin that support PWM output, we need to turn it off
    // before getting a digital reading.
    // if (timer != NOT_ON_TIMER) turnOffPWM(timer);

    return *portInputRegister(port);
}

// https://arduino.stackexchange.com/questions/13165/how-to-read-pinmode-for-digital-pin
int pinMode(uint8_t pin) {
    if (pin >= NUM_DIGITAL_PINS) return (-1);

    uint8_t            bit  = digitalPinToBitMask(pin);
    uint8_t            port = digitalPinToPort(pin);
    volatile uint8_t * reg  = portModeRegister(port);
    if (*reg & bit) return (OUTPUT);

    volatile uint8_t * out = portOutputRegister(port);
    return ((*out & bit) ? INPUT_PULLUP : INPUT);
}

// consider swapping with CircularBuffer library
class BlockHandler {
    uint8_t   dataBlockRead[BLOCKSIZE];
    uint8_t   dataBlockWrite[BLOCKSIZE];
    uint8_t * _pointer8Read  = &dataBlockRead[0];
    uint8_t * _pointer8Write = &dataBlockWrite[0];
    uint8_t * _eofRead       = &dataBlockRead[BLOCKSIZE];
    uint8_t * _eofWrite      = &dataBlockWrite[BLOCKSIZE];
    bool      _locked        = false;

  public:
    // I'm sure there's a way to do this by swapping pointers and stuff
    // BlockHandler() { dataBlock = new uint8_t[BLOCKSIZE]; }
    // ~BlockHandler() { delete[] dataBlock; };

    bool isReadFinished(const uint16_t afterNBytes = 0) const {
        return _pointer8Read + afterNBytes >= _eofRead;
    }
    bool isWriteFinished(const uint16_t afterNBytes = 0) const {
        return _pointer8Write + afterNBytes >= _eofWrite;
    }
    bool isReadEmpty() const { return (lengthRead()) ? false : true; }
    bool isWriteLocked() const { return _locked; }
    // TODO: Include afterNBytes
    bool canRead() const { return !isReadEmpty() && !isReadFinished(); }
    bool canWrite() const { return !isWriteLocked() && !isWriteFinished(); }
    bool readyForNextLoad() const {
        return _pointer8Read >= &dataBlockRead[BLOCKSIZE / 2];
    }
    bool readyForNextSwap() const {
        return isWriteLocked() && isWriteFinished() && isReadFinished();
    }

    const uint16_t lengthRemainingRead() const { return _eofRead - _pointer8Read; }
    const uint16_t lengthRemainingWrite() const { return _eofRead - _pointer8Write; }

    const uint16_t lengthRead() const { return _eofRead - dataBlockRead; }
    const uint16_t lengthWrite() const { return _eofWrite - dataBlockRead; }

    void lockWrite() {
        if (_pointer8Write < _eofWrite) { _eofWrite = _pointer8Write; }
        _locked = true;
    }

    void swapWriteToRead() {
        memcpy(dataBlockRead, dataBlockWrite, sizeof(dataBlockWrite));
        _pointer8Read = &dataBlockRead[0];
        _eofRead      = &dataBlockRead[0] + lengthWrite();
        memset(dataBlockWrite, 0U, sizeof(dataBlockWrite));
        _pointer8Write = &dataBlockWrite[0];
        _eofWrite      = &dataBlockWrite[BLOCKSIZE];
    }

    // Will always skip & return 0 or nBytes, which probably isn't ideal.
    const uint16_t skipBytesWrite(uint16_t nBytes) {
        if (!isWriteLocked() && isWriteFinished(nBytes)) {
            uint8_t skipped = 0;
            for (skipped; skipped < nBytes; skipped++) {
                *_pointer8Write++ = 0x00U;
            }
            return skipped;
        } else { return 0U; }
    }

    bool writeByte(const uint8_t byte) {
        if (!isWriteLocked() && !isWriteFinished()) {
            *_pointer8Write++ = byte;
            return true;
        } else {
            return false;
        }
    }

    uint8_t writeBytes(const uint8_t * buffer, uint8_t nBytes) {
        if (!isWriteLocked() && !isWriteFinished()) {
            uint8_t written = 0U;
            for (written; written < nBytes; written++) {
                if (!writeByte(*buffer++)) { break; };
            }
            return written;
        } else {
            return 0U;
        }
    }

    bool readByte(uint8_t & buffer) {
        if (!isReadEmpty() && !isReadFinished()) {
            buffer = *_pointer8Read++;
            return true;
        } else {
            return false;
        }
    }
    bool read16(uint16_t & buffer) {
        if (!isReadEmpty() && !isReadFinished(sizeof(uint16_t))) {
            buffer = *_pointer8Read++;
            buffer += *_pointer8Read++ << 8U;
            return true;
        } else {
            return false;
        }
    }
    bool read24(uint32_t & buffer) {
        if (!isReadEmpty() && !isReadFinished(sizeof(uint32_t))) {
            buffer = *_pointer8Read++;
            buffer += *_pointer8Read++ << 8U;
            buffer += *_pointer8Read++ << 16U;
            return true;
        } else {
            return false;
        }
    }
    bool read32(uint32_t & buffer) {
        if (!isReadEmpty() && !isReadFinished(sizeof(uint32_t))) {
            buffer = *_pointer8Read++;
            buffer += *_pointer8Read++ << 8U;
            buffer += *_pointer8Read++ << 16U;
            buffer += *_pointer8Read++ << 24U;
            return true;
        } else {
            return false;
        }
    }

    // Not appropriate for reading long numbers, use relevant readXX instead
    bool readBytes(uint8_t * buffer, uint16_t nBytes) {
        if (!isReadEmpty() && !isReadFinished(nBytes)) {
            for (uint8_t i = 0U; i < nBytes; i++) {
                *buffer++ = *_pointer8Read++;
            }
        }
    }
    
    // To avoid writing peek methods, rollback one byte
    bool rollbackRead() {
        if (_pointer8Read > &dataBlockRead[0]) {
            _pointer8Read--;
            return true;
        } else {
            return false;
        }
    }

    const bool skipBytesRead(uint16_t nBytes) {
        if (!isReadEmpty() && !isReadFinished(nBytes)) {
            _pointer8Read += nBytes;
            return true;
        } else {
            return false;
        }
    }

    // For 'wiping' a block without reading its contents.
    // For large data-related instructions which cannot be processed.
    void skipBlock() {
        _pointer8Read = _eofRead;
    }
};

namespace VGM {
    uint32_t operator"" _samples(unsigned long long);

    const uint32_t OFFSET_DEFAULT = 0x0C;
    const uint16_t SIXTY_SAMPLES  = 735_samples;
    const uint16_t FIFTY_SAMPLES  = 882_samples;

    enum CommandType : uint8_t {
        T_writePSG         = 0x4F,
        T_writeSN76489     = 0x50,
        T_writeYM2413      = 0x51,
        T_writeYM2612port0 = 0x52,
        T_writeYM2612port1 = 0x53,
        T_writeYM2151      = 0x54,
        T_writeYM2203      = 0x55,
        T_writeYM2608port0 = 0x56,
        T_writeYM2608port1 = 0x57,
        T_writeYM2610port0 = 0x58,
        T_writeYM2610port1 = 0x59,
        T_writeYM3812      = 0x5A,
        T_writeYM3526      = 0x5B,
        T_writeY8950       = 0x5C,
        T_writeYMZ280B     = 0x5D,
        T_writeYMF262port0 = 0x5E,
        T_writeYMF262port1 = 0x5F,
        T_waitN            = 0x61,
        T_wait60th         = 0x62,
        T_wait50th         = 0x63,
        T_endofSoundData   = 0x66,
        T_writeDataBlock   = 0x67,
        T_writePCMRAM      = 0x68,
        T_waitNplus1       = 0x70,
        T_writeDataYM2612  = 0x80,
        T_DACSetup         = 0x90,
        T_DACData          = 0x91,
        T_DACFrequency     = 0x92,
        T_DACStart         = 0x93,
        T_DACStop          = 0x94,
        T_DACStartFastCall = 0x95,
        T_offsetPCM        = 0xE0
    };

    enum CommandLength : uint8_t {
        L_writePSG         = 1U,
        L_writeSN76489     = 1U,
        L_writeYM2413      = 2U,
        L_writeYM2612port0 = 2U,
        L_writeYM2612port1 = 2U,
        L_writeYM2151      = 2U,
        L_writeYM2203      = 2U,
        L_writeYM2608port0 = 2U,
        L_writeYM2608port1 = 2U,
        L_writeYM2610port0 = 2U,
        L_writeYM2610port1 = 2U,
        L_writeYM3812      = 2U,
        L_writeYM3526      = 2U,
        L_writeY8950       = 2U,
        L_writeYMZ280B     = 2U,
        L_writeYMF262port0 = 2U,
        L_writeYMF262port1 = 2U,
        L_waitN            = 2U,
        L_wait60th         = 0U,
        L_wait50th         = 0U,
        L_endofSoundData   = 0U,
        L_writeDataBlock   = 6U,
        L_writePCMRAM      = 11U,
        L_waitNplus1       = 0U,
        L_writeDataYM2612  = 0U,
        L_DACSetup         = 4U,
        L_DACData          = 4U,
        L_DACFrequency     = 5U,
        L_DACStart         = 10U,
        L_DACStop          = 1U,
        L_DACStartFastCall = 4U,
        L_offsetPCM        = 4U
    };

    struct Header {
        // TODO: Figure out if any of these can be union'd
        uint32_t fileID;               // 0x00
        uint32_t offsetEOF;            // 0x04
        uint32_t offsetStart;          // 0x34
        uint32_t fileVersion;          // 0x08
        uint32_t sampleTotal;          // 0x18
        uint32_t loopOffset;           // 0x1C
        uint32_t loopTotal;            // 0x20
        uint32_t frameRate;            // 0x24
        uint32_t YM2612clock;          // 0x2C
        uint32_t SN76489clock;         // 0x0C
        uint16_t SN76489feedback;      // 0x28
        uint8_t  SN76489shiftRegWidth; // 0x2A
        uint8_t  SN76489flags;         // 0x2B
        bool     isValid = false;
    } currentHeader;

    struct Instruction {
        CommandType type;
        uint8_t     bytesNeeded = 0U;
        uint8_t     progress    = 0U; // should this be a pointer instead?
        uint8_t     operands[11U];
    };

    const VGM::Header readVGMHeader(BlockHandler & fileP) {
        VGM::Header tHead; // .isValid is implicitly false;
        uint32_t    SN76478temp;

        if (!fileP.read32(tHead.fileID)) { return tHead; }          // 0x00++
        if (!fileP.read32(tHead.offsetEOF)) { return tHead; }       // 0x04++
        if (!fileP.read32(tHead.fileVersion)) { return tHead; }     // 0x08++
        if (!fileP.read32(tHead.SN76489clock)) { return tHead; }    // 0x0C++
        if (!fileP.skipBytesRead(8)) { return tHead; }              // 0x10+8
        if (!fileP.read32(tHead.sampleTotal)) { return tHead; }     // 0x18++
        if (!fileP.read32(tHead.loopOffset)) { return tHead; }      // 0x1C++
        if (!fileP.read32(tHead.loopTotal)) { return tHead; }       // 0x20++
        if (!fileP.read32(tHead.frameRate)) { return tHead; }       // 0x24++
        if (!fileP.read16(tHead.SN76489feedback)) { return tHead; } // 0x28++
        if (!fileP.readByte(tHead.SN76489shiftRegWidth)) {
            return tHead;
        }                                                          // 0x2A++
        if (!fileP.readByte(tHead.SN76489flags)) { return tHead; } // 0x2B++
        if (!fileP.read32(tHead.YM2612clock)) { return tHead; }    // 0x2C++
        if (!fileP.skipBytesRead(4)) { return tHead; }             // 0x30+4

        if (tHead.fileVersion < 0x00000150) {
            tHead.offsetStart = VGM::OFFSET_DEFAULT;
        } else {
            if (!fileP.read32(tHead.offsetStart)) { return tHead; } // 0x30+4
        }

        if (!fileP.skipBytesRead(tHead.offsetStart)) {
            return tHead;
        } // end of header

        tHead.isValid = true;

        return tHead;
    }

    // @return TRUE if complete instruction is read from block, FALSE otherwise
    const bool readVGMInstruction(Instruction &  buffer,
                                  BlockHandler & vgmBlocks,
                                  Header         header = currentHeader) {
        uint8_t read;

        if (buffer.bytesNeeded == 0xFFU) {
            // Data block; can't handle currently so this skips bytes.
            uint16_t remaining = vgmBlocks.lengthRemainingRead();
            auto & temp = *(uint32_t *)buffer.operands;
            if(temp >= (uint32_t)remaining) {
                // entire block can be dismissed
                vgmBlocks.skipBlock();
                return false;
            } else {
                // temp must now be smaller than a uint16_t?
                // Smaller than BLOCKSIZE, even
                vgmBlocks.skipBytesRead(temp);
                buffer.bytesNeeded = 0U;
                return true;
            }
        } else if (buffer.bytesNeeded != 0) {
            // finish previous instruction
            uint16_t remaining = vgmBlocks.lengthRemainingRead();
            if (buffer.bytesNeeded <= remaining) {
                vgmBlocks.readBytes(buffer.operands + buffer.progress,
                                    buffer.bytesNeeded);
                if (buffer.type == CommandType::T_writeDataBlock) {
                    buffer.bytesNeeded = 0xFFU;
                    uint32_t temp = buffer.operands[2];
                    temp += buffer.operands[3] << 8U;
                    temp += buffer.operands[4] << 16U;
                    temp += buffer.operands[5] << 32U;
                    *(uint32_t *)buffer.operands = temp; // this feels dodgy af
                    return false;
                } else {
                    buffer.bytesNeeded = 0;
                    return true;
                }
            } else {
                // partial
                while (buffer.bytesNeeded >= 0U) {
                    if (vgmBlocks.readByte(buffer.operands[buffer.progress])) {
                        buffer.progress++;
                        buffer.bytesNeeded--;
                    } else {
                        return false;
                    }
                }
                return true; // this shouldn't happen
            }
        } else {
            // new instruction
            if (vgmBlocks.readByte(read)) {
                uint16_t remaining = vgmBlocks.lengthRemainingRead();
                uint8_t  length    = 0U;
                buffer.type        = (CommandType)read;

                switch (read) {
                // length 0
                case CommandType::T_wait60th:
                case CommandType::T_wait50th:
                case CommandType::T_endofSoundData:
                case CommandType::T_waitNplus1: return true; break;

                // would like to still do the consolidation thing
                case T_writeDataYM2612 ... T_writeDataYM2612 + 0x0FU:
                    return true;
                    break;

                // length 1
                case CommandType::T_writePSG:
                case CommandType::T_writeSN76489:
                case CommandType::T_DACStop:
                case 0x30 ... 0x3F: // reserved
                    length = 1U;
                    break;

                // length 2
                case CommandType::T_writeYM2413... CommandType::
                    T_writeYMF262port1:
                case CommandType::T_waitN:
                case 0xA1 ... 0xAF: // reserved
                    length = 2U;
                    break;

                // length 3
                case 0xC0 ... 0xC8:
                case 0xC9 ... 0xCF: // reserved
                case 0xD0 ... 0xD6:
                case 0xD7 ... 0xDF: // reserved
                    length = 3U;
                    break;

                // length 4
                case CommandType::T_DACSetup:
                case CommandType::T_DACData:
                case CommandType::T_DACStartFastCall:
                case CommandType::T_offsetPCM:
                case 0xE1:
                case 0xE2 ... 0xFF: // reserved
                    length = 4U;
                    break;

                // other lengths
                case CommandType::T_writePCMRAM:
                    length = CommandLength::L_writePCMRAM;
                    break;
                case CommandType::T_DACStart:
                    length = CommandLength::L_DACStart;
                    break;
                case CommandType::T_DACFrequency:
                    length = CommandLength::L_DACFrequency;
                    break;

                case 0x40 ... 0x4E: // reserved, one operand < v1.60
                    length = (header.fileVersion >= 0x00000160) ? 2U : 1U;
                    break;

                case CommandType::T_writeDataBlock: 
                    length = CommandLength::L_writeDataBlock;
                    break; // 5 + var

                default: return false; // undefined type
                }

                if (length == 0U) {
                    return true; // this shouldn't happen (previous return)
                } else if (length <= remaining) {
                    vgmBlocks.readBytes(buffer.operands, length);
                    if(buffer.type == CommandType::T_writeDataBlock) {
                        buffer.bytesNeeded = 0xFFU;
                        uint32_t temp      = buffer.operands[2];
                        temp += buffer.operands[3] << 8U;
                        temp += buffer.operands[4] << 16U;
                        temp += buffer.operands[5] << 32U;
                        // this feels dodgy af
                        *(uint32_t *)buffer.operands = temp;
                        return false;
                    } else {
                        return true;
                    }
                } else {
                    // partial
                    buffer.bytesNeeded = length;
                    while (buffer.bytesNeeded >= 0U) {
                        if (vgmBlocks.readByte(
                                buffer.operands[buffer.progress])) {
                            buffer.progress++;
                            buffer.bytesNeeded--;
                        } else {
                            return false;
                        }
                    }
                    return true; // this shouldn't happen
                }
            } else {
                return false;
            } // block finished, no type read
        }
    };

    void testSize() { sizeof(currentHeader); }
} // namespace VGM

struct Command {
    enum class Type : uint8_t {
        null,
        ignore,
        settings,
        wait,
        sn76489_write,
        ym2612_inactive,
        ym2612_read,
        ym2612_write
    } type = Type::null;
    union {
        // uint32_t       pcmOffset;
        uint16_t       wait;
        YM2612::Write  ymWrite;
        SN76489::Write snWrite;
    };
};

class Command2 {
  public:
    enum class Type : uint8_t {
        null,
        ignore,
        settings,
        wait,
        sn76489_write,
        ym2612_inactive,
        ym2612_read,
        ym2612_write
    };

    union {
        // uint32_t       pcmOffset;
        uint16_t       wait;
        YM2612::Write  ymWrite;
        SN76489::Write snWrite;
    };

  private:
    Type type = Type::null;

  public:
    const Type & getType() const {
        return type;
    };

    // auto getData() const {
    //     switch(type) {
    //         case Type::wait: return data.wait;
    //         case Type::sn76489_write: return data.ymWrite;
    //         case Type::ym2612_write: return data.snWrite;

    //         default: return false;
    //     }
    // }

    void setWait(const uint16_t & newWait) {
        type = Type::wait;
        wait = newWait;
    };
    void setSNWrite(const SN76489::Write & newWrite) {
        type = Type::sn76489_write;
        snWrite = newWrite;
    };
    void setYMWrite(const YM2612::Write & newWrite) {
        type = Type::ym2612_write;
        ymWrite = newWrite;
    };

    const bool getWait(uint16_t * const buffer) const {
        if (type != Type::wait) {
            return false;
        } else {
            *buffer = wait;
            return true;
        }
    }
};

// Consider replacing with CircularBuffer library
class CommandPile {
    // const commands?
    Command         _pile[COMMANDPILESIZE];
    const Command * _currentCommandToRead;
    Command *       _currentCommandToWrite;
    uint8_t         _commandCount;
    // TODO: This will break if COMMANDPILESIZE exceeds uint8_t

  public:
    CommandPile()
        : _currentCommandToRead(_pile), _currentCommandToWrite(_pile),
          _commandCount(0){ };

    const bool addToPile(const Command & newCommand) {
        // can this overflow unintentionally?
        if (_commandCount < COMMANDPILESIZE &&
            newCommand.type != Command::Type::null) {
            // is this right??
            if (_currentCommandToWrite >= _pile + sizeof(_pile)) {
                _currentCommandToWrite = _pile;
            }
            _commandCount++;
            *_currentCommandToWrite++ = newCommand;
        } else {
            return false;
        }
    }

    const uint8_t getCount() { return _commandCount; }

    // retrieves next Command, 'removing' it from the pile.
    const bool takeFromPile(Command & buffer) {
        if (_commandCount) {
            if (_currentCommandToRead >= _pile + sizeof(_pile)) {
                _currentCommandToRead = _pile;
            }
            _commandCount--;
            buffer = *_currentCommandToRead++;
            return true;
        } else {
            return false;
        }
    }
} currentCommandPile;

const Command
convertVGMInstructionToCommand(VGM::Instruction & instruction,
                               VGM::Header &      header = VGM::currentHeader) {
    using namespace VGM;
    Command tempCommand;

    switch (instruction.type) {
        // waits
    case CommandType::T_waitN: 
        tempCommand.type = Command::Type::wait;
        tempCommand.wait =
            instruction.operands[0] + (instruction.operands[1] << 8);
        break;
    case T_wait60th:
        tempCommand.type = Command::Type::wait;
        tempCommand.wait = VGM::SIXTY_SAMPLES;
        break;
    case T_wait50th:
        tempCommand.type = Command::Type::wait;
        tempCommand.wait = VGM::FIFTY_SAMPLES;
        break;
    case T_waitNplus1 ... T_waitNplus1 + 0x0FU:
        tempCommand.type = Command::Type::wait;
        tempCommand.wait = instruction.type - T_waitNplus1 + 1;
        break;
    case T_writeDataYM2612 ... T_writeDataYM2612 + 0x0FU:
        // write one byte of PCM data to YM2612 DAC Data (0x2A)
        // And then wait n samples. For now, just a wait.
        tempCommand.type = Command::Type::wait;
        tempCommand.wait = instruction.type & B00001111;
        break;    

        // writes
    case CommandType::T_writePSG:
        tempCommand.type = Command::Type::ignore;
        break;
    case CommandType::T_writeSN76489:
        tempCommand.type    = Command::Type::sn76489_write;
        tempCommand.snWrite = instruction.operands[0];
        break;
    case CommandType::T_writeYM2612port0:
    case CommandType::T_writeYM2612port1:
        tempCommand.type = Command::Type::ym2612_write;
        tempCommand.ymWrite.bank =
            (instruction.type == T_writeYM2612port1) ? true : false;
        tempCommand.ymWrite.address = (YM2612::RegisterAddress)instruction.operands[0];
        tempCommand.ymWrite.data    = instruction.operands[1];
        break;

    case T_endofSoundData:
        // some kind of flag?
        tempCommand.type = Command::Type::ym2612_inactive;
        break;

    // not yet handled
    case CommandType::T_writePCMRAM:
        tempCommand.type = Command::Type::ignore;
        break;
    case CommandType::T_writeDataBlock: // no need for actual command now
        tempCommand.type = Command::Type::ignore;
        break;
    case CommandType::T_offsetPCM: tempCommand.type = Command::Type::ignore;

        // ignores
    case 0x30 ... 0x3F: // reserved    
    case 0x40 ... 0x4E: // reserved, one operand < v1.60
    case CommandType::T_writeYM2413:
    case CommandType::T_writeYM2151 ... CommandType::T_writeYMF262port1:
    case CommandType::T_DACSetup:
    case CommandType::T_DACData:
    case CommandType::T_DACFrequency:
    case CommandType::T_DACStart:
    case CommandType::T_DACStop:
    case CommandType::T_DACStartFastCall:
    case 0xA1 ... 0xAF: // reserved        
    case 0xC0 ... 0xC8:
    case 0xC9 ... 0xCF: // reserved
    case 0xD0 ... 0xD6:
    case 0xD7 ... 0xDF: // reserved
    case 0xE1:
    case 0xE2 ... 0xFF: // reserved
    default: // should probably log these
        tempCommand.type = Command::Type::null;
        break;
    }

    return tempCommand;
}

/* const Command readVGMCommand(BlockHandler & vgmBlocks,
                             VGM::Header    header = VGM::currentHeader) {
    // it is probably Very Bad to just copy and paste an enum like this
    enum CommandType : uint8_t {
        T_writePSG         = 0x4F,
        T_writeSN76489     = 0x50,
        T_writeYM2413      = 0x51,
        T_writeYM2612port0 = 0x52,
        T_writeYM2612port1 = 0x53,
        T_writeYM2151      = 0x54,
        T_writeYM2203      = 0x55,
        T_writeYM2608port0 = 0x56,
        T_writeYM2608port1 = 0x57,
        T_writeYM2610port0 = 0x58,
        T_writeYM2610port1 = 0x59,
        T_writeYM3812      = 0x5A,
        T_writeYM3526      = 0x5B,
        T_writeY8950       = 0x5C,
        T_writeYMZ280B     = 0x5D,
        T_writeYMF262port0 = 0x5E,
        T_writeYMF262port1 = 0x5F,
        T_waitN            = 0x61,
        T_wait60th         = 0x62,
        T_wait50th         = 0x63,
        T_endofSoundData   = 0x66,
        T_writeDataBlock   = 0x67,
        T_writePCMRAM      = 0x68,
        T_waitNplus1       = 0x70,
        T_writeDataYM2612  = 0x80,
        T_offsetPCM        = 0xE0
    };
    
    // these are bytes AFTER the type byte; consider revising
    // variable length is FF
    // This could potentially be condensed given length is consistent within ranges
    enum CommandLength : uint8_t {
        L_writePSG         = 0x01,
        L_writeSN76489     = 0x01,
        L_writeYM2413      = 0x02,
        L_writeYM2612port0 = 0x02,
        L_writeYM2612port1 = 0x02,
        L_writeYM2151      = 0x02,
        L_writeYM2203      = 0x02,
        L_writeYM2608port0 = 0x02,
        L_writeYM2608port1 = 0x02,
        L_writeYM2610port0 = 0x02,
        L_writeYM2610port1 = 0x02,
        L_writeYM3812      = 0x02,
        L_writeYM3526      = 0x02,
        L_writeY8950       = 0x02,
        L_writeYMZ280B     = 0x02,
        L_writeYMF262port0 = 0x02,
        L_writeYMF262port1 = 0x02,
        L_waitN            = 0x02,
        L_wait60th         = 0x00,
        L_wait50th         = 0x00,
        L_endofSoundData   = 0x00,
        L_writeDataBlock   = 0xFF,
        T_writePCMRAM      = 0xFF,
        L_waitNplus1       = 0x00,
        L_writeDataYM2612  = 0x00,
        L_offsetPCM        = 0x04
    };

    uint32_t tempUINT32;
    uint8_t  tempUINT8;
    uint8_t  read;
    Command  tempCommand;

    if (vgmBlocks.readByte(read)) {
        uint16_t remaining = vgmBlocks.lengthRemainingRead();
        // this all seems dodgy as hell. At least no data can be changed??
        switch (read) {
            // defined types to be processed
            // length 0
        
        case T_wait60th:
            tempCommand.wait = VGM::SIXTY_SAMPLES;
            tempCommand.type = Command::Type::wait;
            break;
        case T_wait50th:
            tempCommand.wait = VGM::FIFTY_SAMPLES;
            tempCommand.type = Command::Type::wait;
            break;
        case T_endofSoundData:
            // some kind of flag?
        case T_waitNplus1 ... T_waitNplus1 + 0x0FU:
            tempCommand.wait = read - T_waitNplus1 + 1;
            tempCommand.type = Command::Type::wait;
            break;
            

        case T_writeDataYM2612 ... T_writeDataYM2612 + 0x0FU: {
            // write one byte of PCM data to YM2612 DAC Data (0x2A)
            // And then wait n samples.

            // For now, just treat as a wait. To condense multiple waits,
            // read ahead until end of block or non-0xEn byte.
            tempCommand.wait = read & B00001111;
            bool isValid     = true;
            while (isValid && vgmBlocks.readByte(tempUINT8)) {
                if (tempUINT8 & 0xF0 == T_writeDataYM2612) {
                    tempCommand.wait += (tempUINT8 & B00001111);
                } else {
                    isValid = false;
                    vgmBlocks.rollbackRead();
                }
            }
            tempCommand.type = Command::Type::wait;
        }
        
        // length 1
        case T_writePSG:
            if(!vgmBlocks.skipBytesRead(1)) {
                // need 1 byte from next block
            };
            tempCommand.type = Command::Type::ignore;
            break;
        case T_writeSN76489:
            if(!vgmBlocks.skipBytesRead(1)) {
                // need 1 byte from next block
            };
            // tempCommand.type = Command::Type::sn76489_write;
            tempCommand.type = Command::Type::ignore;
            break;

            // length 2
        case T_writeYM2612port0:
        case T_writeYM2612port1:
            tempCommand.ymWrite.bank =
                (read == T_writeYM2612port1) ? true : false;
            if (!vgmBlocks.readByte(tempCommand.ymWrite.address)) {
                // need 2 bytes from next block
                tempCommand.type = Command::Type::ignore;
            } else if (!vgmBlocks.readByte(tempCommand.ymWrite.data)) {
                // need 1 byte from next block
                tempCommand.type = Command::Type::ignore;
            } else {
                tempCommand.type = Command::Type::ym2612_write;
            }
            break;
        case T_waitN:
            tempCommand.type = Command::Type::wait;
            if (vgmBlocks.readByte(tempUINT8)) {
                tempCommand.wait += tempUINT8;
            } else { break;} // need 2 bytes from next block
            if (vgmBlocks.readByte(tempUINT8)) {
                tempCommand.wait += tempUINT8 << 8U;
            } else { break; } // need 1 bytes from next block
            break;

        case T_offsetPCM:
            // First four uint8_ts define offset from PCM block (same as DAC
            // Data??) Each subsequent 0x8n plays a sample and steps ahead n
            // samples
            // if (fileP.read32(tempUINT32)) { tempCommand.pcmOffset =
            // tempUINT32; }
            tempCommand.type = Command::Type::ignore;
        


        // defined (or reserved) commands to be ignored
        // length 4
        case 0xE1:
        case 0xE2 ... 0xFF: vgmBlocks.skipBytesRead(1);
        // length 3
        case 0xC0 ... 0xDF: vgmBlocks.skipBytesRead(1);
        // length 2
        case 0x40 ... 0x4E: // potential issue with < v1.60
        case T_writeYM2413:
        case T_writeYM2151 ... T_writeYMF262port1:
        case 0xA0:
        case 0xA1 ... 0xAF:
        case 0xB0 ... 0xBF: vgmBlocks.skipBytesRead(1);
        // length 1
        case 0x30 ... 0x3F:
            vgmBlocks.skipBytesRead(1);
            tempCommand.type = Command::Type::ignore;
            break;

        // variable length
        case T_writePCMRAM: {
            if(vgmBlocks.isReadFinished(11)) { 
                break; 
            } else {
                vgmBlocks.skipBytesRead(1);
                uint8_t  chipType;
                vgmBlocks.readByte(chipType);
                uint32_t readOffset, writeOffset, dataSize; // technically these are all 24 bit
                vgmBlocks.read24(readOffset);
                vgmBlocks.read24(writeOffset);
                vgmBlocks.read24(dataSize);
                if(dataSize == 0U) {
                    dataSize = 0x01000000;
                }
            }
        }
            

        case T_writeDataBlock: {
            // No idea how to handle 4mB potential data.
            // Should probably break up the block so that progress can be
            // tested.
            if (!vgmBlocks.skipBytesRead(1)) { break; }
            uint8_t  typeOfDataBlock;
            vgmBlocks.readByte(typeOfDataBlock);
            uint32_t blockLength;
            if (vgmBlocks.readByte(tempUINT8)) { blockLength += tempUINT8; }
            if (vgmBlocks.readByte(tempUINT8)) {
                blockLength += (uint32_t)tempUINT8 << 8U;
            }
            if (vgmBlocks.readByte(tempUINT8)) {
                blockLength += (uint32_t)tempUINT8 << 16U;
            }
            if (vgmBlocks.readByte(tempUINT8)) {
                blockLength += (uint32_t)tempUINT8 << 24U;
            }
            // if(file.pointer8 + blockLength >=)

            if (typeOfDataBlock == 0x00U) {
                // PANIC!!!
                // fileP.pointer8 += blockLength; 
                // not somebody else's problem :(
            } else {
                // can't ignore it :(
                // fileP.pointer8 += blockLength;
            }
            break;
        }

        case 0x90 ... 0x95:

        // Undefined commands. Need to produce some kind of error
        default: break;
        }
    }

    return tempCommand;
}
 */
class Control {
    public:
      enum class Type : uint8_t {
          null = 0U,
          Settings,
          YM2612Write,
      };

    protected:
      // Are these future-appropriate?
      const uint8_t _minValue;
      const uint8_t _maxValue;
      const Type    _type;

      Control(const Type & type, const uint8_t & minValue, const uint8_t & maxValue)
      : _type(type), _minValue(minValue), _maxValue(maxValue) {};

      union {
          struct {
              const YM2612::RegisterAddress ym2612Address;
              const YM2612::RegisterType    ym2612Type;
              const uint8_t                 minValue = 0U;
              const uint8_t                 maxValue;
              const uint16_t                data;
          };
    };
    
};

class ControlYM2612 : public Control {
    const YM2612::RegisterAddress _rAddress;
    const YM2612::RegisterType    _rType;

    // is minValue always 0? Probably true binary-wise if not logic-wise
    // That reminds me, some kind of number remap is probably needed. 
    // Maybe on encoder though?
    ControlYM2612(const YM2612::RegisterAddress & rAddress,
                  const YM2612::RegisterType &    rType)
        : _rAddress(rAddress), _rType(rType), Control(Type::YM2612Write, 0U, (2 << (rType & B00001111))) {};
};

class Encoder {
    uint8_t         _deltaValue;
    uint8_t         _value;
    bool            isControlAttached = false;
    Control *       _currentControl = nullptr;

  public:
    Encoder(const uint8_t a, const uint8_t b, Control & control,
            const uint8_t initialValue = 0)
        : pinA(a), pinB(b), _deltaValue(), _value(initialValue),
          _currentControl(&control) {}

    const uint8_t pinA;
    const uint8_t pinB;

    Encoder & operator++() {
        _deltaValue++;
        _value++;
        return *this;
    }
    Encoder operator++(int) {
        Encoder temp = *this;
        ++*this;
        return temp;
    }

    Encoder & operator--() {
        _deltaValue--;
        _value--;
        return *this;
    }
    Encoder operator--(int) {
        Encoder temp = *this;
        --*this;
        return temp;
    }

    const uint8_t changeSinceLastUpdate() {
        auto temp   = _deltaValue;
        _deltaValue = 0U;
        return temp;
    }
    const uint8_t & value() { return _value; }

    void attachControlPointer(Control & control) {
        _currentControl = &control;
        isControlAttached = true;
    }
    void detachControlPointer() {
        _currentControl = nullptr;
        isControlAttached = false;
    }
    const Control * getControlPointer() {
        return _currentControl;
    }
};

// I think the read versions are unnecessary??
// Keep for reading from files
// TODO: Figure out if these are backwards?? Endianness etc.
word readTimerA(uint8_t a, uint8_t b) {
    return ((word)a << 2) | (b & B00000011);
}
word writeTimerA(word a) {
    uint8_t b = (uint8_t)a & B00000011;
    return ((a >> 2) << 8 | b);
}
word readFNum(uint8_t a, uint8_t b) { return a | ((b & B00000111) << 8); }
word writeFNum(word a) {
    word b = (a >> 8) & B00000111;
    return (a << 8 | b);
}

static uint16_t       overflowCount             = 0;
static const uint16_t clockCyclesPerSample      = 363U; // Closer to 362.8
static const uint8_t  samplesPerClockOverflow   = 180U; // Closer to 180.5
static const uint8_t  clockCyclesPerYM2612Cycle = 2;    // assumes 8MHz clock

// TODO: See about refactoring into namespace or enum or whatever.
volatile static bool isWaitCompleteVGMSamples   = false;
volatile static bool isWaitCompleteYM2612Cycles = false;
volatile static bool isControlChangeOverdue     = false;
static bool          controlChange              = false;
static bool          readyToWrite               = false;
static bool          readyToLoad                = false;
static bool          readyToSwap                = false;
volatile static bool isControlChangeDetected    = false;

struct Controls {

} currentControls;

static Encoder * encoder1;

static BlockHandler vgmbuffer;

const uint8_t loadBlock() {
    // This will break if blocksize exceeds uint8_t
    uint8_t count = 0U;
    while (Serial.available() && vgmbuffer.canWrite()) {
        vgmbuffer.writeByte(Serial.read());
        count++;
    }

    if (vgmbuffer.isWriteFinished()) {
        vgmbuffer.lockWrite();
        readyToLoad = false;
    }

    return count;
}

bool readInstructionFromBlock(VGM::Instruction & instruction) {
    if (vgmbuffer.canRead()) {
        if (vgmbuffer.readyForNextLoad() && vgmbuffer.canWrite()) { readyToLoad = true; }
        return VGM::readVGMInstruction(instruction, vgmbuffer);        
    } else if (vgmbuffer.readyForNextSwap()) {
        vgmbuffer.swapWriteToRead();
        return false;
    } else {
        readyToLoad = true;
        return false;
    }
}

// Timer/Interrupt Functions

// https://www.gammon.com.au/timers
void waitNSamples(uint16_t nSamples) {
    isWaitCompleteVGMSamples = false;

    // Reset timer settings
    TCCR1A = 0;
    TIMSK1 = 0;

    OCR1A = clockCyclesPerSample;
    if (nSamples > 0x100U) { // not worth factoring in unleap samples for less
        // -1 unleap cycle per five samples
        // -1 unleap sample per 1815 samples
        if (nSamples > clockCyclesPerSample * 5) {
            nSamples -= (nSamples / (clockCyclesPerSample * 5));
        }
        OCR1A -= (nSamples / 5);
        if (OCR1A > 50U) { OCR1A = 50U; } // completely arbitrary, figure it out
    }

    overflowCount = nSamples;

    if (overflowCount > samplesPerClockOverflow) {
        TIMSK1 = bit(OCIE1C);
    } else if (overflowCount > 0x10U) {
        TIMSK1 = bit(OCIE1B);
    } else {
        TIMSK1 = bit(OCIE1A);
    }

    TCCR1A = bit(WGM11); // CTC Mode
    TCNT1  = 0;          // Reset counter, not sure if redundant?
    TCCR1B = bit(CS10);  // turn on counter, no prescale
};

// sets Timer3 to trigger an interrupt after n YM2612 cycles
// I imagine behaviour is wonky for low cycle counts
void waitNCycles(const YM2612::WaitCycleType nCycles) {
    YM2612::currentState.isReadyToAccess = false;

    // Reset timer settings
    TCCR3A = 0;
    TIMSK3 = 0;

    OCR3A = (uint8_t)nCycles * clockCyclesPerYM2612Cycle;

    // Wait length cannot currently overflow

    TIMSK3 = bit(OCIE1A);

    TCCR3A = bit(WGM11); // CTC Mode
    TCNT3  = 0;          // Reset counter, not sure if redundant?
    TCCR3B = bit(CS10);  // turn on counter, no prescale
};

ISR(TIMER1_COMPA_vect) {
    overflowCount--;
    if (!overflowCount) {
        TIMSK1              = 0;
        isWaitCompleteVGMSamples = true;
    }
}

ISR(TIMER1_COMPB_vect) {
    overflowCount -= 0x10U;
    if (overflowCount) {
        if (overflowCount < 0x10U) {
            // go to compA
            TIMSK1 = bit(OCIE1A);
        }
    } else {
        isWaitCompleteVGMSamples = true;
    }
}

ISR(TIMER1_COMPC_vect) {
    overflowCount -= samplesPerClockOverflow;
    // there has to be a better way to represent this.
    if (overflowCount) {
        if (overflowCount < samplesPerClockOverflow) {
            if (overflowCount < 0x10U) {
                // go to compA
                TIMSK1 = bit(OCIE1A);
            } else {
                // go to compB
                TIMSK1 = bit(OCIE1B);
            }
        }
    } else {
        isWaitCompleteVGMSamples = true;
    }
}

ISR(TIMER3_COMPA_vect) {
    TIMSK3             = 0;
    isWaitCompleteYM2612Cycles = true;
}

// Physical controls such as encoders and buttons using PORTK pins
// While this should ideally be genericised, right now that just adds unneccessary abstraction.
ISR(PCINT2_vect) {
    static byte current;
    static byte last;
    static byte changed;

    current = PORTK;
    changed = current ^ last;

    // For encoders, only A should trigger the interrupt
    // https://youtu.be/v4BbSzJ-hz4?t=145
    if(bitRead(changed, ENC_OFF_1A)) {
        if (bitRead(current, ENC_OFF_1A) == bitRead(current, ENC_OFF_1B)) {
            encoder1--;
        } else {
            encoder1++;
        }
        isControlChangeDetected = true;
    }

    last = current;
    bitSet(PCIFR, PCIF2); // clear interrupt; not sure if done automatically
}

// Setup Functions
void setupBlock() {
    uint8_t count;

    Serial.begin(9600);

    while (!Serial) {
        Serial.begin(9600);
        delay(1000);
    }

    if(Serial.availableForWrite()) {
        Serial.write("Connected :)");
    }

    readyToLoad = true;
    count       = loadBlock();

    while (readyToLoad) {
        count += loadBlock(); // count is currently redundant
        delay(1000);
    }

    vgmbuffer.swapWriteToRead();

    VGM::currentHeader = VGM::readVGMHeader(vgmbuffer);

    // A max-length Header can exhaust the first block
    if (vgmbuffer.readyForNextLoad()) {
        readyToLoad = true;
        count       = loadBlock();

        while (readyToLoad) {
            count += loadBlock(); // count is currently redundant
            delay(1000);
        }
    }
}

void setupControls() { 
    encoder1 = &Encoder(ENC_PIN_1A, ENC_PIN_1B); 
}

void setupClocks() {
    // OCR1A = clockCyclesPerSample; Defined per-wait due to unleap cycles
    OCR1B = clockCyclesPerSample * 0x10U;                   // 5808
    OCR1C = clockCyclesPerSample * samplesPerClockOverflow; // 65340
}

void setupMIDI() { MIDI_CREATE_DEFAULT_INSTANCE(); }

void setupPins() {
    pinMode(YM_PIN_IC, OUTPUT);
    pinMode(YM_PIN_IRQ, OUTPUT);
    pinMode(YM_PIN_CS, OUTPUT);
    pinMode(YM_PIN_RD, OUTPUT);
    pinMode(YM_PIN_WR, OUTPUT);
    pinMode(YM_PIN_A0, OUTPUT);
    pinMode(YM_PIN_A1, OUTPUT);

    for (int i = 0; i < 8; i++) {
        pinMode(YM_PIN_D0 + i, INPUT);
    }

    pinMode(SN_PIN_WE, OUTPUT);
    pinMode(SN_PIN_CE, OUTPUT);
    pinMode(SN_PIN_READY, INPUT);

    for (int i = 0; i < 8; i++) {
        pinMode(SN_PIN_D0 + i, OUTPUT);
    }

    // attachInterrupt(digitalPinToInterrupt(ENC_PIN_1A), interruptEncoder1,
    // CHANGE);
    bitSet(PCICR, PCIE2);
    bitSet(PCMSK2, PCINT16);
    pinMode(ENC_PIN_1A, INPUT_PULLUP);
    pinMode(ENC_PIN_1B, INPUT_PULLUP);

    digitalWrite(YM_PIN_IC, HIGH);
    digitalWrite(YM_PIN_IRQ, HIGH);
    digitalWrite(YM_PIN_CS, HIGH);
    digitalWrite(YM_PIN_RD, HIGH);
    digitalWrite(YM_PIN_WR, HIGH);
    digitalWrite(YM_PIN_A0, HIGH);
    digitalWrite(YM_PIN_A1, HIGH);

    digitalWrite(SN_PIN_WE, HIGH);
    digitalWrite(SN_PIN_CE, HIGH);
    for (int i = 0; i < 8; i++) {
        digitalWrite(SN_PIN_D0 + i, LOW);
    }
}

void setup() {
    setupBlock();
    setupControls();
    setupClocks();
    // setupMIDI();
    setupPins();
    isControlChangeOverdue = false;
    readyToWrite           = true;
}

void updateYM2612(const Control & control) {
    YM2612::WaitCycleType cycles = YM2612::currentState.updateRegister(
        control.ym2612Address, control.ym2612Type, control.data);
    if (cycles != YM2612::WaitCycleType::null) {
        waitNCycles(cycles);
    } else {
    } // write failed, for some reason (probably the address)
}

// Returns true on successful write
const bool updateYM2612(const YM2612::Write & write) {
    YM2612::WaitCycleType cycles = YM2612::currentState.updateRegister(write);
    switch (cycles) {
    case YM2612::WaitCycleType::null:
        // case YM2612::WaitCycleType::error:
        return false;
    default: waitNCycles(cycles); return true;
    }
    if (cycles != YM2612::WaitCycleType::null) {
        waitNCycles(cycles);
        return true;
    } else {
        return false;
    }
}

void loop() {
    static bool    isVGMPileReady  = false;
    static bool    isVGMTempFull = false;
    static Command vgmTemp;

    static bool          isYM2612TempFull = false;
    static YM2612::Write ym2612Temp;
    static bool &        isYM2612Ready = YM2612::currentState.isReadyToAccess;
    static YM2612::WaitCycleType ym2612WaitReason = YM2612::WaitCycleType::null;

    static bool           isSN76489TempFull = false;
    static SN76489::Write sn76489Temp;

    if (isWaitCompleteYM2612Cycles) {
        using namespace YM2612;
        switch (ym2612WaitReason) {
        case WaitCycleType::RESETPULSEWIDTH: currentState.finishReset(); break;
        case WaitCycleType::WRITEWAITWIDTH1: // continue write
            currentState.finishUpdate(WaitCycleType::WRITEWAITWIDTH1);
            break;
        case WaitCycleType::WRITEWAITWIDTH2:
        case WaitCycleType::WRITEWAITWIDTH3: // clear for next write
            isYM2612Ready = true;
            break;
        default: break;
        }
        ym2612WaitReason           = WaitCycleType::null;
        isWaitCompleteYM2612Cycles = false;
    }
    
    // before control changes, and before taking from the pile, process any
    // command not yet dealt with
    if (isSN76489TempFull) {
        isSN76489TempFull =
            !SN76489::currentState.updateRegister(sn76489Temp);
    }
    if (isYM2612TempFull) {
        isYM2612TempFull = !updateYM2612(ym2612Temp);
    }
    if (isVGMTempFull) {
        switch (vgmTemp.type) {
        case Command::Type::sn76489_write:
            if (SN76489::currentState.isReadyToAccess &&
                SN76489::currentState.updateRegister(vgmTemp.snWrite)) {
                isSN76489TempFull = false;
                isVGMTempFull     = false;
            } else if (!isSN76489TempFull) {
                isSN76489TempFull = true;
                sn76489Temp       = vgmTemp.snWrite;
                isVGMTempFull     = false;
            } // else vgmBuffer is still full
            break;
        case Command::Type::ym2612_write:
            if (isYM2612Ready && updateYM2612(vgmTemp.ymWrite)) {
                isYM2612TempFull = false;
                isVGMTempFull    = false;
            } else if (!isYM2612TempFull) {
                isYM2612TempFull = true;
                ym2612Temp       = vgmTemp.ymWrite;
                isVGMTempFull    = false;
            } // else vgmBuffer is still full
            break;
        default: isVGMTempFull = false; break;
        }
    }

    if (isControlChangeOverdue) {}

    // if a control change is detected (should this be before/after VGM
    // command?)
    if (isControlChangeDetected) {
        // this should be genericised for each encoder
        if (encoder1->changeSinceLastUpdate() != 0) {
            const Control * currentControl = encoder1->getControlPointer();
            switch (currentControl->type) {
            case Control::Type::null:
            default: break;
            case Control::Type::Settings: break;
            case Control::Type::YM2612Write:
                auto butts = YM2612::currentState.getRegister(
                    currentControl->ym2612Address, currentControl->ym2612Type);

                butts += encoder1->changeSinceLastUpdate();

                // this still seems rough. Need a proper clamp
                // also, some controls could cycle
                if (butts > 0xFFF) {
                    butts = currentControl->minValue; // is this always 0U?
                } else if (butts > currentControl->maxValue) {
                    butts = currentControl->maxValue;
                }

                YM2612::currentState.updateRegister(
                    currentControl->ym2612Address, currentControl->ym2612Type,
                    butts);
                readyToWrite = false;
                break;
            }
        }

        isControlChangeDetected = false;
    }

    if (isWaitCompleteVGMSamples && !isVGMTempFull && isVGMPileReady) {
        if (currentCommandPile.takeFromPile(vgmTemp)) {
            switch (vgmTemp.type) {
            case Command::Type::wait: waitNSamples(vgmTemp.wait); break;
            case Command::Type::ym2612_inactive: // what's going on here?
                YM2612::currentState.startReset();
                waitNCycles(YM2612::WaitCycleType::RESETPULSEWIDTH);
                ym2612WaitReason = YM2612::WaitCycleType::RESETPULSEWIDTH;
                break;
            case Command::Type::ym2612_read: break;

            // null should never happen, but isn't necessarily 'wrong'
            case Command::Type::null:
            default: break;

            // handled next 'turn'
            case Command::Type::sn76489_write:
            case Command::Type::ym2612_write: isVGMTempFull = true; break;
            }
        } else {
            isVGMPileReady = false;
        }
    }

    static bool             isInstructionIncomplete = false;
    static VGM::Instruction incomplete;
    uint8_t readAttempts = 0U;
    while (currentCommandPile.getCount() < COMMANDPILESIZE && readAttempts < READATTEMPTS) {
        VGM::Instruction current;
        if (isInstructionIncomplete) { current = incomplete; }
        if (readInstructionFromBlock(current)) {
            // complete instruction; convert to command
            Command tempCommand = convertVGMInstructionToCommand(current);
            switch (tempCommand.type) {
            case Command::Type::null:
            case Command::Type::ignore: break;
            default:
                if (currentCommandPile.addToPile(tempCommand)) {
                    isVGMPileReady = true;
                }
                break;
            }
            isInstructionIncomplete = false;
            readAttempts            = 0U;
        } else if (current.bytesNeeded != 0) {
            // incomplete instruction
            incomplete = current;
            if (isInstructionIncomplete) {
                // incomplete 2nd+ time too, probably a data block/similar
            } else {
                isInstructionIncomplete = true;
            }
        } else {
            // unsuccessful read
            readAttempts++;
        }
    }

    if (Serial && readyToLoad) { loadBlock(); }
}