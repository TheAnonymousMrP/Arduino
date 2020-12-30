#include <SD.h>
#include <MIDI.h>

#include "Test/testdata.h"

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

#define pIC     PORTA + PA5 // 73
#define pIRQ    PORTA + PA6 // 72
#define pCS     PORTA + PA0 // 78
#define pRD     PORTA + PA1 // 77
#define pWR     PORTA + PA2 // 76
#define pA1     PORTA + PA3 // 75
#define pA0     PORTA + PA4 // 74
#define pD0     PORTC + PC0 // 53
#define DataBus PORTC       // 53-60

#define ENCODERPORT PORTK
#define ENCODER1A   PORTK + PCINT16 // 89
#define ENCODER1B   PORTK + PCINT17 // 88

#define BLOCKSIZE 0x100U
#define COMMANDPILESIZE 0x80U

class BlockWriter {
    uint8_t         dataBlock[BLOCKSIZE];
    uint8_t *       _pointer8 = &dataBlock[0];
    uint8_t * const _SOF      = _pointer8;
    uint8_t *       _EOF      = _SOF + BLOCKSIZE;
    bool            _locked   = false;

  public:
    // BlockWriter();
    operator bool() { return !_locked && getLength() > 0 && (_pointer8 < _EOF); }

    const uint8_t * const getSOF() { return _SOF; }
    const uint8_t * const getEOF() { return _EOF; }
    const uint16_t        getLength() { return _EOF - _SOF; }
    const uint8_t * const getPosition() { return _pointer8; }
    const uint8_t * const getPositionMid() { return _SOF + (getLength() / 2); }

    void lock() {
        if (_pointer8 < _EOF) { _EOF = _pointer8; }
        _locked = true;
    }

    bool isLocked() { return _locked; }

    bool isFull() { return _pointer8 >= _EOF; }

    bool skipBytes(uint16_t nBytes) {
        if (this && _pointer8 + nBytes < _EOF) {
            for (int i = 0; i < nBytes; i++) {
                *_pointer8++ = 0x00U;
            }
            return true;
        } else {
            return false;
        }
    }

    bool writeByte(const uint8_t byte) {
        if (this) { *_pointer8++ = byte; }
    }

    uint8_t writeBytes(const uint8_t * buffer, uint8_t nBytes) {
        if (this) {
            uint8_t written = 0U;
            for (written; written < nBytes; written++) {
                if (!writeByte(*buffer++)) { break; };
            }
            return written;
        } else {
            return 0U;
        }
    }
};

class BlockReader {
    const uint8_t * const _SOF;
    const uint8_t * const _EOF;
    const uint8_t *       _pointer8;

  public:
    BlockReader(const uint8_t start, const uint8_t end)
        : _SOF(&start), _pointer8(_SOF), _EOF(&end){};
    BlockReader(const uint8_t * const start, const uint8_t * const end)
        : _SOF(start), _pointer8(_SOF), _EOF(end){}; 
    BlockReader(BlockWriter & data)
        : _SOF(data.getSOF()), _pointer8(_SOF), _EOF(data.getEOF()){};

    operator bool() const { return getLength() > 0 && (_pointer8 < _EOF); }

    const uint16_t getLength() const { return _EOF - _SOF; }

    const uint8_t * const getPosition() const { return _pointer8; }
    const uint8_t * const getPositionMid() const { return _SOF + getLength() / 2; }

    bool read32(uint32_t & buffer) {
        if (getLength() && _pointer8 + 4 < _EOF) {
            buffer = *(uint32_t *)_pointer8;
            return true;
        } else {
            return false;
        }
    }

    bool read16(uint16_t & buffer) {
        if (getLength() && _pointer8 + 2 < _EOF) {
            buffer = *(uint16_t *)_pointer8;
            return true;
        } else {
            return false;
        }
    }

    bool readByte(uint8_t & buffer) {
        if (this) {
            buffer = *_pointer8++;
            return true;
        } else {
            return false;
        }
    }
    // this probably doesn't work, don't use it
    uint8_t readBytes(uint8_t * buffer, uint16_t nBytes) {
        uint8_t returned = 0U;
        for (returned; returned < nBytes; returned++) {
            if (!readByte(*buffer++)) { break; }
        }
        return returned;
    }

    bool skipBytes(uint16_t nBytes) {
        if (_pointer8 + nBytes < _EOF) {
            _pointer8 += nBytes;
            return true;
        } else {
            // consider setting _pointer8 to _EOF
            return false;
        }
    }

    // To avoid writing peek methods, rollback one byte
    bool rollback() {
        if(_pointer8 > _SOF) {
            _pointer8--;
            return true;
        } else {
            return false;
        }
    }
};

class Encoder {
    uint8_t        _deltaValue;
    uint8_t        _value;

  public:  
    Encoder(const uint8_t & a, const uint8_t & b, const Control & control,
            const uint8_t & initialValue = 0)
        : portA(a), portB(b), control(control), _deltaValue(),
          _value(initialValue) {}

    const uint8_t portA;
    const uint8_t portB;
    const Control control;

    Encoder & operator++() {
        _deltaValue++;
        _value++;
        return *this;
        }
        Encoder & operator++(int) {
            Encoder temp = *this;
            ++*this;
            return temp;
        }

        Encoder & operator--() {
            _deltaValue--;
            _value--;
            return *this;
        }
        Encoder & operator--(int) {
            Encoder temp = *this;
            --*this;
            return temp;
        }

        const uint8_t & changeSinceLastUpdate() { 
            auto temp = _deltaValue;
            _deltaValue = 0U;
            return temp; 
        }
        const uint8_t & value() { return _value; }
};

class Control {
  public:
    enum class Type : uint8_t {
        null = 0U,
        Settings,
        YM2612Write,
    };

    Control(const YM2612::RegisterAddress & rAddress,
            const YM2612::RegisterType &    rType)
        : ym2612Address(rAddress), ym2612Type(rType), type(Type::YM2612Write){};

    union {
        struct {
            const YM2612::RegisterAddress ym2612Address;
            const YM2612::RegisterType    ym2612Type;
        };
    };
    const Type type;
};

class ControlChange : public Control {
  public:
    ControlChange(const Control & control, const uint16_t & data = 0U)
        : Control(control), ym2612Data(data),
          maxValue(2 << (control.ym2612Type & B00001111)){};

    ControlChange(const YM2612::RegisterAddress & rAddress,
                  const YM2612::RegisterType &    rType,
                  const uint16_t &                data = 0U)
        : Control(rAddress, rType), ym2612Data(data),
          maxValue(2 << (rType & B00001111)){};

    union {
        uint16_t ym2612Data;
    };

    const uint8_t minValue = 0U;
    const uint8_t maxValue;
};

namespace VGM {
    const uint32_t OFFSET_DEFAULT = 0x0C;
    const uint16_t SIXTY_SAMPLES  = 735;
    const uint16_t FIFTY_SAMPLES  = 882;
    enum class Type : uint8_t {
        null,
        dataBlock, // is this not just a long sequence of Instructions??
        header,
        instruction,
        wait
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

    const VGM::Header readVGMHeader(BlockReader & fileP) {
        VGM::Header tHead; // .isValid is implicitly false;
        uint32_t    SN76478temp;

        if (!fileP.read32(tHead.fileID)) { return tHead; }          // 0x00++
        if (!fileP.read32(tHead.offsetEOF)) { return tHead; }       // 0x04++
        if (!fileP.read32(tHead.fileVersion)) { return tHead; }     // 0x08++
        if (!fileP.read32(tHead.SN76489clock)) { return tHead; }    // 0x0C++
        if (!fileP.skipBytes(8)) { return tHead; }                  // 0x10+8
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
        if (!fileP.skipBytes(4)) { return tHead; }                 // 0x30+4

        if (tHead.fileVersion < 0x00000150) {
            tHead.offsetStart = VGM::OFFSET_DEFAULT;
        } else {
            if (!fileP.read32(tHead.offsetStart)) { return tHead; } // 0x30+4
        }

        if (!fileP.skipBytes(tHead.offsetStart)) {
            return tHead;
        } // end of header

        tHead.isValid = true;

        return tHead;
    }

    void testSize() { auto sHeader = sizeof(currentHeader); }
} // namespace VGM

namespace YM2612 {
    const uint32_t clockSpeed = 7670453U;

    // TODO: Pick one and stick with it.
    // Pros: Easier to abuse compares for ifs and switches
    enum RegisterAddress : uint8_t { // Address
        Test1         = 0x21,
        LFO           = 0x22,
        TimerA        = 0x24,
        TimerB        = 0x26,
        TimerLoad     = 0x27,
        TimerEnable   = 0x27,
        TimerReset    = 0x27,
        TimerMode     = 0x27,
        Channel       = 0x28,
        Slot          = 0x28,
        DACData       = 0x2A,
        DACSelect     = 0x2B,
        Test2         = 0x2C,
        // These have a byte for each operator for each channel (see Page 16).
        // TODO: Figure out how to factor in an offset
        DT            = 0x30,
        Multi         = 0x30,
        TL            = 0x40,
        KS            = 0x50,
        AR            = 0x50,
        AM            = 0x60,
        DR            = 0x60,
        SR            = 0x70,
        SL            = 0x80,
        RR            = 0x80,
        SSGEG         = 0x90,
        // These have a byte for each channel.
        FNum1         = 0xA0,
        FNum2         = 0xA4,
        Block         = 0xA4,
        // These have a byte for each operator (channel 3).
        // TODO: Figure out how to factor in an offset.
        Channel3FNum1 = 0xA8,
        Channel3FNum2 = 0xAC,
        Channel3Block = 0xAC,
        // These have a byte for each channel.
        FB            = 0xB0,
        Algo          = 0xB0,
        LR            = 0xB4,
        AMS           = 0xB4,
        PMS           = 0xB4
    };

    enum RegisterType : uint8_t { // Size | Offset << 4 | Bank << 7
        Test1       = 8 | (0 << 4),
        LFO         = 4 | (0 << 4),
        TimerA      = 10 | (0 << 4),
        TimerB      = 8 | (0 << 4),
        TimerLoad   = 2 | (0 << 4),
        TimerEnable = 2 | (2 << 4),
        TimerReset  = 2 | (4 << 4),
        TimerMode   = 2 | (6 << 4),
        Channel     = 3 | (0 << 4),
        Slot        = 4 | (4 << 4),
        DACData     = 8 | (0 << 4),
        DACSelect   = 1 | (7 << 4),
        Test2       = 8 | (0 << 4),
        // These have a byte for each operator for each channel (see Page 16).
        // TODO: Figure out how to factor in an offset
        DT    = 3 | (4 << 4),
        Multi = 4 | (0 << 4),
        TL    = 7 | (0 << 4),
        KS    = 2 | (6 << 4),
        AR    = 5 | (0 << 4),
        AM    = 1 | (7 << 4),
        DR    = 5 | (0 << 4),
        SR    = 5 | (0 << 4),
        SL    = 4 | (4 << 4),
        RR    = 4 | (0 << 4),
        SSGEG = 4 | (0 << 4),
        // These have a byte for each channel.
        FNum1 = 8 | (0 << 4),
        FNum2 = 3 | (0 << 4),
        Block = 3 | (3 << 4),
        // These have a byte for each operator (channel 3).
        // TODO: Figure out how to factor in an offset.
        Channel3FNum1 = 8 | (0 << 4),
        Channel3FNum2 = 3 | (0 << 4),
        Channel3Block = 3 | (3 << 4),
        // These have a byte for each channel.
        FB   = 3 | (3 << 4),
        Algo = 3 | (0 << 4),
        LR   = 2 | (6 << 4),
        AMS  = 3 | (4 << 4),
        PMS  = 3 | (0 << 4)
    };

    struct Command {
        enum class Type : uint8_t { null, ignore, inactive, read, wait, write, writeDAC };
        union {
            // uint32_t pcmOffset;
            uint16_t wait;
            struct {
                uint8_t address;
                uint8_t data;
            };
        };
        Type type = Type::null;
        bool bank = false;
    };

    class CommandPile {
        // const commands?
        Command         pile[COMMANDPILESIZE];
        const Command * currentCommandToRead;
        Command *       currentCommandToWrite;
        static uint8_t  commandCount;
        // TODO: This will break if COMMANDPILESIZE exceeds uint8_t

      public:
        CommandPile()
            : pile(), currentCommandToRead(pile), currentCommandToWrite(pile) {
            commandCount = 0;
        };

        bool addToPile(const Command & newCommand) {
            // can this overflow unintentionally?
            if (commandCount < COMMANDPILESIZE &&
                newCommand.type != Command::Type::null) {
                // is this right??
                if (currentCommandToWrite >= pile + sizeof(pile)) {
                    currentCommandToWrite = pile;
                }
                commandCount++;
                *currentCommandToWrite++ = newCommand;
            } else {
                return false;
            }
        }

        const uint8_t getCount() { return commandCount; }

        // retrieves next Command, 'removing' it from the pile.
        const Command takeFromPile() {
            if (commandCount) {
                if (currentCommandToRead >= pile + sizeof(pile)) {
                    currentCommandToRead = pile;
                }
                commandCount--;
                return *currentCommandToRead++;
            } else {
                return Command();
            }
        }
    } currentCommandPile;

    // Maintains a copy of the YM2612's memory and handles data transfer
    class State {
        static const uint8_t sharedStart  = 0x21U; // Test1
        static const uint8_t channelStart = 0x30U; // DT
        static const uint8_t channelEnd   = 0xB6U; // LR-AMS-PMS
        uint8_t              sharedRegister[channelStart - sharedStart];
        uint8_t              channelRegister0[channelEnd - channelStart];
        uint8_t              channelRegister1[channelEnd - channelStart];
        Command              lastCommand;
        // TODO: Remove unused addresses from internal state

      public:
        State() : sharedRegister(), channelRegister0(), channelRegister1() {}; // prep some default state

        // evaluates if this State is initialised and active
        // operator bool() const {}

        // Returns 0/false if invalid; this seems wrong
        const uint16_t & getRegister(const RegisterAddress & address,
                                     const RegisterType &    type) {
            const uint8_t size     = type & B00001111;
            const uint8_t offset   = (type >> 4)  & B00000111;
            const bool    bank0Or1 = (type >> 15) ? true : false;

            uint8_t * temp;

            switch (address) {
            case sharedStart ... 0x2F:
                temp = &sharedRegister[address - sharedStart];
                if (address == 0x23) {
                    return false; // invalid address
                } else if (address == 0x24) {
                    // TimerA
                }
                break;
            case channelStart ... channelEnd:
                temp = (bank0Or1) ? &channelRegister1[address - channelStart]
                                  : &channelRegister0[address - channelStart];
                if (address % 4U == 3U) {
                    return false; // invalid address
                }
                break;
            default: return false; break;
            }

            if (size == 8U) {
                return *temp;
            } else if (size > 8U) {
                // need something for TimerA and FNum(?)
            } else {
                return (*temp & getMask(type)) >> offset;
            }
        }

        void finishUpdate(const WaitCycleType & reason) {
            writeDataToBus(lastCommand.data, lastCommand.bank);
            writeWaitWidth(true, lastCommand.address);
        }

        // method to write VGM file data to YM2612
        bool updateRegister(const Command & command) {
            if (command.type != Command::Type::write) { return false; }
            const uint8_t addr = command.address;

            uint8_t * temp;
            switch (addr) {
            case sharedStart ... 0x2F:
                temp = &sharedRegister[addr - sharedStart];
                if (addr == 0x23) {
                    return false; // invalid address
                }
                break;
            case channelStart ... channelEnd:
                temp = (command.bank) ? &channelRegister1[addr - channelStart]
                                      : &channelRegister0[addr - channelStart];
                if (addr % 4U == 3U) {
                    return false; // invalid address
                }
                break;
            default: return false; break;
            }

            *temp = command.data;
            startUpdate(command, lastCommand);
            lastCommand = command;
            return true;
        }

        // method to write specific data to YM2612
        bool updateRegister(const RegisterAddress & address, const RegisterType & type,
                            const uint16_t &    data = 0U) {
            const uint8_t size     = type & B00001111;
            const uint8_t offset   = (type >> 4)  & B00000111;
            const bool    bank0Or1 = (type >> 15) ? true : false;

            uint8_t * temp;
            switch (address) {
            case sharedStart ... 0x2F:
                temp = &sharedRegister[address - sharedStart];
                if (address == 0x23) {
                    return false; // invalid address
                } else if (address == 0x24) {
                    // TimerA
                }
                break;
            case channelStart ... channelEnd:
                temp = (bank0Or1) ? &channelRegister1[address - channelStart]
                                  : &channelRegister0[address - channelStart];
                if (address % 4U == 3U) {
                    return false; // invalid address
                }
                break;
            default: return false; break;
            }
            
            if (size == 8U) {
                *temp = (uint8_t)data;
            } else if (size > 8U) {
                // need something for TimerA and FNum(?)
            } else {
                *temp &= ~getMask(type);
                *temp |= (uint8_t)data << offset;
            }

            Command command;
            command.type          = YM2612::Command::Type::write;
            command.address = address;
            command.data    = *temp;
            command.bank          = bank0Or1;
            startUpdate(command, lastCommand);
            lastCommand = command;
        }

        bool updateRegister(const ControlChange & control) {
            if (control.type == Control::Type::YM2612Write) {
                return updateRegister(control.ym2612Address,
                                      control.ym2612Type,
                                      control.ym2612Data);
            } else {
                return false;
            }
        }

        void startReset() {
            YM2612::startReset();
        }

        // empty internal memory to match YM2612
        // may need some initial state
        void finishReset() {
            YM2612::finishReset();
            memset(sharedRegister, 0U, sizeof(sharedRegister));
            memset(channelRegister0, 0U, sizeof(channelRegister0));
            memset(channelRegister1, 0U, sizeof(channelRegister1));
            lastCommand = Command();
        }
    } currentState;

    uint8_t getMask(const RegisterType & type) {
            const uint8_t size    = type & B00001111;
            const uint8_t offset  = (type >> 4U & B00000111);

            return (0xFF >> (8 - size)) << offset;
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

    uint8_t readDataBus() {
        digitalWriteFast(pA0, LOW);
        digitalWriteFast(pA1, LOW);

        __asm__ __volatile__("NOPX 1"); // tAS, min 10ns
        digitalWriteFast(pCS, LOW);
        digitalWriteFast(pRD, LOW);
        // no need to change mode for pins

        __asm__ __volatile__("NOPX 21"); // tACC, min 250ns (I think)

        uint8_t status = DataBus;

        __asm__ __volatile__("NOPX 9"); // tCSR & tRW, min 100ns (kind of)
        digitalWriteFast(pRD, HIGH);
        digitalWriteFast(pCS, HIGH);

        __asm__ __volatile__("NOPX 1"); // tAH & tDH, min 10ns
        // no need to change mode for pins

        return status;
    }

    void startReset() {
        digitalWriteFast(pIC, LOW);
        // Wait 192 cycles
        waitNCycles(WaitCycleType::RESETPULSEWIDTH);
    }

    void finishReset() {
        digitalWriteFast(pIC, HIGH);
        // setupPins();
    }

    void startUpdate(const Command & command, const Command & lastCommand) {
        bool       processAddress = true;
        const bool isCurBank1 =
            (lastCommand.type == Command::Type::write && lastCommand.bank)
                ? true
                : false;
        const bool isNewBank1 = command.bank;

        // check address against last instruction if appropriate
        if (lastCommand.type == Command::Type::write) {
            processAddress != isCurBank1 ==
                isNewBank1 && command.address ==
                lastCommand.address;
        }

        if (processAddress) {
            writeDataToBus(command.address, command.bank);
            writeWaitWidth(false, command.address);
        } else {
            writeDataToBus(command.data, command.bank);
            writeWaitWidth(true, command.address);
        }
    }

    void writeDataToBus(const uint8_t & data, const bool & bank0or1 = false) {
        digitalWriteFast(pA0, !bank0or1);
        digitalWriteFast(pA1, bank0or1);

        __asm__ __volatile__("NOPX 1"); // tAS, min 10ns
        digitalWriteFast(pCS, LOW);
        digitalWriteFast(pWR, LOW);
        for (int i = 0; i < 8; i++) {
            pinMode(pD0 + i, OUTPUT);
        }

        DataBus |= data;

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

    bool writeWaitWidth(bool fromAddressOrData, const uint8_t & address) {
        WaitCycleType temp = WaitCycleType::null;

        if (fromAddressOrData) {
            switch (address) {
            case 0x21 ... 0x9F: temp = WaitCycleType::WRITEWAITWIDTH2; break;
            case 0xA0 ... 0xB6: temp = WaitCycleType::WRITEWAITWIDTH3; break;

            default: return false;
            }
        } else
            temp = WaitCycleType::WRITEWAITWIDTH1;

        if (waitNCycles(temp)) return true;
        else
            return false;
    }

    namespace Test {
        void sizeTest() {
            // using namespace YM2612;
            sizeof(Command::Type);
            sizeof(Command::wait);
            sizeof(Command::address);
            sizeof(Command::data);
            sizeof(Command::bank); 
            // sizeof(Command::pcmOffset);
            sizeof(Command);
            sizeof(RegisterAddress);
            sizeof(RegisterType);
        }

        void maskTest() { using namespace YM2612; }

        void stateTest() { using namespace YM2612; }

        class StateTest : State {
          public:
            void testMasks() {}
        };
    } // namespace Test
}; // namespace YM2612

// TODO: Rework this to be agnostic.
const YM2612::Command readVGMCommand(BlockReader & fileP,
                                     VGM::Header header = VGM::currentHeader) {
    enum CommandType : uint8_t {
        writeYM26120    = 0x52,
        writeYM26121    = 0x53,
        waitN           = 0x61,
        wait60th        = 0x62,
        wait50th        = 0x63,
        endofSoundData  = 0x66,
        writeDataBlock  = 0x67,
        waitNplus1      = 0x70,
        writeDataYM2612 = 0x80,
        offsetPCM       = 0xE0
    };

    uint32_t        tempUINT32;
    uint8_t         tempUINT8;
    uint8_t         read;
    YM2612::Command tempCommand;

    if (fileP.readByte(read)) {
        // this all seems dodgy as hell. At least no data can be changed??
        switch (read) {
        case writeYM26120:
        case writeYM26121:
            if (fileP.readByte(tempCommand.address) &&
                fileP.readByte(tempCommand.data)) {
                tempCommand.bank = (read == writeYM26121) ? true : false;
                tempCommand.type = YM2612::Command::Type::write;
            }
            break;
        case waitN:
            // Not sure if necessary to 'initialise' the union.
            tempCommand.wait = 0U; 
            if (fileP.readByte(tempUINT8)) { tempCommand.wait += tempUINT8; }
            if (fileP.readByte(tempUINT8)) {
                tempCommand.wait += tempUINT8 << 8U;
            }
            if (tempCommand.wait > 0U) {
                tempCommand.type = YM2612::Command::Type::wait;
            }
            break;
        case wait60th:
            tempCommand.wait = VGM::SIXTY_SAMPLES;
            tempCommand.type = YM2612::Command::Type::wait;
            break;
        case wait50th:
            tempCommand.wait = VGM::FIFTY_SAMPLES;
            tempCommand.type = YM2612::Command::Type::wait;
            break;
        case waitNplus1 ... waitNplus1 + 0x0FU:
            tempCommand.wait = read - waitNplus1 + 1;
            tempCommand.type = YM2612::Command::Type::wait;
            break;
        case writeDataBlock: {
            // No idea how to handle 4mB potential data.
            // Should probably break up the block so that progress can be
            // tested.
            if (!fileP.skipBytes(1)) { break; }
            uint8_t  typeOfDataBlock = fileP.readByte(typeOfDataBlock);
            uint32_t blockLength;
            if (fileP.readByte(tempUINT8)) { blockLength += tempUINT8; }
            if (fileP.readByte(tempUINT8)) {
                blockLength += (uint32_t)tempUINT8 << 8U;
            }
            if (fileP.readByte(tempUINT8)) {
                blockLength += (uint32_t)tempUINT8 << 16U;
            }
            if (fileP.readByte(tempUINT8)) {
                blockLength += (uint32_t)tempUINT8 << 24U;
            }
            // if(file.pointer8 + blockLength >=)

            if (typeOfDataBlock == 0x00U) {
                // PANIC!!!
                // fileP.pointer8 += blockLength; // not somebody else's
                // problem :(
            } else {
                // can't ignore it :(
                // fileP.pointer8 += blockLength;
            }
            break;
        }
        case writeDataYM2612 ... writeDataYM2612 + 0x0FU: 
            // write one byte of PCM data to YM2612 DAC Data (0x2A)
            // And then wait n samples.

            // For now, just treat as a wait. To condense multiple waits,
            // read ahead until end of block or non-0xEn byte.
            tempCommand.wait = read & B00001111;
            bool isValid = true;
            while (fileP.readByte(tempUINT8) && isValid) {
                if (tempUINT8 & 0xF0 == writeDataYM2612) {
                    tempCommand.wait += (tempUINT8 & B00001111);
                } else {
                    isValid = false;
                    fileP.rollback();
                }
            }
            tempCommand.type = YM2612::Command::Type::wait;
        case offsetPCM:
            // First four uint8_ts define offset from PCM block (same as DAC
            // Data??) Each subsequent 0x8n plays a sample and steps ahead n
            // samples
            // if (fileP.read32(tempUINT32)) { tempCommand.pcmOffset = tempUINT32; }
            tempCommand.type = YM2612::Command::Type::ignore;
        case endofSoundData:
            // some kind of flag?
        default:
            // VGMType::null;
            break;
        }
    }

    return tempCommand;
}

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


static uint16_t       overflowCount             = 0;
static const uint16_t clockCyclesPerSample      = 363U; // Closer to 362.8
static const uint8_t  samplesPerClockOverflow   = 180U; // Closer to 180.5
static const uint8_t  clockCyclesPerYM2612Cycle = 2;    // assumes 8MHz clock

// TODO: See about refactoring into namespace or enum or whatever.
static bool waitCompleteSamples = false;
static bool waitCompleteCycles = false;
static bool controlChange = false;
static bool readyToWrite = false;
static bool readyToTake = false;
static bool readyToLoad = false;
static bool readyToControlChange = false;
enum struct WaitCycleType : uint8_t {
    null = 0U,
    RESETPULSEWIDTH = 192U,
    WRITEWAITWIDTH1 = 17U,
    WRITEWAITWIDTH2 = 83U,
    WRITEWAITWIDTH3 = 47U,
    ACCESSPROHIBITIONWIDTH = WRITEWAITWIDTH1
} waitReason;

static Encoder * encoder1;

static BlockWriter* tempBlock;
static BlockReader* activeBlock;
static BlockReader* inactiveBlock;

const uint8_t loadBlock() {
    // This will break if blocksize exceeds uint8_t
    uint8_t count = 0U;
    while (Serial.available() && *tempBlock) {
        tempBlock->writeByte(Serial.read());
        count++;
    }

    if (tempBlock->isFull()) {
        tempBlock->lock();
        inactiveBlock = &BlockReader(*tempBlock);
        tempBlock     = &BlockWriter();
        readyToLoad   = false;
    }

    return count;
};

bool readCommandFromBlock(YM2612::Command & command) {    
    if (*activeBlock) {
        if (activeBlock->getPosition() > activeBlock->getPositionMid()) {
            // load next block into inactiveBlock
            readyToLoad = true;
        }

        command = readVGMCommand(*activeBlock);
    } else if (*inactiveBlock) { // check opposite block for validity
        command = readVGMCommand(*inactiveBlock);
        activeBlock = inactiveBlock;
        readyToLoad = true;
    } else {
        // no active blocks
        // load next block into inactiveBlock
        readyToLoad = true;
    }

    return (command.type != YM2612::Command::Type::null);
}

// Timer/Interrupt Functions

// https://www.gammon.com.au/timers
void waitNSamples(uint16_t nSamples/* , const WaitCycleType & reason = WaitCycleType::null */) {
    waitCompleteSamples = false;
    
    // Reset timer settings
    TCCR1A = 0;
    TIMSK1 = 0;

    OCR1A = clockCyclesPerSample;
    if (nSamples > 0x100U) { // not worth factoring in unleap samples for less
        // -1 unleap cycle per five samples
        // -1 unleap sample per 1815 samples
        if(nSamples > clockCyclesPerSample * 5) {
            nSamples -= (nSamples / (clockCyclesPerSample * 5));
        }
        OCR1A -= (nSamples / 5); 
        if(OCR1A > 50U) { OCR1A = 50U; } // completely arbitrary, figure it out
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
    TCNT1 = 0; // Reset counter, not sure if redundant?
    TCCR1B = bit(CS10); // turn on counter, no prescale
};

// sets Timer3 to trigger an interrupt after n YM2612 cycles
// should override any active sample-waits
// I imagine behaviour is wonky for low cycle counts
bool waitNCycles(const WaitCycleType & nCycles = WaitCycleType::null) {
    if(nCycles == WaitCycleType::null) { return false; }
    waitReason = nCycles;
    waitCompleteCycles = false;
    readyToWrite        = false; // should be redundant
    
    // Reset timer settings
    TCCR3A = 0;
    TIMSK3 = 0;

    OCR3A = (uint8_t)nCycles * clockCyclesPerYM2612Cycle;

    // Wait length cannot currently overflow
    
    TIMSK3 = bit(OCIE1A);

    TCCR3A = bit(WGM11); // CTC Mode
    TCNT3 = 0; // Reset counter, not sure if redundant?
    TCCR3B = bit(CS10); // turn on counter, no prescale
    return true;
};

ISR(TIMER1_COMPA_vect) {
    overflowCount--;
    if (!overflowCount) {
        TIMSK1              = 0;
        waitCompleteSamples = true;
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
        waitCompleteSamples = true;
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
        waitCompleteSamples = true;
    }
}

ISR(TIMER3_COMPA_vect) { TIMSK3 = 0; waitCompleteCycles = true; }

// Physical controls such as encoders and buttons using PORTK pins
ISR(PCINT2_vect) {
    static byte current;
    static byte last;
    static byte changed;

    current = ENCODERPORT;
    changed = current ^ last;

    // Only A needs to trigger the interrupt
    // Does the B bit need to be checked?
    if(changed & B11000000 != 0U) {
        // https://youtu.be/v4BbSzJ-hz4?t=145
        
        if (bit(ENCODER1A - ENCODERPORT) == bit(ENCODER1B - ENCODERPORT)) {
            // counterclockwise
            encoder1--;

        } else if (bit(ENCODER1A - ENCODERPORT) ^ bit(ENCODER1B - ENCODERPORT)) {
            // clockwise
            encoder1++;
        }
    }

    last = current;
    readyToControlChange = true;
}


// Setup Functions
void setupBlock() {
    uint8_t count;

    Serial.begin(9600);

    while(!Serial) {
        Serial.begin(9600);
        delay(1000);
    }
    
    readyToLoad = true;
    count       = loadBlock();

    while (readyToLoad) {
        count += loadBlock(); // count is currently redundant
        delay(1000);
    }

    VGM::currentHeader = VGM::readVGMHeader(*inactiveBlock);

    if (*inactiveBlock) {
        activeBlock = inactiveBlock;
    } else {
        readyToLoad = true;
        count       = loadBlock();

        while (readyToLoad) {
            count += loadBlock(); // count is currently redundant
            delay(1000);
        }
    }
}

void setupControls() {
    encoder1 = &Encoder(
        ENCODER1A, ENCODER1B,
        Control(YM2612::RegisterAddress::TL, YM2612::RegisterType::TL));
}

void setupClocks() {
    // OCR1A = clockCyclesPerSample; Defined per-wait due to unleap cycles
    OCR1B = clockCyclesPerSample * 0x10U;                   // 5808
    OCR1C = clockCyclesPerSample * samplesPerClockOverflow; // 65340
}

void setupMIDI() {
    MIDI_CREATE_DEFAULT_INSTANCE();
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

    pinMode(ENCODER1A, INPUT_PULLUP);
    pinMode(ENCODER1B, INPUT_PULLUP);

    // PCICR = bit(PCIE2);

    PCMSK2 = bit(PCINT16); // Encoder1A

    digitalWrite(pIC, HIGH);
    digitalWrite(pIRQ, HIGH);
    digitalWrite(pCS, HIGH);
    digitalWrite(pRD, HIGH);
    digitalWrite(pWR, HIGH);
    digitalWrite(pA0, HIGH);
    digitalWrite(pA1, HIGH);
}

void setup() { 
    setupBlock();
    setupControls();
    setupClocks();
    // setupMIDI();
    setupPins();
    readyToWrite = true;
}

void loop() {
    if (waitCompleteCycles) {
        // using enum WaitCycleType; // C++20 only :(
        // need to pick up where we left off
        switch (waitReason) {
        case WaitCycleType::RESETPULSEWIDTH:
            YM2612::currentState.finishReset();
            break;
        case WaitCycleType::WRITEWAITWIDTH1: // continue write
            YM2612::currentState.finishUpdate(WaitCycleType::WRITEWAITWIDTH1);
            break;
        case WaitCycleType::WRITEWAITWIDTH2:
        case WaitCycleType::WRITEWAITWIDTH3: // clear for next write
            readyToWrite = true;
            break;

        default: break;
        }
    }

    if (waitCompleteSamples) {
        if (readyToControlChange) {
            if (encoder1->changeSinceLastUpdate() != 0) {
                static ControlChange tempCC =
                    ControlChange(encoder1->control, 0U);
                // TODO: Refactor so change can occur within State method.
                tempCC.ym2612Data = YM2612::currentState.getRegister(
                    tempCC.ym2612Address, tempCC.ym2612Type);

                tempCC.ym2612Data += encoder1->changeSinceLastUpdate();

                // this still seems rough. Need a proper clamp
                if (tempCC.ym2612Data > 0xFFF) { // underflow
                    tempCC.ym2612Data = 0x00;
                } else if (tempCC.ym2612Data > tempCC.maxValue) { // overflow
                    tempCC.ym2612Data = tempCC.maxValue;
                }

                YM2612::currentState.updateRegister(tempCC);
            }
            // handle before loading next command
            YM2612::Command temp;
            temp.type = YM2612::Command::Type::write;
        } else {
            readyToWrite = true;
        }
        waitCompleteSamples = false;
    }

    if (readyToTake && readyToWrite) {
        YM2612::Command temp = YM2612::currentCommandPile.takeFromPile();
        switch (temp.type) {
        case YM2612::Command::Type::null: readyToTake = false; break;
        case YM2612::Command::Type::inactive:
            readyToTake  = false;
            readyToWrite = false;
            YM2612::currentState.startReset();
            break;
        case YM2612::Command::Type::wait:
            readyToWrite = false;
            waitNSamples(temp.wait);
            break;
        case YM2612::Command::Type::write:
            readyToWrite = false;
            YM2612::currentState.updateRegister(temp);

        default: break;
        }
    }

    YM2612::Command temp;
    while (YM2612::currentCommandPile.getCount() < COMMANDPILESIZE &&
           readCommandFromBlock(temp)) {
        switch(temp.type) {
            case YM2612::Command::Type::null:
            case YM2612::Command::Type::ignore:
            break;

            default: 
                if (YM2612::currentCommandPile.addToPile(temp)) {
                readyToTake = true;
            }
            break;
        }
    }

    if (Serial && readyToLoad) { loadBlock(); }
}