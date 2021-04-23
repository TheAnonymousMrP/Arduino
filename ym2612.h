#ifndef _ym2612_h_
#define _ym2612_h_ 1

#ifndef _digitalWriteFast_h_
#include <digitalWriteFast.h>
#endif

namespace YM2612 {
    // need to include a bunch of 'using' to get any value from this
    unsigned long long operator"" _pin(unsigned long long);

    unsigned long long           operator"" _Hz(unsigned long long);
    constexpr unsigned long long operator"" _cycles(unsigned long long n) {
        return n;
    };

    constexpr uint8_t operator"" _address(unsigned long long n) { return n; };

        // YM2612 Pins
#define YM_PIN_CS   78U   // PA0
#define YM_PIN_IC   77U   // PA1
#define YM_PIN_WR   76U   // PA2
#define YM_PIN_RD   75U   // PA3
#define YM_PIN_A0   74U   // PA4
#define YM_PIN_A1   73U   // PA5
#define YM_PIN_IRQ  72U   // PA6
#define YM_PIN_D0   53U   // PC0
#define YM_PIN_DATA 53U   // PORTC (53-60)
#define YM_MEM_DATA PORTC // Pins 53-60

    const uint32_t clockSpeed = 7670453_Hz;

    enum RegisterAddress : uint8_t { // Address
        A_Test1       = 0x21_address,
        A_LFO         = 0x22_address,
        A_NODATA      = 0x23_address,
        A_TimerA      = 0x24_address,
        A_TimerB      = 0x26_address,
        A_TimerLoad   = 0x27_address,
        A_TimerEnable = 0x27_address,
        A_TimerReset  = 0x27_address,
        A_TimerMode   = 0x27_address,
        A_Channel     = 0x28_address,
        A_Slot        = 0x28_address,
        A_DACData     = 0x2A_address,
        A_DACSelect   = 0x2B_address,
        A_Test2       = 0x2C_address,
        // These have a byte for each operator for each channel (see Page 16).
        // TODO: Figure out how to factor in an offset
        A_DT    = 0x30_address,
        A_Multi = 0x30_address,
        A_TL    = 0x40_address,
        A_KS    = 0x50_address,
        A_AR    = 0x50_address,
        A_AM    = 0x60_address,
        A_DR    = 0x60_address,
        A_SR    = 0x70_address,
        A_SL    = 0x80_address,
        A_RR    = 0x80_address,
        A_SSGEG = 0x90_address,
        // These have a byte for each channel.
        A_FNum1 = 0xA0_address,
        A_FNum2 = 0xA4_address,
        A_Block = 0xA4_address,
        // These have a byte for each operator (channel 3).
        // TODO: Figure out how to factor in an offset.
        A_Channel3FNum1 = 0xA8_address,
        A_Channel3FNum2 = 0xAC_address,
        A_Channel3Block = 0xAC_address,
        // These have a byte for each channel.
        A_FB   = 0xB0_address,
        A_Algo = 0xB0_address,
        A_LR   = 0xB4_address,
        A_AMS  = 0xB4_address,
        A_PMS  = 0xB4_address
    };

    enum RegisterType : uint8_t { // Size | Offset << 4 | Bank << 7
        T_Test1       = 8 | (0 << 4),
        T_LFO         = 4 | (0 << 4),
        T_TimerA      = 10 | (0 << 4),
        T_TimerB      = 8 | (0 << 4),
        T_TimerLoad   = 2 | (0 << 4),
        T_TimerEnable = 2 | (2 << 4),
        T_TimerReset  = 2 | (4 << 4),
        T_TimerMode   = 2 | (6 << 4),
        T_Channel     = 3 | (0 << 4),
        T_Slot        = 4 | (4 << 4),
        T_DACData     = 8 | (0 << 4),
        T_DACSelect   = 1 | (7 << 4),
        T_Test2       = 8 | (0 << 4),
        // These have a byte for each operator for each channel (see Page 16).
        // TODO: Figure out how to factor in an offset
        T_DT    = 3 | (4 << 4),
        T_Multi = 4 | (0 << 4),
        T_TL    = 7 | (0 << 4),
        T_KS    = 2 | (6 << 4),
        T_AR    = 5 | (0 << 4),
        T_AM    = 1 | (7 << 4),
        T_DR    = 5 | (0 << 4),
        T_SR    = 5 | (0 << 4),
        T_SL    = 4 | (4 << 4),
        T_RR    = 4 | (0 << 4),
        T_SSGEG = 4 | (0 << 4),
        // These have a byte for each channel.
        T_FNum1 = 8 | (0 << 4),
        T_FNum2 = 3 | (0 << 4),
        T_Block = 3 | (3 << 4),
        // These have a byte for each operator (channel 3).
        // TODO: Figure out how to factor in an offset.
        T_Channel3FNum1 = 8 | (0 << 4),
        T_Channel3FNum2 = 3 | (0 << 4),
        T_Channel3Block = 3 | (3 << 4),
        // These have a byte for each channel.
        T_FB   = 3 | (3 << 4),
        T_Algo = 3 | (0 << 4),
        T_LR   = 2 | (6 << 4),
        T_AMS  = 3 | (4 << 4),
        T_PMS  = 3 | (0 << 4)
    };

    enum struct WaitCycleType : uint8_t {
        null                   = 0U,
        error                  = 0U,
        RESETPULSEWIDTH        = 192_cycles,
        WRITEWAITWIDTH1        = 17_cycles,
        WRITEWAITWIDTH2        = 83_cycles,
        WRITEWAITWIDTH3        = 47_cycles,
        ACCESSPROHIBITIONWIDTH = WRITEWAITWIDTH1
    };

    struct Write {
        RegisterAddress address;
        uint8_t         data;
        bool            bank;
    };

    uint8_t getMask(RegisterType type) {
        const uint8_t size   = type & B00001111;
        const uint8_t offset = (type >> 4U & B00000111);

        return (0xFF >> (8 - size)) << offset;
    }

    void inactiveDataBus() {
        if (portModeRegister(YM_PIN_D0) || portOutputRegister(YM_PIN_D0)) {
            for (int i = 0; i < 8; i++) {
                pinMode(YM_PIN_D0 + i, INPUT);
            }
        }

        digitalWriteFast(YM_PIN_CS, HIGH);
        digitalWriteFast(YM_PIN_RD, HIGH);
        digitalWriteFast(YM_PIN_WR, HIGH);
        digitalWriteFast(YM_PIN_A0, HIGH);
        digitalWriteFast(YM_PIN_A1, HIGH);
    }

    uint8_t readDataBus() {
        digitalWriteFast(YM_PIN_A0, LOW);
        digitalWriteFast(YM_PIN_A1, LOW);

        __asm__ __volatile__("NOPX 1"); // tAS, min 10ns
        digitalWriteFast(YM_PIN_CS, LOW);
        digitalWriteFast(YM_PIN_RD, LOW);
        // no need to change mode for pins

        __asm__ __volatile__("NOPX 21"); // tACC, min 250ns (I think)

        uint8_t status = YM_PIN_DATA;

        __asm__ __volatile__("NOPX 9"); // tCSR & tRW, min 100ns (kind of)
        digitalWriteFast(YM_PIN_RD, HIGH);
        digitalWriteFast(YM_PIN_CS, HIGH);

        __asm__ __volatile__("NOPX 1"); // tAH & tDH, min 10ns
        // no need to change mode for pins

        return status;
    }

    void startReset() {
        digitalWriteFast(YM_PIN_IC, LOW);
    }

    void finishReset() {
        digitalWriteFast(YM_PIN_IC, HIGH);
    }

    void writeToChip(uint8_t data, bool bank0or1 = false) {
        if (bank0or1) {
            digitalWriteFast(YM_PIN_A0, LOW);
            digitalWriteFast(YM_PIN_A1, HIGH);
        } else {
            digitalWriteFast(YM_PIN_A0, HIGH);
            digitalWriteFast(YM_PIN_A1, LOW);
        }

        __asm__ __volatile__("NOPX 1"); // tAS, min 10ns
        digitalWriteFast(YM_PIN_CS, LOW);
        digitalWriteFast(YM_PIN_WR, LOW);
        for (int i = 0; i < 8; i++) {
            pinMode(YM_PIN_D0 + i, OUTPUT);
        }

        PORTC |= data;

        __asm__ __volatile__("NOPX 17"); // tCSW & tWW, min 200ns (kind of)
        digitalWriteFast(YM_PIN_WR, HIGH);
        digitalWriteFast(YM_PIN_CS, HIGH);

        __asm__ __volatile__("NOPX 1"); // tAH, min 10ns
        digitalWriteFast(YM_PIN_A0, LOW);
        digitalWriteFast(YM_PIN_A1, LOW);

        __asm__ __volatile__("NOPX 1"); // tWDH - tAH, min 10ns
        for (int i = 0; i < 8; i++) {
            pinMode(YM_PIN_D0 + i, INPUT);
        }
    }

    const WaitCycleType writeWaitWidth(bool fromAddressOrData, RegisterAddress address) {
        WaitCycleType type = WaitCycleType::null;

        if (fromAddressOrData) {
            switch (address) {
            case 0x21_address ... 0x9F_address: type = WaitCycleType::WRITEWAITWIDTH2; break;
            case 0xA0_address ... 0xB6_address: type = WaitCycleType::WRITEWAITWIDTH3; break;
            default: break;
            }
        } else {
            type = WaitCycleType::WRITEWAITWIDTH1;
        }
        return type;
    }

    // Maintains a copy of the YM2612's memory and handles data transfer
    class State {
        static const auto sharedStart  = RegisterAddress::A_Test1;
        static const auto channelStart = RegisterAddress::A_DT;
        static const auto channelEnd   = RegisterAddress::A_PMS; // Also LR, AMS
        uint8_t           sharedRegister[channelStart - sharedStart];
        uint8_t           channelRegister0[channelEnd - channelStart];
        uint8_t           channelRegister1[channelEnd - channelStart];
        Write             lastWrite;
        bool              lastInstructionWasAWrite = false;
        // TODO: Remove unused addresses from internal state

      public:
        bool isReadyToAccess = false;

        State()
            : sharedRegister(), channelRegister0(),
              channelRegister1(){}; // prep some default state

        // evaluates if this State is initialised and active
        // operator bool() const {}

        // Returns 0/false if invalid
        const uint16_t getRegister(RegisterAddress address,
                                   RegisterType    type) {
            const uint8_t size     = type & B00001111;
            const uint8_t offset   = (type >> 4) & B00000111;
            const bool    bank0Or1 = (type >> 15) ? true : false;

            if(isAddressValid(address, bank0Or1)) {
                // return *getStateMemoryPointer(address, bank0Or1);
            } else {
                return false;
            }

            uint8_t * temp = getStateMemoryPointer(address, bank0Or1);

            if (size == 8U) {
                return *temp;
            } else if (size > 8U) {
                // need something for TimerA and FNum(?)
            } else {
                return (*temp & getMask(type)) >> offset;
            }
        }

        bool isAddressValid(uint8_t address, bool bank) {
            if (address > sharedStart ||
                address < channelEnd) {
                return false;
            } else if(address == RegisterAddress::A_NODATA) {
                return false;
            } else if(address >= channelStart && address <= channelEnd) {
                if (address % 4U == 3U) {
                    return false; // invalid address
                }
            }
            return true;
        }

        uint8_t * getStateMemoryPointer(uint8_t address,
                                        bool    bank) {
            if (isAddressValid(address, bank)) {
                switch (address) {
                case RegisterAddress::A_TimerA:
                case RegisterAddress::A_TimerA + 1:
                    // requires additional step?
                case sharedStart:
                case RegisterAddress::A_LFO:
                case RegisterAddress::A_TimerB... 0x2F:
                    return &sharedRegister[address - sharedStart];
                    break;
                case channelStart ... channelEnd:
                    return (bank) ? &channelRegister1[address - channelStart]
                                  : &channelRegister0[address - channelStart];
                    break;
                }
            } else {
            }
        }

        // Does NOT validate address
        const Write convertToWrite(RegisterAddress address,
                                   RegisterType    type,
                                   uint16_t        data = 0U) {
            const uint8_t size     = type & B00001111;
            const uint8_t offset   = (type >> 4) & B00000111;
            const bool    bank0Or1 = (type >> 15) ? true : false;

            uint8_t temp;
            if (size == 8U) {
                temp = (uint8_t)data;
            } else if (size > 8U) {
                // need something for TimerA and FNum(?)
            } else {
                temp &= ~getMask(type);
                temp |= (uint8_t)data << offset;
            }

            Write write;
            write.address = address;
            write.data    = temp;
            write.bank    = bank0Or1;
            return write;
        }

        // method to write (presumably) validated data to YM2612
        const WaitCycleType updateRegister(const Write & write) {
            if (isAddressValid(write.address, write.bank)) {
                uint8_t * statePointer =
                    getStateMemoryPointer(write.address, write.bank);
                *statePointer = write.data;

                const WaitCycleType waitType = startUpdate(write);
                if (waitType != WaitCycleType::null) {
                    lastWrite                = write;
                    lastInstructionWasAWrite = true;
                }
                return waitType;
            } else {
                return WaitCycleType::error;
            }
        }

        // method to write specific data to YM2612
        const WaitCycleType updateRegister(RegisterAddress address,
                                           RegisterType    type,
                                           uint16_t        data = 0U) {
            Write temp = convertToWrite(address, type, data);

            return updateRegister(temp);
        }

        // empty internal memory to match YM2612
        // may need some initial state
        void startReset() {
            isReadyToAccess = false;
            YM2612::startReset();
            memset(sharedRegister, 0U, sizeof(sharedRegister));
            memset(channelRegister0, 0U, sizeof(channelRegister0));
            memset(channelRegister1, 0U, sizeof(channelRegister1));
            lastWrite                = Write();
            lastInstructionWasAWrite = false;
        }

        void finishReset() {
            YM2612::finishReset();
            isReadyToAccess = true;
        }

        const WaitCycleType startUpdate(const Write & write) {
            isReadyToAccess = false;
            bool processAddress = true;

            // check address against last instruction if appropriate
            if (lastInstructionWasAWrite) {
                processAddress != write.bank ==
                    lastWrite.bank && write.address == lastWrite.address;
            }

            if (processAddress) {
                writeToChip(write.address, write.bank);
                return writeWaitWidth(false, write.address);
            } else {
                writeToChip(write.data, write.bank);
                return writeWaitWidth(true, write.address);
            }
        }

        void finishUpdate(WaitCycleType reason) {
            writeToChip(lastWrite.data, lastWrite.bank);
            writeWaitWidth(true, lastWrite.address);
        }

    } currentState;

    namespace Test {
        void sizeTest() {
            sizeof(Write);
            sizeof(RegisterAddress);
            sizeof(RegisterType);
        }

        void maskTest() { }

        void stateTest() { }

        class StateTest : State {
          public:
            void testMasks() {}
        };
    } // namespace Test
};    // namespace YM2612

#endif //__ym2612_h_
