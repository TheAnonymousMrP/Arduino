#ifndef _sn76489_h_
#define _sn76489_h_ 1

#ifndef _digitalWriteFast_h_
#include <digitalWriteFast.h>
#endif

// SN76489 Pins
#define SN_MEM_DATA  PORTL // Pins 35-42
#define SN_PIN_DATA  35U   // PORTL
#define SN_PIN_WE    51U   // PG0
#define SN_PIN_CE    70U   // PG2
#define SN_PIN_READY 52U   // PG1
#define SN_PIN_D0    35U   // PL0

uint8_t reverse(const uint8_t n) {
    // https://stackoverflow.com/questions/2602823/in-c-c-whats-the-simplest-way-to-reverse-the-order-of-bits-in-a-byte
    // Index 1==0b0001 => 0b1000
    // Index 7==0b0111 => 0b1110
    // etc
    static const unsigned char lookup[16] = {
        0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe,
        0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf,
    };
    // Reverse the top and bottom nibble then swap them.
    return (lookup[n & 0b1111] << 4) | lookup[n >> 4];
}

namespace SN76489 {
    uint32_t operator"" _Hz(unsigned long long);

    const uint32_t clockSpeed = 3579545_Hz;

    typedef uint8_t Write;

    class Format {
        uint8_t data;

        public:
            enum Type: uint8_t {
                frequency,
                noise,
                attenuator
            };

            void pack(const bool & isFirstByte, const uint8_t & address, const uint8_t & data) {

            };
            void pack(const bool & isFirstByte, const uint8_t & address, const bool & feedback, const uint8_t & shift) {

            };
    };

    void writeToChip(const uint8_t & data) {
        SN_MEM_DATA = data;

        digitalWriteFast(SN_PIN_CE, LOW);
        digitalWriteFast(SN_PIN_WE, LOW);

        // wait for READY??
        // Apparently about 32 clock cycles, may not be worth an interrupt.
        // Need to prevent interrupts while waiting though?
        while (digitalReadFast(SN_PIN_READY) == LOW) {}

        digitalWriteFast(SN_PIN_CE, HIGH);
        digitalWriteFast(SN_PIN_WE, HIGH);
    }

    class State {
        uint16_t _tone[3];
        uint8_t  _noise;

      public:
        bool isReadyToAccess = false;

        State() : _tone() { isReadyToAccess = true; };

        enum Tone : uint8_t {
            Tone1 = 0,
            Tone2 = 1,
            Tone3 = 2,
            Noise = 3,
            null  = 4
        };

        // TODO: Test these. Consider inlining the assigns.
        void toneFrequency(const Tone & tone, const uint16_t & toneFrequency,
                           const bool & firstByte) {
            if (firstByte) {
                _tone[tone] &= 0xFFF0U;
                _tone[tone] += toneFrequency;
            } else {
                _tone[tone] &= 0xF00FU;
                _tone[tone] |= toneFrequency << 4;
            }
        }

        void toneAttenuation(const Tone &     tone,
                             const uint16_t & toneAttenuation) {
            _tone[tone] = (_tone[tone] & 0x03FFU) + (toneAttenuation << 12);
        }

        void noiseControl(const uint8_t & control) {
            _noise = (_noise & B11110000) + (control & B00001111);
        }

        void noiseFeedback(const bool & feedback) {
            _noise &= B11110011;
            if (feedback) { _noise |= B00000100; }
        }

        void noiseFrequency(const uint8_t & frequency) {
            _noise &= B11110100;
            _noise += frequency & B00000011;
        }

        void noiseAttenuation(const uint8_t & attenuation) {
            _noise = (_noise & B00001111) + (attenuation << 4);
        }

        const uint16_t toneFrequency(const Tone & tone) {
            return _tone[tone] & 0x03FFU;
        }

        const uint8_t toneAttenuation(const Tone & tone) {
            return _tone[tone] >> 12;
        }

        const uint8_t noiseControl() { return _noise & B00001111; }

        const bool noiseFeedback() {
            return (_noise & B00000100 > 0) ? true : false;
        }

        const uint8_t noiseFrequency() { return (_noise & B00000011); }

        bool updateRegister(const Write & data) {
            static Tone lastTone = Tone::null;
            // bit 7 decides if first byte (true) or second byte (false)
            const bool firstByte = ((data & B10000000) > 0U) ? true : false;

            if (firstByte) {
                // First bytes have bits 6-4 determine address to be written
                const Tone select = (Tone)((data >> 5) & B00000011);
                const bool isAttenuation =
                    ((data & B00010000) > 0) ? true : false;
                // last 4 bits are data
                if (select < Tone::Noise) {
                    (isAttenuation)
                        ? toneFrequency(select, data & B00001111, firstByte)
                        : toneAttenuation(select, data & B00001111);
                } else if (select == Tone::Noise) {
                    (isAttenuation) ? noiseControl(data & B00001111)
                                    : noiseAttenuation(data & B00001111);
                } else {
                    return false; // oh no
                }
                lastTone = select;
            } else {
                if (lastTone < Tone::Noise) {
                    toneFrequency(lastTone, data & B00111111, firstByte);
                } else {
                    return false; // oh no
                }
                lastTone = Tone::null;
            }

            // I remain unsure on the bit numbering situation
            isReadyToAccess = false;
            writeToChip(reverse(data));
            isReadyToAccess = true;
            return true;
        }

        void reset(void) { isReadyToAccess = false; }
    } currentState;

} // namespace SN76489
#endif