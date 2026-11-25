/*
;    Project:       Open Vehicle Monitor System
;    Subproject:    Integrate VW e-Golf — comfort-bus network management (NM) wake frame
;
; Permission is hereby granted, free of charge, to any person obtaining a copy
; of this software and associated documentation files (the "Software"), to deal
; in the Software without restriction, including without limitation the rights
; to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
; copies of the Software, and to permit persons to whom the Software is
; furnished to do so, subject to the following conditions:
;
; The above copyright notice and this permission notice shall be included in
; all copies or substantial portions of the Software.
;
; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
; IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
; AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
; OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
; THE SOFTWARE.
*/

#ifndef __VWEGOLF_NM_H__
#define __VWEGOLF_NM_H__

#include <stdint.h>

// The NM frame OVMS sends to wake the comfort bus (KCAN) and keep it awake, laid out the way the
// car's telematics unit (OCU) sends it, but from a spare node id instead of the OCU's own (0x67).
// Two units sending the same id with different data at the same instant cause bus errors, so OVMS
// must never use the OCU's id: cars with an OCU have it on this bus.
//
// Layout: static analysis of OCU firmware 0561 (Vector NMHigh). Checked against the car's own nodes
// in KCAN captures: NM every 200 ms, 3.8 s of "repeat message" after a wake, byte 1 always 00,
// byte 2 = last state change (01 just woken, 04 normal operation, 08 back from ready-sleep),
// byte 3 = wake reason (01 "woken by bus traffic" on every node OVMS woke).
//   byte 0    source node
//   byte 1    00
//   byte 2    0x40 active wake-up ("CarWakeUp", set only by the unit that requests the bus)
//             | last state change: 0x01 bus sleep -> repeat message, 0x04 repeat message -> normal
//   byte 3    wake reason: 0x80 + n, one per remote function (Reason)
//   byte 4    keep-awake reasons: 0x04 own communication request + the reason's job bit (JobBit)
//   byte 5-7  00
// The OCU sends it every 200 ms; OVMS sends it about once a second (Ticker1), which every test so far
// has shown to be enough. The first frame on a sleeping bus is never acknowledged: the MCP2515 driver
// gives up once error-passive (the OCU's controller keeps retrying), so callers send it again.
namespace vwegolf_nm {

constexpr uint32_t kIdBase     = 0x1B000000u;  // NM id = kIdBase | node
constexpr uint8_t  kCarWakeUp  = 0x40;         // byte 2: active wake-up request
constexpr uint8_t  kToRepeat   = 0x01;         // byte 2: bus sleep -> repeat message
constexpr uint8_t  kToNormal   = 0x04;         // byte 2: repeat message -> normal operation
constexpr uint8_t  kComRequest = 0x04;         // byte 4: own communication request
constexpr uint32_t kRepeatMs   = 3800;         // length of the repeat-message phase after a wake

// Wake reasons (byte 3), as the OCU sends them for its remote functions.
enum Reason : uint8_t {
    REASON_START_CHARGE  = 0x80,
    REASON_STOP_CHARGE   = 0x81,
    REASON_TIMERS        = 0x82,  // change departure timers
    REASON_PROFILES      = 0x83,  // change charge profiles
    REASON_SETTINGS      = 0x84,  // change climate / charge settings
    REASON_START_CLIMATE = 0x85,
    REASON_STOP_CLIMATE  = 0x86,
    REASON_STATUS        = 0x87,  // status / SOC query
    REASON_START_HEATING = 0x88,  // window heating on
    REASON_STOP_HEATING  = 0x89,  // window heating off
};

// Byte-4 job bit for a reason: 0x08 charging, 0x10 timers / profiles / settings, 0x20 climate and
// window heating, 0x40 status.
inline uint8_t JobBit(Reason r) {
    switch (r) {
        case REASON_START_CHARGE:
        case REASON_STOP_CHARGE:
            return 0x08;
        case REASON_TIMERS:
        case REASON_PROFILES:
        case REASON_SETTINGS:
            return 0x10;
        case REASON_START_CLIMATE:
        case REASON_STOP_CLIMATE:
        case REASON_START_HEATING:
        case REASON_STOP_HEATING:
            return 0x20;
        case REASON_STATUS:
            return 0x40;
    }
    return 0;
}

// The NM frames of one sender. Start() when a wake request begins, Frame() for each NM frame while
// it lasts, Release() once at its end: the OCU then sends one more frame with the wake-up request
// and the job bit cleared, and goes quiet. Times are in ms from any free-running clock.
class Sender {
 public:
    explicit Sender(uint8_t node) : m_node(node) {}

    uint32_t Id() const { return kIdBase | m_node; }

    void Start(Reason reason, uint32_t nowMs) {
        m_reason   = reason;
        m_start_ms = nowMs;
    }

    void Frame(uint32_t nowMs, uint8_t out[8]) const {
        Build(out, (uint8_t)(kCarWakeUp | State(nowMs)), (uint8_t)(kComRequest | JobBit(m_reason)));
    }

    void Release(uint32_t nowMs, uint8_t out[8]) const {
        Build(out, State(nowMs), kComRequest);
    }

 private:
    // Repeat message for the first 3.8 s of the request, then normal operation.
    uint8_t State(uint32_t nowMs) const {
        return (uint32_t)(nowMs - m_start_ms) < kRepeatMs ? kToRepeat : kToNormal;
    }

    void Build(uint8_t out[8], uint8_t byte2, uint8_t byte4) const {
        out[0] = m_node;
        out[1] = 0x00;
        out[2] = byte2;
        out[3] = m_reason;
        out[4] = byte4;
        out[5] = 0x00;
        out[6] = 0x00;
        out[7] = 0x00;
    }

    uint8_t  m_node;
    Reason   m_reason   = REASON_STATUS;
    uint32_t m_start_ms = 0;
};

}  // namespace vwegolf_nm

#endif  // __VWEGOLF_NM_H__
