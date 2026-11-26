/*
;    Project:       Open Vehicle Monitor System
;	 Subproject:    Integrate VW e-Golf
;
;    (C) 2026  Erick Fuentes <fuentes.erick@gmail.com>
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

#ifndef __VEHICLE_VWEG_H__
#define __VEHICLE_VWEG_H__

#include "can.h"
#include "ovms_command.h"
#include "ovms_config.h"
#include "ovms_log.h"
#include "ovms_metrics.h"
#include "vehicle.h"
#include "vehicle_vwegolf_bat_ctrl.h"  // BatteryControl: remote climate + charge (self-contained)

// Poll states — index matches the timing array in poll_pid_t entries.
#define VWEGOLF_OFF 0       // All systems sleeping
#define VWEGOLF_AWAKE 1     // Base systems online
#define VWEGOLF_CHARGING 2  // Base systems online and charging
#define VWEGOLF_ON 3        // All systems online, drivable

// KCAN goes silent within a few seconds of the car sleeping. After this many
// consecutive Ticker1 ticks with no incoming frames, the bus is treated as offline
// and we suppress OCU keepalive transmission to avoid accumulating TX errors.
#define VWEGOLF_BUS_TIMEOUT_SECS 10

// `wakeup` session (WakeKcanBus): OVMS holds its NM wake request this long, like the OCU's 10 s
// status wake. The car's gateway keeps the bus up ~15-17 s after an active wake anyway, enough for
// one BCU heartbeat (~13 s). The 0x5A7 frame stops with it.
#define VWEGOLF_WAKE_HOLD_SECS 10
// `wakeup` reports success once the BCU is heard on KCAN; it answers ~0.3 s after the wake frame
// gets through (seen on the car).
#define VWEGOLF_WAKE_ANSWER_MS 3000

class OvmsVehicleVWeGolf : public OvmsVehicle {
 public:
    OvmsVehicleVWeGolf();
    ~OvmsVehicleVWeGolf();

    void IncomingFrameCan3(CAN_frame_t* p_frame) override;
    // FCAN (can2, listen-only): charging fallback while KCAN sleeps (see the constructor).
    void IncomingFrameCan2(CAN_frame_t* p_frame) override;

    vehicle_command_t CommandHorn();
    vehicle_command_t CommandPanic();
    vehicle_command_t CommandIndicators();
    vehicle_command_t CommandMirrorFoldIn();
    vehicle_command_t CommandLock(const char* pin) override;
    vehicle_command_t CommandUnlock(const char* pin) override;
    vehicle_command_t CommandWakeup() override;
    vehicle_command_t CommandClimateControl(bool enable) override;
    // Charge control — shares the BatteryControl (LSG 0x25) command path with climate.
    vehicle_command_t CommandSetChargeCurrent(uint16_t limit) override;  // persistent maxCurrent edit
    vehicle_command_t CommandStartCharge() override;  // immediate charge start (on-car validated)
    vehicle_command_t CommandStopCharge() override;   // immediate charge stop (op-specific; blocked while climate on)
    void SendOcuHeartbeat();
    void SendWakeNm(bool release);  // one NM frame of the `wakeup` session (or its last one)
    bool WakeKcanBus();             // false if the KCAN controller could not be restarted
    void EndWakeSession(const char* why, bool release);

 protected:
    void Ticker1(uint32_t ticker) override;

 private:
    // Decode one frame into metrics. Called for every KCAN frame, and for the FCAN charging set while
    // KCAN sleeps — so it must not touch KCAN bus state (idle counter, OCU keepalive).
    void DecodeFrame(CAN_frame_t* p_frame);
    // Program can2's MCP2515 acceptance filter so only the FCAN charging set reaches the CPU.
    void SetFcanFilter();

    // Awake / drivable state, derived from the terminal (0x3C0 KL_15) and READY
    // (0x391) frames in IncomingFrameCan3.
    bool m_kl15_on = false;
    bool m_drivetrain_ready = false;
    // HV electrical system active (0x187 d[1] bit 7 = opendbc Motor_EV_01 HV_Bordnetz_aktiv). While the
    // HV system is up the DC-DC converter supplies the 12V network -> drives v.e.charging12v.
    bool m_hv_active = false;

    // Seconds since the last KCAN (can3) frame arrived. Reset to 0 in IncomingFrameCan3,
    // incremented each second in Ticker1. Bus is alive while < VWEGOLF_BUS_TIMEOUT_SECS.
    // Initialized to timeout so we treat the bus as offline at cold boot.
    uint8_t m_bus_idle_ticks = VWEGOLF_BUS_TIMEOUT_SECS;

    // Seconds since the car's own OCU was last seen: its KN frame 0x17F00067 (every 500 ms while it is
    // awake), an NM frame from its node 0x67 (OVMS never sends as 0x67), or a non-zero 0x5A7. While it
    // is present, OVMS sends no 0x5A7: two senders with different data on one id cause bus errors.
    // Initialized to timeout so cold boot treats the OEM OCU as absent.
    uint8_t m_oem_ocu_idle_ticks = VWEGOLF_BUS_TIMEOUT_SECS;

    // `wakeup` session: NM wake request as the spare node + the 0x5A7 frame (which carries the
    // mirror / lock / horn request flags), both once a second from Ticker1, for
    // VWEGOLF_WAKE_HOLD_SECS. Only started by `wakeup`, so OVMS never shows up as a node unasked.
    bool m_wake_session = false;
    uint8_t m_wake_secs = 0;           // seconds since the session started
    bool m_wake_ocu_noted = false;     // "OEM OCU seen" logged for this session
    vwegolf_nm::Sender m_wake_nm{VWEGOLF_NM_WAKE_NODE};
    // BCU status frames (0x17332510) received on KCAN; `wakeup` waits for this to change. Written by
    // the CAN RX task, read by the command's task.
    volatile uint32_t m_kcan_bcu_rx = 0;

    // BatteryControl (LSG 0x25) controller: remote climate AND charge, which share one BAP
    // command path (handshake -> GET profile 0 -> RMW-arm -> trigger). Self-contained (see
    // vehicle_vwegolf_bat_ctrl.h): owns its own spare-node NM wake, independent of the 0x5A7
    // heartbeat above. Ticker1 forwards it a tick; IncomingFrameCan3 forwards it the BCU status
    // (0x17332510) and clima ECU status (0x5EA) frames.
    VWeGolfBatteryControl m_batctrl;

    bool m_mirror_fold_in_requested = false;
    bool m_horn_requested = false;
    bool m_indicators_requested = false;
    bool m_panic_requested = false;
    bool m_unlock_requested = false;
    bool m_lock_requested = false;

    // VIN assembly state. Frame 0x6B4 carries the 17-char VIN split across three frames
    // identified by data[0]. We collect all three before committing to the metric.
    uint8_t m_vin_parts_received = 0;
    char m_vin_buf[18] = {};
    // Regenerative-braking strength, decoded from 0x187 (see IncomingFrameCan3).
    // The e-Golf's five regen levels as a 0..4 scale (least->most): D0 (coast) = 0,
    // D1 = 1, D2 = 2, D3 = 3, B = 4. -1 = N/A (not in gear D or B).
    OvmsMetricInt* m_recup_level = nullptr;

    // Body signals (comfort bus): window positions, per-door locks, side lights.
    // The door / window frames are driver / passenger based; m_rhd (from 0x32A) maps them to left /
    // right, as the OEM OCU does. Array index: 0 fl, 1 fr, 2 rl, 3 rr.
    bool m_rhd = false;                          // right-hand drive per 0x32A; LHD until the car says so
    OvmsMetricBool* m_rhd_metric = nullptr;      // xvg.v.rhd
    OvmsMetricInt*  m_window[4] = {};            // xvg.v.d.window.*: % open, 0 = closed, -1 = not seen
    OvmsMetricBool* m_door_lock[4] = {};         // xvg.v.d.lock.*: door locked
    int8_t m_lock_mode = -1;                     // 0x583 d[7]: 1 locked, 0 unlocked, -1 not seen yet
    // v.e.locked = locking mode locked and every door locked (see UpdateLocked in the .cpp).
    void UpdateLocked();
    OvmsMetricBool* m_sidelights = nullptr;      // xvg.v.e.sidelights: light switch on side lights
    // Metric index for a driver / passenger based signal. seat: 0 driver, 1 passenger, 2 rear driver
    // side, 3 rear passenger side. LHD: the same; RHD: left and right swapped.
    uint8_t SideIndex(uint8_t seat) const { return m_rhd ? (uint8_t)(seat ^ 1) : seat; }

    // `xvg charge profile list` handler: fetch the car's charge profiles (charge locations) via the
    // BatteryControl controller and print them to the writer. Blocks (bounded) while the BCU is woken
    // and answers. Read-only — it never writes a profile.
    void CommandListProfiles(OvmsWriter* writer);
    // `xvg charge profile set` handler: RMW selected fields of one profile (profile 0 = current/minsoc/
    // temp; charge locations 1-3 = flags/current/soc). Blocks (bounded) for the controller's result.
    void CommandSetProfile(OvmsWriter* writer, int argc, const char* const* argv);

    // `xvg windowheat on|off [both|front|rear]` / `xvg seatheat on|off [front|fl|fr]`: fire-and-forget
    // remote heating (like climatecontrol). `xvg seatheat clear`: profile-0 RMW dropping the stored seat
    // bits (blocks).
    void CommandWindowHeat(OvmsWriter* writer, bool enable, int argc, const char* const* argv);
    void CommandSeatHeat(OvmsWriter* writer, bool enable, int argc, const char* const* argv);
    void CommandSeatHeatClear(OvmsWriter* writer);

    // `xvg charge timer ...` handlers. list = read-only; set/enable/disable/clear write to the car
    // (recurring timers only). Each blocks (bounded) for the BatteryControl controller's result.
    void CommandListTimers(OvmsWriter* writer);
    void CommandSetTimer(OvmsWriter* writer, int argc, const char* const* argv);
    void CommandTimerEnable(OvmsWriter* writer, int argc, const char* const* argv, bool enable);
    void CommandClearTimer(OvmsWriter* writer, int argc, const char* const* argv);
    // Block (bounded) polling the BatteryControl CLI result; true = LIST_READY, else prints the reason.
    bool PollBatCtrlResult(OvmsWriter* writer);

#ifdef VWEGOLF_NATIVE_TEST
 public:
    uint8_t test_bus_idle_ticks() const { return m_bus_idle_ticks; }
    VWeGolfBatteryControl& test_batctrl() { return m_batctrl; }
#endif
};

#endif  // __VEHICLE_VWEG_H__
