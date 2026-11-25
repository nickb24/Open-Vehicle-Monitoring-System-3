// test_wake.cpp — `wakeup` and the NM wake frame (egolf/nm.h): frame layout per wake reason, the
// battery-control commands' wake frames, and the `wakeup` session (wake, answer check, 1 Hz NM and
// 0x5A7, 10 s hold, release frame, standing down for the car's own OCU).

#include "mock/mock_ovms.hpp"
#include "../src/vehicle_vwegolf.h"

#include <cstdio>
#include <cstring>
#include <vector>

extern int tests_run;
extern int tests_passed;

#define CHECK(cond, msg) do { \
    tests_run++; \
    if (cond) { tests_passed++; printf("  PASS: %s\n", msg); } \
    else       { printf("  FAIL: %s\n", msg); } \
} while(0)

static const uint32_t kNmId     = 0x1B00007D;  // spare node 0x7D
static const uint32_t kBcuSts   = 0x17332510;
static const uint8_t  kWindows  = bap::egolf::PO2_WINDOW_HEATER_FRONT | bap::egolf::PO2_WINDOW_HEATER_REAR;

static bool same(const uint8_t* a, std::initializer_list<uint8_t> b) {
    size_t i = 0;
    for (uint8_t x : b) if (a[i++] != x) return false;
    return true;
}

static OvmsVehicleVWeGolf* make_vehicle() {
    g_metrics     = MetricStore{};
    g_tick_count  = 0;
    g_delay_hook  = nullptr;
    MyNotify      = OvmsNotify{};
    return new OvmsVehicleVWeGolf();
}

static void rx(OvmsVehicleVWeGolf* v, uint32_t id, std::initializer_list<uint8_t> bytes) {
    CAN_frame_t f{};
    f.MsgID = id;
    f.origin = v->m_can3;
    f.FIR.B.DLC = static_cast<uint8_t>(bytes.size());
    int i = 0;
    for (uint8_t b : bytes) f.data.u8[i++] = b;
    v->IncomingFrameCan3(&f);
}

// One second: a KCAN frame keeps the bus alive (if asked), then Ticker1.
static void tick(OvmsVehicleVWeGolf* v, bool keep_alive = true) {
    g_tick_count += 1000;
    if (keep_alive) rx(v, 0x131, {0, 0, 0, 0x64, 0, 0, 0, 0});
    static_cast<OvmsVehicle*>(v)->Ticker1(1);
}

static std::vector<TxRecord> nm_frames(canbus* bus) {
    std::vector<TxRecord> out;
    for (auto& r : bus->tx_log) if (r.extended && r.id == kNmId) out.push_back(r);
    return out;
}

static std::vector<TxRecord> frames_5a7(canbus* bus) {
    std::vector<TxRecord> out;
    for (auto& r : bus->tx_log) if (!r.extended && r.id == 0x5A7) out.push_back(r);
    return out;
}

// `wakeup` on a sleeping bus where the BCU answers during the wait (its BAP Reset 09 42).
static vehicle_command_t wake_answered(OvmsVehicleVWeGolf* v) {
    int delays = 0;
    g_delay_hook = [&](TickType_t t) {
        g_tick_count += t;
        if (++delays == 3) rx(v, kBcuSts, {0x09, 0x42});  // 5 ms, 50 ms, then the first 100 ms wait
    };
    vehicle_command_t r = v->CommandWakeup();
    g_delay_hook = nullptr;
    return r;
}

// ---------------------------------------------------------------------------
// NM frame layout
// ---------------------------------------------------------------------------

void test_nm_frame_layout() {
    printf("\ntest_nm_frame_layout\n");
    using namespace vwegolf_nm;
    Sender s(0x7D);
    uint8_t d[8];
    CHECK(s.Id() == 0x1B00007D, "NM id = 0x1B000000 | node");

    s.Start(REASON_STATUS, 1000);
    s.Frame(1000, d);
    CHECK(same(d, {0x7D, 0x00, 0x41, 0x87, 0x44, 0x00, 0x00, 0x00}), "status wake: 7D 00 41 87 44 00 00 00");
    s.Frame(1000 + 3799, d);
    CHECK(d[2] == 0x41, "byte 2 stays 41 (repeat message) for 3.8 s");
    s.Frame(1000 + 3800, d);
    CHECK(same(d, {0x7D, 0x00, 0x44, 0x87, 0x44, 0x00, 0x00, 0x00}), "after 3.8 s: 7D 00 44 87 44 (normal operation)");
    s.Release(1000 + 9000, d);
    CHECK(same(d, {0x7D, 0x00, 0x04, 0x87, 0x04, 0x00, 0x00, 0x00}), "release: wake-up bit and job bit cleared");
    s.Release(1000 + 500, d);
    CHECK(d[2] == 0x01 && d[4] == 0x04, "early release keeps the repeat-message state (01)");

    s.Start(REASON_STATUS, 0xFFFFF000u);  // clock wraps during the request
    s.Frame(0xFFFFF000u + 3000, d);
    CHECK(d[2] == 0x41, "clock wrap: still repeat message at +3 s");
    s.Frame(0xFFFFF000u + 4000, d);
    CHECK(d[2] == 0x44, "clock wrap: normal operation at +4 s");

    struct { Reason r; uint8_t b4; const char* name; } t[] = {
        {REASON_START_CHARGE, 0x0C, "start charge 80 / 0C"},  {REASON_STOP_CHARGE, 0x0C, "stop charge 81 / 0C"},
        {REASON_TIMERS, 0x14, "timers 82 / 14"},              {REASON_PROFILES, 0x14, "profiles 83 / 14"},
        {REASON_SETTINGS, 0x14, "settings 84 / 14"},          {REASON_START_CLIMATE, 0x24, "start climate 85 / 24"},
        {REASON_STOP_CLIMATE, 0x24, "stop climate 86 / 24"},  {REASON_STATUS, 0x44, "status 87 / 44"},
        {REASON_START_HEATING, 0x24, "window heating on 88 / 24"},
        {REASON_STOP_HEATING, 0x24, "window heating off 89 / 24"},
    };
    for (auto& c : t) {
        s.Start(c.r, 0);
        s.Frame(0, d);
        CHECK(d[3] == (uint8_t)c.r && d[4] == c.b4, c.name);
    }
}

// ---------------------------------------------------------------------------
// Battery-control commands: wake reason per command, release frame
// ---------------------------------------------------------------------------

void test_batctrl_nm_reasons() {
    printf("\ntest_batctrl_nm_reasons\n");
    struct Case {
        const char* name;
        std::function<bool(OvmsVehicleVWeGolf*)> cmd;
        uint8_t b3, b4;
    } cases[] = {
        {"climate on -> 85 / 24", [](OvmsVehicleVWeGolf* v) { return v->CommandClimateControl(true) == Success; }, 0x85, 0x24},
        {"climate off -> 86 / 24", [](OvmsVehicleVWeGolf* v) { return v->CommandClimateControl(false) == Success; }, 0x86, 0x24},
        {"charge start -> 80 / 0C", [](OvmsVehicleVWeGolf* v) { return v->CommandStartCharge() == Success; }, 0x80, 0x0C},
        {"charge stop -> 81 / 0C", [](OvmsVehicleVWeGolf* v) { return v->CommandStopCharge() == Success; }, 0x81, 0x0C},
        {"charge current -> 84 / 14", [](OvmsVehicleVWeGolf* v) { return v->CommandSetChargeCurrent(16) == Success; }, 0x84, 0x14},
        {"profile list -> 83 / 14", [](OvmsVehicleVWeGolf* v) { return v->test_batctrl().ListProfiles(); }, 0x83, 0x14},
        {"timer list -> 82 / 14", [](OvmsVehicleVWeGolf* v) { return v->test_batctrl().ListTimers(); }, 0x82, 0x14},
        {"window heating on -> 88 / 24", [](OvmsVehicleVWeGolf* v) { return v->test_batctrl().WindowHeat(true, kWindows); }, 0x88, 0x24},
        {"window heating off -> 89 / 24", [](OvmsVehicleVWeGolf* v) { return v->test_batctrl().WindowHeat(false, kWindows); }, 0x89, 0x24},
    };
    for (auto& c : cases) {
        auto* v = make_vehicle();
        bool ok = c.cmd(v);
        auto nm = nm_frames(v->m_can3);
        CHECK(ok && nm.size() == 2 && nm[0].data[0] == 0x7D && nm[0].data[1] == 0x00 && nm[0].data[2] == 0x41 &&
                  nm[0].data[3] == c.b3 && nm[0].data[4] == c.b4 && memcmp(nm[0].data, nm[1].data, 8) == 0,
              c.name);
        delete v;
    }
}

void test_batctrl_nm_release() {
    printf("\ntest_batctrl_nm_release\n");
    // Bus awake, the BCU never answers: the command gives up after its 20 s hold and withdraws the
    // wake request with one release frame, then goes quiet.
    auto* v = make_vehicle();
    v->CommandClimateControl(true);
    for (int i = 0; i < VWEGOLF_BATCTRL_WAKE_SECS; i++) tick(v);
    auto nm = nm_frames(v->m_can3);
    CHECK(!nm.empty() && same(nm.back().data, {0x7D, 0x00, 0x04, 0x85, 0x04, 0x00, 0x00, 0x00}),
          "last NM frame = release 7D 00 04 85 04");
    size_t n = nm.size();
    tick(v);
    CHECK(nm_frames(v->m_can3).size() == n, "nothing after the release");
    delete v;

    // Bus asleep the whole time: no release frame (it would only poke a sleeping bus).
    v = make_vehicle();
    v->CommandClimateControl(true);
    for (int i = 0; i < VWEGOLF_BATCTRL_WAKE_SECS + 1; i++) tick(v, /*keep_alive=*/false);
    bool any_release = false;
    for (auto& r : nm_frames(v->m_can3)) if (!(r.data[2] & 0x40)) any_release = true;
    CHECK(!any_release, "no release frame while the bus is asleep");
    delete v;
}

// ---------------------------------------------------------------------------
// `wakeup`
// ---------------------------------------------------------------------------

void test_wakeup_session() {
    printf("\ntest_wakeup_session\n");
    auto* v = make_vehicle();
    auto* bus = v->m_can3;

    CHECK(wake_answered(v) == Success, "wakeup on a sleeping bus: Success once the BCU answers");
    CHECK(bus->reset_count == 1, "KCAN controller restarted once");
    auto nm = nm_frames(bus);
    CHECK(bus->tx_log.size() == 2 && nm.size() == 2, "two wake frames, nothing else");
    CHECK(nm.size() == 2 && same(nm[0].data, {0x7D, 0x00, 0x41, 0x87, 0x44, 0x00, 0x00, 0x00}) &&
              same(nm[1].data, {0x7D, 0x00, 0x41, 0x87, 0x44, 0x00, 0x00, 0x00}),
          "both = 7D 00 41 87 44 00 00 00 (spare node, status wake)");
    bool old_ids = false;
    for (auto& r : bus->tx_log) if (r.id == 0x1B000067 || r.id == 0x17330301) old_ids = true;
    CHECK(!old_ids, "no OCU node 0x67, no 0x17330301 frame");

    bus->tx_log.clear();
    rx(v, 0x131, {0, 0, 0, 0x64, 0, 0, 0, 0});
    CHECK(bus->tx_log.empty(), "received frames trigger no TX (0x5A7 only from Ticker1)");

    // Seconds 1-9: one NM and one 0x5A7 per second.
    size_t nm_n = 0, f5a7_n = 0;
    bool all_zero = true, b2_early = false, b2_late = false;
    for (int s = 1; s <= 9; s++) {
        bus->tx_log.clear();
        tick(v);
        auto n = nm_frames(bus);
        auto h = frames_5a7(bus);
        nm_n += n.size();
        f5a7_n += h.size();
        for (auto& r : h) for (int i = 0; i < 8; i++) if (r.data[i]) all_zero = false;
        if (s == 1 && n.size() == 1) b2_early = n[0].data[2] == 0x41;
        if (s == 5 && n.size() == 1) b2_late = n[0].data[2] == 0x44;
    }
    CHECK(nm_n == 9 && f5a7_n == 9, "seconds 1-9: exactly one NM and one 0x5A7 per second");
    CHECK(all_zero, "0x5A7 all zero (no request pending)");
    CHECK(b2_early && b2_late, "byte 2: 41 in the first 3.8 s, then 44");

    // Second 10: hold over -> release frame, no 0x5A7.
    bus->tx_log.clear();
    tick(v);
    nm = nm_frames(bus);
    CHECK(nm.size() == 1 && same(nm[0].data, {0x7D, 0x00, 0x04, 0x87, 0x04, 0x00, 0x00, 0x00}),
          "after 10 s: release frame 7D 00 04 87 04");
    CHECK(frames_5a7(bus).empty(), "0x5A7 stops with the NM");

    bus->tx_log.clear();
    tick(v);
    tick(v);
    CHECK(bus->tx_log.empty(), "session over: nothing sent");
    delete v;
}

void test_wakeup_lock_request_rides_5a7() {
    printf("\ntest_wakeup_lock_request_rides_5a7\n");
    // Unchanged behaviour: a pending lock request goes out once in the next 0x5A7 frame.
    auto* v = make_vehicle();
    wake_answered(v);
    v->CommandLock("1234");
    v->m_can3->tx_log.clear();
    tick(v);
    auto h = frames_5a7(v->m_can3);
    CHECK(h.size() == 1 && h[0].data[6] == 0x02, "lock flag in byte 6 of the next 0x5A7");
    v->m_can3->tx_log.clear();
    tick(v);
    h = frames_5a7(v->m_can3);
    CHECK(h.size() == 1 && h[0].data[6] == 0x00, "sent once, then all zero again");
    delete v;
}

void test_wakeup_no_answer() {
    printf("\ntest_wakeup_no_answer\n");
    auto* v = make_vehicle();
    g_delay_hook = [](TickType_t t) { g_tick_count += t; };
    CHECK(v->CommandWakeup() == Fail, "no BCU answer within 3 s: Fail");
    g_delay_hook = nullptr;
    CHECK(nm_frames(v->m_can3).size() == 2 && v->m_can3->tx_log.size() == 2, "only the two wake frames");
    v->m_can3->tx_log.clear();
    tick(v);
    CHECK(v->m_can3->tx_log.empty(), "session ended: no NM / 0x5A7 afterwards");
    delete v;

    // Other nodes waking without the BCU is not enough.
    v = make_vehicle();
    g_delay_hook = [&](TickType_t t) { g_tick_count += t; rx(v, 0x1B000010, {0x10, 0, 1, 1, 4, 0, 0, 0}); };
    CHECK(v->CommandWakeup() == Fail, "comfort nodes awake, BCU silent: Fail");
    g_delay_hook = nullptr;
    delete v;
}

void test_wakeup_reset_fails() {
    printf("\ntest_wakeup_reset_fails\n");
    auto* v = make_vehicle();
    v->m_can3->reset_result = ESP_FAIL;
    CHECK(v->CommandWakeup() == Fail, "controller restart failed: Fail");
    CHECK(v->m_can3->tx_log.empty(), "nothing sent");
    delete v;
}

void test_wakeup_bus_already_awake() {
    printf("\ntest_wakeup_bus_already_awake\n");
    auto* v = make_vehicle();
    rx(v, 0x131, {0, 0, 0, 0x64, 0, 0, 0, 0});
    CHECK(v->CommandWakeup() == Success, "bus already active: Success");
    CHECK(v->m_can3->reset_count == 0 && v->m_can3->tx_log.empty(), "no restart, nothing sent");
    tick(v);
    CHECK(v->m_can3->tx_log.empty(), "no session started");
    delete v;
}

void test_wakeup_oem_ocu() {
    printf("\ntest_wakeup_oem_ocu\n");
    auto check = [](const char* name, std::function<void(OvmsVehicleVWeGolf*)> ocu_frame) {
        auto* v = make_vehicle();
        wake_answered(v);
        tick(v);
        CHECK(frames_5a7(v->m_can3).size() == 1, "before: 0x5A7 sent");
        ocu_frame(v);
        v->m_can3->tx_log.clear();
        tick(v);
        char msg[96];
        snprintf(msg, sizeof(msg), "%s: no 0x5A7, NM continues", name);
        CHECK(frames_5a7(v->m_can3).empty() && nm_frames(v->m_can3).size() == 1, msg);
        delete v;
    };
    check("OCU KN frame 0x17F00067",
          [](OvmsVehicleVWeGolf* v) { rx(v, 0x17F00067, {0x20, 0x67, 0, 0, 0, 0, 0, 0}); });
    check("OCU NM frame 0x1B000067",
          [](OvmsVehicleVWeGolf* v) { rx(v, 0x1B000067, {0x67, 0x00, 0x01, 0x01, 0x04, 0, 0, 0}); });
    check("non-zero 0x5A7", [](OvmsVehicleVWeGolf* v) { rx(v, 0x5A7, {0, 0, 0, 0, 0, 0, 0x08, 0}); });
    // An all-zero 0x5A7 from elsewhere does not count (unchanged).
    auto* v = make_vehicle();
    wake_answered(v);
    rx(v, 0x5A7, {0, 0, 0, 0, 0, 0, 0, 0});
    v->m_can3->tx_log.clear();
    tick(v);
    CHECK(frames_5a7(v->m_can3).size() == 1, "all-zero 0x5A7 received: OVMS keeps sending");
    delete v;
}

void test_wakeup_defers_to_batctrl() {
    printf("\ntest_wakeup_defers_to_batctrl\n");
    // A battery-control command during a `wakeup` session sends its own NM; the session sends none,
    // and no release while the command still holds the wake.
    auto* v = make_vehicle();
    wake_answered(v);
    v->CommandClimateControl(true);
    v->m_can3->tx_log.clear();
    for (int s = 1; s <= 11; s++) tick(v);
    size_t status_frames = 0, climate_frames = 0;
    for (auto& r : nm_frames(v->m_can3)) {
        if (r.data[3] == 0x87) status_frames++;
        if (r.data[3] == 0x85) climate_frames++;
    }
    CHECK(status_frames == 0, "no status-wake NM (or release) while the climate command holds the wake");
    CHECK(climate_frames == 11, "the climate command's NM every second");
    delete v;
}

void test_wake_all() {
    test_nm_frame_layout();
    test_batctrl_nm_reasons();
    test_batctrl_nm_release();
    test_wakeup_session();
    test_wakeup_lock_request_rides_5a7();
    test_wakeup_no_answer();
    test_wakeup_reset_fails();
    test_wakeup_bus_already_awake();
    test_wakeup_oem_ocu();
    test_wakeup_defers_to_batctrl();
}
