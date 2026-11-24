// test_bap.cpp — BAP segmentation transport: bap::AssemblerT reassembly rules.
//
// One message per group, a new start replaces it, a skipped continuation index or a short frame
// drops it, more than 500 ms between frames drops it, a BAP Reset drops everything, 12-bit length.
// The frames are those of the e-Golf BCU (0x17332510) where the case occurs on the car.

#include "mock/mock_ovms.hpp"
#include "../src/bap/bap.h"

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

typedef std::vector<uint8_t> Bytes;

struct Got {
    uint8_t op, fct;
    Bytes body;
};

// Feed frames at time t (ms), stepping t by `stepMs` per frame; collect every completed element.
template <class A>
static std::vector<Got> feed_all(A& a, const std::vector<Bytes>& frames, uint32_t& t, uint32_t stepMs = 50) {
    std::vector<Got> out;
    for (const Bytes& f : frames) {
        bap::Element el;
        if (a.feed(f.data(), (uint8_t)f.size(), el, t))
            out.push_back({el.opcode, el.func, Bytes(el.body, el.body + el.bodyLen)});
        t += stepMs;
    }
    return out;
}

// Frame a body the way the BCU does (bap::sendElement), on `group`.
static std::vector<Bytes> frame_of(uint8_t op, uint8_t fct, const Bytes& body, uint8_t group = 0) {
    std::vector<Bytes> frames;
    auto sink = [&](const uint8_t* d, uint8_t dlc) -> bool {
        frames.push_back(Bytes(d, d + dlc));
        return true;
    };
    bap::sendElement(sink, (bap::Opcode)op, 0x25, fct, body.data(), (uint16_t)body.size(), group);
    return frames;
}

// ClimateState heartbeat (op 3, fct 0x12, idle): start + one continuation [WIRE].
static const Bytes kCsStart = {0x80, 0x07, 0x39, 0x52, 0x00, 0x00, 0x00, 0x0a};
static const Bytes kCsCont  = {0xc0, 0x00, 0xbb, 0x00};
static const Bytes kCsBody  = {0x00, 0x00, 0x00, 0x0a, 0x00, 0xbb, 0x00};
// BCU BAP Reset (op 0 on BAP-Config), its first frame after it starts [WIRE].
static const Bytes kReset   = {0x09, 0x42, 0x03, 0x00, 0x25, 0x00, 0x04, 0x00};

// Profile list (status, fct 0x19) of 119 bytes, ending in "end": 1 start + 17 continuations, whose
// index runs c0..cf and wraps to c0 for the last one (c0 65 6e 64) [WIRE: 2026-11-22 10:07].
static Bytes profile_list_body() {
    Bytes b(119);
    for (size_t i = 0; i < b.size(); i++) b[i] = (uint8_t)(0x20 + i % 64);
    b[116] = 'e'; b[117] = 'n'; b[118] = 'd';
    return b;
}

static void test_bap_short_and_long() {
    printf("\ntest_bap_short_and_long\n");
    bap::AssemblerT<224, 4> a;
    uint32_t t = 0;
    auto got = feed_all(a, {{0x49, 0x58, 0x01}, kCsStart, kCsCont}, t);
    CHECK(got.size() == 2, "short frame + 2-frame message: 2 elements");
    CHECK(got.size() == 2 && got[0].op == 4 && got[0].fct == 0x18 && got[0].body == Bytes{0x01},
          "short frame 49 58 01 -> status, fct 0x18, body 01");
    CHECK(got.size() == 2 && got[1].op == 3 && got[1].fct == 0x12 && got[1].body == kCsBody,
          "ClimateState heartbeat reassembled");

    const Bytes body = profile_list_body();
    auto frames = frame_of(4, 0x19, body);
    CHECK(frames.size() == 18 && frames.back() == (Bytes{0xc0, 'e', 'n', 'd'}),
          "119-byte message: 18 frames, the last continuation index wraps to c0");
    got = feed_all(a, frames, t);
    CHECK(got.size() == 1 && got[0].fct == 0x19 && got[0].body == body, "18-frame message across the index wrap");
}

static void test_bap_groups() {
    printf("\ntest_bap_groups\n");
    bap::AssemblerT<224, 4> a;
    uint32_t t = 0;
    const Bytes b0 = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};  // start + 1 continuation
    const Bytes b1 = {21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38};
    auto f0 = frame_of(4, 0x11, b0, 0);
    auto f1 = frame_of(4, 0x12, b1, 1);  // start + 2 continuations
    auto got = feed_all(a, {f0[0], f1[0], f1[1], f0[1], f1[2]}, t);
    CHECK(got.size() == 2 && got[0].fct == 0x11 && got[0].body == b0 && got[1].fct == 0x12 && got[1].body == b1,
          "messages interleaved on groups 0 and 1 both complete");

    // A new start on a group replaces the message in progress there.
    auto old = frame_of(4, 0x11, b0, 0);
    got = feed_all(a, {old[0], kCsStart, kCsCont}, t);
    CHECK(got.size() == 1 && got[0].fct == 0x12 && got[0].body == kCsBody, "a start replaces its group's message");
    CHECK(a.stats().replaced == 1, "replaced message counted");
    got = feed_all(a, {old[1]}, t);
    CHECK(got.empty() && a.stats().droppedOrphans == 1, "the replaced message's continuation is an orphan");
}

static void test_bap_lost_frames() {
    printf("\ntest_bap_lost_frames\n");
    bap::AssemblerT<224, 4> a;
    uint32_t t = 0;
    const Bytes body = profile_list_body();
    auto f = frame_of(4, 0x19, body);

    // Continuation c1 lost: the message is dropped at c2, the rest are orphans.
    std::vector<Bytes> lost(f);
    lost.erase(lost.begin() + 2);
    auto got = feed_all(a, lost, t);
    CHECK(got.empty(), "a skipped continuation index drops the message");
    CHECK(a.stats().sequenceDrops == 1 && a.stats().droppedOrphans == 14, "1 sequence drop, 14 orphans");

    // A middle continuation cut short drops the message; a padded last one is fine.
    std::vector<Bytes> cut(f);
    cut[3].resize(5);
    got = feed_all(a, cut, t);
    CHECK(got.empty() && a.stats().truncatedDrops == 1, "a short middle continuation drops the message");
    std::vector<Bytes> padded = frame_of(3, 0x12, kCsBody);
    padded.back().resize(8, 0x55);
    got = feed_all(a, padded, t);
    CHECK(got.size() == 1 && got[0].body == kCsBody, "padding after the last body byte is ignored");
    Bytes shortStart = kCsStart;
    shortStart.resize(6);
    got = feed_all(a, {shortStart, kCsCont}, t);
    CHECK(got.empty() && a.stats().truncatedDrops == 2, "a start frame missing body bytes drops the message");
}

static void test_bap_timeout() {
    printf("\ntest_bap_timeout\n");
    bap::AssemblerT<224, 4> a;
    bap::Element el;

    CHECK(!a.feed(kCsStart.data(), 8, el, 1000), "start");
    CHECK(a.feed(kCsCont.data(), 4, el, 1500), "continuation 500 ms later: complete");

    CHECK(!a.feed(kCsStart.data(), 8, el, 2000), "start");
    CHECK(!a.feed(kCsCont.data(), 4, el, 2510) && a.stats().timeouts == 1, "continuation 510 ms later: dropped");

    // The timeout runs from the last frame, not the start: 18 frames 400 ms apart still complete.
    uint32_t t = 3000;
    auto got = feed_all(a, frame_of(4, 0x19, profile_list_body()), t, 400);
    CHECK(got.size() == 1 && got[0].body.size() == 119, "7 s message with 400 ms gaps completes");

    // Clock wrap.
    CHECK(!a.feed(kCsStart.data(), 8, el, 0xFFFFFF00u), "start just before the clock wraps");
    CHECK(a.feed(kCsCont.data(), 4, el, 0x00000010u), "continuation after the wrap (272 ms): complete");
}

static void test_bap_reset() {
    printf("\ntest_bap_reset\n");
    bap::AssemblerT<224, 4> a;
    uint32_t t = 0;
    auto f = frame_of(4, 0x12, {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12}, 1);
    auto got = feed_all(a, {kCsStart, f[0], kReset, kCsCont, f[1]}, t);
    CHECK(got.size() == 1 && got[0].op == bap::OP_RESET && got[0].fct == bap::kFuncBapConfig,
          "BAP Reset is passed on");
    CHECK(a.stats().droppedOrphans == 2, "BAP Reset drops the messages in progress on every group");
}

static void test_bap_long_length() {
    printf("\ntest_bap_long_length\n");
    // 300-byte body: start 81 2c (12-bit length 0x12c) + 43 continuations.
    Bytes body(300);
    for (size_t i = 0; i < body.size(); i++) body[i] = (uint8_t)(i * 7);
    std::vector<Bytes> frames;
    frames.push_back({0x81, 0x2c, 0x49, 0x59, body[0], body[1], body[2], body[3]});
    uint8_t idx = 0;
    for (size_t off = 4; off < body.size(); off += 7, idx = (idx + 1) & 0xF) {
        Bytes c = {(uint8_t)(0xC0 | idx)};
        for (size_t i = off; i < off + 7 && i < body.size(); i++) c.push_back(body[i]);
        frames.push_back(c);
    }
    CHECK(bap::segLength(0x81, 0x2c) == 300, "length bits 8-11 come from the low nibble of byte 0");

    bap::AssemblerT<360, 4> big;
    uint32_t t = 0;
    auto got = feed_all(big, frames, t);
    CHECK(got.size() == 1 && got[0].body == body, "300-byte message reassembled");

    bap::AssemblerT<224, 4> small;
    std::vector<Bytes> seq = {kCsStart};
    seq.insert(seq.end(), frames.begin(), frames.end());
    seq.push_back(kCsCont);
    got = feed_all(small, seq, t);
    CHECK(got.empty() && small.stats().overflowDrops == 1, "too long for MaxBody: dropped, not truncated");
    CHECK(small.stats().replaced == 1, "the oversize start still clears its group");
}

// The false alert of 2026-11-22 10:07 ("Window/seat heating not started: unknown (14)"): a
// ClimateState heartbeat lost its continuation, then the profile list lost one continuation, and the
// old reassembler completed the stale ClimateState start with the list's last frame (c0 65 6e 64)
// as 00 00 00 0a 65 6e 64.
static void test_bap_stitch_1007() {
    printf("\ntest_bap_stitch_1007\n");
    auto list = frame_of(4, 0x19, profile_list_body());
    std::vector<Bytes> seq = {kCsStart};  // continuation lost
    seq.insert(seq.end(), list.begin(), list.end());
    seq.erase(seq.begin() + 1 + 5);       // one profile-list continuation lost

    bap::AssemblerT<224, 4> a;
    uint32_t t = 0;
    auto got = feed_all(a, seq, t);
    CHECK(got.empty(), "10:07 sequence: no element (no stitched ClimateState, no damaged list)");

    // A stale start (its continuation lost) is not completed by a continuation seconds later.
    bap::AssemblerT<224, 4> c;
    bap::Element el;
    c.feed(kCsStart.data(), 8, el, 0);
    const Bytes tail = {0xc0, 0x65, 0x6e, 0x64};
    CHECK(!c.feed(tail.data(), 4, el, 3000) && c.stats().timeouts == 1,
          "continuation 3 s after a stale start: dropped");
}

static void test_bap_eviction() {
    printf("\ntest_bap_eviction\n");
    bap::AssemblerT<64, 2> a;  // 2 slots for 4 groups
    uint32_t t = 0;
    const Bytes b = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    auto g0 = frame_of(4, 0x10, b, 0), g1 = frame_of(4, 0x11, b, 1), g2 = frame_of(4, 0x12, b, 2);
    auto got = feed_all(a, {g0[0], g1[0], g2[0], g1[1], g2[1], g0[1]}, t);
    CHECK(got.size() == 2 && got[0].fct == 0x11 && got[1].fct == 0x12, "2 slots: groups 1 and 2 complete");
    CHECK(a.stats().evictions == 1 && a.stats().droppedOrphans == 1, "group 0 (least recent) evicted");
}

void test_bap_all() {
    test_bap_short_and_long();
    test_bap_groups();
    test_bap_lost_frames();
    test_bap_timeout();
    test_bap_reset();
    test_bap_long_length();
    test_bap_stitch_1007();
    test_bap_eviction();
}
