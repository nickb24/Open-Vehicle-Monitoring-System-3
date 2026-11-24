/*
;    Project:       BAP protocol library
;    Subproject:    Generic BAP (Bedien- und Anzeigeprotokoll) transport + framing
;
;    (C) 2026  Jona Wagner <jona@jonawagner.me>
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
; IMPLIED. See the accompanying LICENSE / SPDX-License-Identifier: MIT.
*/

// Generic BAP protocol codec: application element header + segmentation
// transport. This layer knows nothing about any specific vehicle, logical
// device (LSG) or function -- it only frames and reassembles telegrams. The
// vehicle-specific semantics (LSG 0x25 battery control, etc.) live one layer
// up in vehicle/.
//
// Portability: pure C++11, only <cstdint>/<cstddef>/<cstring>. No exceptions,
// no RTTI, no dynamic allocation. Intended to drop into OVMS as-is, but builds
// and tests on a host with no ESP-IDF/Arduino dependency.

#ifndef BAP_BAP_H_
#define BAP_BAP_H_

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace bap {

// ---------------------------------------------------------------------------
// Element header (2 bytes)
// ---------------------------------------------------------------------------
// Once segmentation transport is stripped, every BAP telegram begins with a
// 2-byte application header:
//
//     byte0 = (opcode << 4) | (func12 >> 8)
//     byte1 =  func12 & 0xFF
//     func12 = (lsg << 6) | func            // 12-bit "function id"
//
//   opcode = byte0 >> 4                      // operation nibble
//   lsg    = func12 >> 6                     // 6-bit logical device id
//   func   = func12 & 0x3F                   // 6-bit function/property id
//
// Worked example: op=SET_GET(2), lsg=0x25, func=0x18
//   func12 = (0x25<<6)|0x18 = 0x958
//   byte0  = (2<<4)|(0x958>>8) = 0x29 ; byte1 = 0x58  ->  "29 58"
//
// This model is corroborated by two independent reverse-engineering sources
// (an OCU firmware and an offline capture decoder), each cross-checked against
// real comfort-bus captures.

// BAP operation codes carried in the header's top nibble. These are the values
// observed on the wire (agreed by both RE sources); they are NOT the higher
// DSI-RPC opcode enumeration, which is a different layer.
enum Opcode : uint8_t {
  OP_RESET      = 0,  // sent by a function ECU when its BAP layer (re)starts (with func 2)
  OP_GET        = 1,  // read current value
  OP_SET_GET    = 2,  // set value (command); function ECU replies with a status
  OP_HEARTBEAT  = 3,  // periodic status
  OP_STATUS     = 4,  // status broadcast / reply to GET/SET_GET
  OP_STATUS_ACK = 5,
  OP_ACK        = 6,
  OP_ERROR      = 7,  // command failed
};

// Function 2 is BAP-Config in every LSG. A Reset (OP_RESET) on it means the
// sender's BAP layer restarted.
constexpr uint8_t kFuncBapConfig = 0x02;

// A fully-decoded BAP element: application header fields plus a VIEW of the
// body -- `body` is never owned by the Element. Its lifetime depends on which
// producer filled it:
//   * decodeElement(): `body` aliases the CALLER's buffer.
//   * Assembler::feed(): `body` aliases Assembler-internal storage that is
//     valid only until the NEXT feed() call on that Assembler.
// Consume it before the next feed(), or copyBody() it if you must retain it.
// One Assembler is single-task/single-consumer; do not hand `body` to another
// task without copying first.
struct Element {
  uint8_t opcode = 0;
  uint8_t lsg = 0;     // logical device id
  uint8_t func = 0;    // function / property id within the LSG
  uint16_t bodyLen = 0;
  const uint8_t* body = nullptr;
};

// Copy an Element's body into caller storage (for retaining it past the next
// feed()). Returns the number of bytes copied (min of bodyLen and cap).
inline uint16_t copyBody(const Element& e, uint8_t* out, uint16_t cap) {
  uint16_t n = (e.bodyLen < cap) ? e.bodyLen : cap;
  if (e.body && n) memcpy(out, e.body, n);
  return n;
}

// Encode the 2-byte element header for (opcode, lsg, func).
void encodeHeader(uint8_t out[2], uint8_t opcode, uint8_t lsg, uint8_t func);

// Decode a 2-byte element header into its fields.
void decodeHeader(const uint8_t in[2], uint8_t& opcode, uint8_t& lsg, uint8_t& func);

// Decode a fully-reassembled element (2-byte header + body). `elem`/`len` must
// cover the whole telegram with segmentation already removed. Returns false
// (and leaves `out` untouched) if len < 2.
bool decodeElement(const uint8_t* elem, uint16_t len, Element& out);

// ---------------------------------------------------------------------------
// Segmentation transport
// ---------------------------------------------------------------------------
// A telegram whose body fits in one CAN frame is sent "short": the element
// header sits at byte 0 and the body follows. A larger telegram is split with
// a leading transport control byte:
//
//   short frame  : byte0 bit7 == 0  -> byte0/byte1 are the element header
//   start frame  : byte0 = 0x80 | (group<<4) | (length>>8) ; byte1 = length & 0xFF
//                  byte2/byte3 = element header  ; byte4.. = first body bytes
//   continuation : byte0 = 0xC0 | (group<<4) | index ; byte1.. = body bytes
//
// `group` (0..3) allows up to four interleaved long messages; `index` is a
// per-message continuation counter (start implies 0, first continuation is 0)
// that wraps from 15 to 0.
//
// The length counts the BODY only (element length minus the 2-byte header).
// Verified against a real capture: an 8-byte body / 10-byte element is
// announced as 0x08. (One RE source's *encoder* wrote element length here;
// that is a bug relative to the wire and is deliberately not reproduced.)
// The length is 12 bits wide: the low nibble of byte0 holds bits 8-11 (OCU
// firmware; 0xFFF is reserved). It is 0 for every message the e-Golf BCU has
// sent, but other devices send longer messages (the OCU firmware has 360-byte
// buffers for the instrument cluster).

inline bool isSegStart(uint8_t b) { return (b >> 4) >= 0x8 && (b >> 4) <= 0xB; }
inline bool isSegCont(uint8_t b)  { return (b >> 4) >= 0xC; }
inline bool isShortFrame(uint8_t b) { return (b & 0x80) == 0; }
inline uint8_t segGroup(uint8_t b) { return (b >> 4) & 0x3; }
inline uint8_t segIndex(uint8_t b) { return b & 0xF; }
// Body length announced by a start frame (bytes 0 and 1).
inline uint16_t segLength(uint8_t b0, uint8_t b1) { return (uint16_t)(((b0 & 0x0F) << 8) | b1); }

// Little-endian 16-bit helpers (BAP multi-byte scalars are little-endian).
inline void putLe16(uint8_t* p, uint16_t v) { p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; }
inline uint16_t getLe16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }

// ---------------------------------------------------------------------------
// MQB CAN-id scheme
// ---------------------------------------------------------------------------
// MQB BAP telegrams use 29-bit extended ids that embed the logical device:
//     base(16) | lsg(8) | subsystem(8)
// subsystem 0x10 = the function ECU's broadcast/status side (FSG); the
// command/display side (ASG) low byte is device-specific (0x00 or 0x01). Derive
// ids from these rather than hardcoding them, and recover the sender by decoding
// an incoming id's subsystem byte.
constexpr uint16_t kMqbBase      = 0x1733;
constexpr uint8_t  kSubsystemFsg = 0x10;

constexpr uint32_t mqbCanId(uint8_t lsg, uint8_t subsystem, uint16_t base = kMqbBase) {
  return ((uint32_t)base << 16) | ((uint32_t)lsg << 8) | subsystem;
}
struct CanId { uint16_t base; uint8_t lsg; uint8_t subsystem; };
inline CanId decodeCanId(uint32_t id) {
  CanId c;
  c.base = (uint16_t)((id >> 16) & 0xFFFF);
  c.lsg = (uint8_t)((id >> 8) & 0xFF);
  c.subsystem = (uint8_t)(id & 0xFF);
  return c;
}
inline bool isFromFsg(uint32_t id) { return (id & 0xFF) == kSubsystemFsg; }

// ---------------------------------------------------------------------------
// Body reader / writer
// ---------------------------------------------------------------------------
// Cursor codecs for BAP element bodies, so field decoders/encoders read and
// write in sequence rather than hand-computing offsets. Every access is
// bounds-checked; the first access that would run past the buffer latches an
// error (ok() == false) and further scalar reads return 0. Multi-byte scalars
// are little-endian (BAP convention).
class Reader {
 public:
  Reader(const uint8_t* data, uint16_t len) : d_(data), n_(len) {}
  bool ok() const { return ok_; }
  uint16_t pos() const { return pos_; }
  uint16_t remaining() const { return (pos_ < n_) ? (uint16_t)(n_ - pos_) : 0; }

  uint8_t u8() {
    if ((uint16_t)(pos_ + 1) > n_) { ok_ = false; return 0; }
    return d_[pos_++];
  }
  uint16_t u16le() {
    if ((uint16_t)(pos_ + 2) > n_) { ok_ = false; return 0; }
    uint16_t v = (uint16_t)(d_[pos_] | (d_[pos_ + 1] << 8));
    pos_ = (uint16_t)(pos_ + 2);
    return v;
  }
  void skip(uint16_t k) {
    if ((uint16_t)(pos_ + k) > n_) { pos_ = n_; ok_ = false; }
    else pos_ = (uint16_t)(pos_ + k);
  }
  // Length-prefixed string (1-byte length). Consumes the full declared length
  // from the source (error if it runs past the end); copies up to cap-1 chars
  // into `out` and NUL-terminates. Returns the number of chars copied. If the
  // declared name was longer than the destination it is truncated (the source
  // still advances by the full length); `*truncated` (when given) reports that,
  // since a truncated copy is otherwise silent data loss.
  uint8_t strLp(char* out, uint8_t cap, bool* truncated = nullptr) {
    if (truncated) *truncated = false;
    uint8_t len = u8();
    if (!ok_) { if (cap) out[0] = 0; return 0; }
    if ((uint16_t)(pos_ + len) > n_) { ok_ = false; if (cap) out[0] = 0; return 0; }
    uint8_t copy = (cap && len > (uint8_t)(cap - 1)) ? (uint8_t)(cap - 1) : len;
    if (truncated && copy < len) *truncated = true;
    for (uint8_t i = 0; i < copy; i++) out[i] = (char)d_[pos_ + i];
    if (cap) out[copy] = 0;
    pos_ = (uint16_t)(pos_ + len);
    return copy;
  }

 private:
  const uint8_t* d_;
  uint16_t n_;
  uint16_t pos_ = 0;
  bool ok_ = true;
};

class Writer {
 public:
  Writer(uint8_t* out, size_t cap) : d_(out), cap_(cap) {}
  bool ok() const { return ok_; }
  size_t size() const { return pos_; }

  void u8(uint8_t v) {
    if (pos_ + 1 > cap_) { ok_ = false; return; }
    d_[pos_++] = v;
  }
  void u16le(uint16_t v) {
    if (pos_ + 2 > cap_) { ok_ = false; return; }
    d_[pos_++] = (uint8_t)(v & 0xFF);
    d_[pos_++] = (uint8_t)((v >> 8) & 0xFF);
  }
  // Length-prefixed string (1-byte length) mirroring Reader::strLp.
  void strLp(const char* s, uint8_t len) {
    u8(len);
    if (pos_ + len > cap_) { ok_ = false; return; }
    for (uint8_t i = 0; i < len; i++) d_[pos_++] = (uint8_t)s[i];
  }

 private:
  uint8_t* d_;
  size_t cap_;
  size_t pos_ = 0;
  bool ok_ = true;
};

// ---------------------------------------------------------------------------
// Sending
// ---------------------------------------------------------------------------
// Outcome of a send. `framesSent` frames reached the sink; `status` says how it
// ended. The important distinction for a live bus is SinkFailedPartial: a start
// (and maybe some continuations) were already written, so a TRUNCATED telegram
// is on the wire and the peer will hold a reassembly slot until it times out.
struct SendResult {
  enum Status : uint8_t {
    Ok = 0,               // whole element sent
    Refused,              // nothing sent (bodyLen > 255, or null body w/ length)
    SinkFailedFirst,      // sink rejected the first frame; nothing on the bus
    SinkFailedPartial,    // sink failed mid-send; a truncated telegram is on the bus
  } status;
  uint16_t framesSent;
  bool ok() const { return status == Ok; }
};

// Number of frames a body of this size will be framed into (0 if it can't be:
// bodyLen > 255). Lets a caller reason about a send without re-deriving framing.
constexpr uint16_t expectedFrames(uint16_t bodyLen) {
  return (bodyLen <= 6) ? 1
       : (bodyLen > 255) ? 0
       : (uint16_t)(1 + ((bodyLen - 4) + 6) / 7);  // start(4 body) + ceil(rest/7)
}

// Non-template framing core (defined in bap.cpp): a SINGLE shared instantiation,
// so the short/long framing logic isn't stamped into flash once per sink type.
// `sink` is a C function pointer plus an opaque context. `group` selects the
// interleave channel (0..3) for long messages.
typedef bool (*FrameSinkFn)(void* ctx, const uint8_t* data, uint8_t dlc);
SendResult sendElementFn(FrameSinkFn sink, void* ctx, uint8_t opcode, uint8_t lsg,
                         uint8_t func, const uint8_t* body, uint16_t bodyLen,
                         uint8_t group = 0);

// Ergonomic wrapper: accepts any callable `bool(const uint8_t*, uint8_t)` (a
// lambda, functor, or function). Only the tiny adapter below is duplicated per
// sink type; the framing routine lives once in sendElementFn.
template <typename Sink>
SendResult sendElement(Sink&& sink, Opcode opcode, uint8_t lsg, uint8_t func,
                       const uint8_t* body, uint16_t bodyLen, uint8_t group = 0) {
  using S = typename std::remove_reference<Sink>::type;
  FrameSinkFn fn = [](void* ctx, const uint8_t* d, uint8_t dlc) -> bool {
    return (*static_cast<S*>(ctx))(d, dlc);
  };
  return sendElementFn(fn, static_cast<void*>(&sink), opcode, lsg, func, body, bodyLen, group);
}

// ---------------------------------------------------------------------------
// Reassembly
// ---------------------------------------------------------------------------
// Feed raw CAN frames (for a SINGLE arbitration id) one at a time, with the
// time each was received in milliseconds (any free-running clock; only
// differences are used, so wrap-around is fine). When a complete element is
// ready, feed() returns true and fills `out`; out.body points into internal
// storage valid until the next feed() (see Element). Short frames complete
// immediately.
//
// Long messages follow the receiver rules of a VW OCU firmware, so a lost frame
// loses only its own message and frames of two messages are never joined:
//   * A sender has at most one long message in flight per group, so each group
//     holds at most one message. A start frame replaces the message in progress
//     on its group (the sender gave it up).
//   * A continuation must carry the next index of its group's message;
//     otherwise a frame was lost and the message is dropped. A continuation
//     with no message in progress is ignored.
//   * Every frame but the last must be full (4 body bytes in the start frame, 7
//     in a continuation); a shorter one drops the message.
//   * More than kTimeoutMs between two frames of a message drops it. Without
//     this, a message whose last frames were lost would wait forever and could
//     later be completed by the frames of another message.
//   * A BAP Reset (OP_RESET on kFuncBapConfig) drops every message in progress.
//
// MaxBody caps a reassembled body; a longer message is dropped (counted in
// overflowDrops). MaxPending is the number of messages held at once: with 4
// (one per group) none is ever evicted; fewer saves RAM, and a start that finds
// every slot busy then evicts the least recently active message.
template <uint16_t MaxBody = 256, uint8_t MaxPending = 4>
class AssemblerT {
  static_assert(MaxPending >= 1 && MaxPending <= 4, "one message per group, 4 groups");

 public:
  static constexpr uint16_t kMaxBody = MaxBody;
  static constexpr uint8_t kMaxPending = MaxPending;
  // Longest gap allowed between two frames of one message (OCU: 50 x 10 ms).
  static constexpr uint32_t kTimeoutMs = 500;

  void reset() {
    for (uint8_t i = 0; i < MaxPending; i++) pending_[i].active = false;
  }

  bool feed(const uint8_t* data, uint8_t dlc, Element& out, uint32_t nowMs) {
    if (dlc < 1) return false;
    if (dlc > 8) { badDlc_++; return false; }  // classic CAN caps a frame at 8 bytes
    const uint8_t ctrl = data[0];

    // Short single-frame element: header at byte 0, body follows.
    if (isShortFrame(ctrl)) {
      if (dlc < 2) return false;
      decodeHeader(data, out.opcode, out.lsg, out.func);
      if (out.opcode == OP_RESET && out.func == kFuncBapConfig) reset();  // the sender restarted
      out.bodyLen = (uint16_t)(dlc - 2);       // <= 6
      if (out.bodyLen) {
        memcpy(shortBuf_, data + 2, out.bodyLen);
        out.body = shortBuf_;
      } else {
        out.body = nullptr;
      }
      return true;
    }

    const uint8_t group = segGroup(ctrl);
    const int cur = slotFor(group);

    // Long start frame: 0x8x len hdr0 hdr1 body[0..3]
    if (isSegStart(ctrl)) {
      if (cur >= 0) { replaced_++; pending_[cur].active = false; }
      if (dlc < 4) { truncatedDrops_++; return false; }
      const uint16_t len = segLength(ctrl, data[1]);
      if (len > MaxBody) { overflowDrops_++; return false; }  // can never complete
      const uint8_t first = (uint8_t)(len < 4 ? len : 4);
      if (dlc - 4 < first) { truncatedDrops_++; return false; }
      Pending& pm = pending_[cur >= 0 ? cur : allocSlot()];
      pm.group = group;
      pm.nextIndex = 0;
      pm.expected = len;
      pm.hdr0 = data[2];
      pm.hdr1 = data[3];
      memcpy(pm.buf, data + 4, first);
      pm.got = first;
      pm.lastMs = nowMs;
      pm.lastSeq = seq_++;
      if (pm.got >= pm.expected) return complete(pm, out);
      pm.active = true;
      return false;
    }

    // Long continuation frame: 0xCx body...
    if (cur < 0) { droppedOrphans_++; return false; }  // no message in progress (start lost)
    Pending& pm = pending_[cur];
    pm.active = false;  // dropped unless this frame checks out
    if ((uint32_t)(nowMs - pm.lastMs) > kTimeoutMs) { timeouts_++; return false; }
    if (segIndex(ctrl) != pm.nextIndex) { sequenceDrops_++; return false; }
    const uint16_t remaining = (uint16_t)(pm.expected - pm.got);  // > 0 while active
    const uint8_t chunk = (uint8_t)(remaining < 7 ? remaining : 7);
    if (dlc - 1 < chunk) { truncatedDrops_++; return false; }
    memcpy(pm.buf + pm.got, data + 1, chunk);
    pm.got = (uint16_t)(pm.got + chunk);
    pm.nextIndex = (uint8_t)((pm.nextIndex + 1) & 0xF);
    pm.lastMs = nowMs;
    pm.lastSeq = seq_++;
    if (pm.got >= pm.expected) return complete(pm, out);
    pm.active = true;
    return false;
  }

  // Monotonic diagnostic counters for the RX task to poll/log. They surface the
  // drops feed()'s bool return can't distinguish from a normal in-progress
  // frame.
  struct Stats {
    uint32_t droppedOrphans;  // continuation with no message in progress (its start was lost)
    uint32_t evictions;       // message evicted by a start (only with MaxPending < 4)
    uint32_t overflowDrops;   // start frame announcing a body longer than MaxBody
    uint32_t badDlc;          // frame longer than 8 bytes
    uint32_t replaced;        // message given up: a new start frame on its group
    uint32_t sequenceDrops;   // message dropped: a continuation index was skipped
    uint32_t truncatedDrops;  // message dropped: a frame shorter than its content
    uint32_t timeouts;        // message dropped: more than kTimeoutMs between frames
  };
  Stats stats() const {
    return {droppedOrphans_, evictions_,     overflowDrops_,  badDlc_,
            replaced_,       sequenceDrops_, truncatedDrops_, timeouts_};
  }

 private:
  struct Pending {
    bool active = false;
    uint8_t group = 0;
    uint8_t nextIndex = 0;
    uint8_t hdr0 = 0;
    uint8_t hdr1 = 0;
    uint16_t expected = 0;
    uint16_t got = 0;
    uint32_t lastMs = 0;      // time of the last frame (inter-frame timeout)
    uint32_t lastSeq = 0;     // eviction order (see allocSlot)
    uint8_t buf[MaxBody];     // also backs the emitted Element for long messages
  };

  // The slot holding the message in progress on `group`, or -1.
  int slotFor(uint8_t group) const {
    for (uint8_t i = 0; i < MaxPending; i++)
      if (pending_[i].active && pending_[i].group == group) return i;
    return -1;
  }

  // A slot for a start on a group with no message in progress: a free one, else
  // (MaxPending < 4 only) the least recently active message is evicted.
  uint8_t allocSlot() {
    for (uint8_t i = 0; i < MaxPending; i++)
      if (!pending_[i].active) return i;
    uint8_t oldest = 0;
    for (uint8_t i = 1; i < MaxPending; i++)
      if ((uint32_t)(seq_ - pending_[i].lastSeq) > (uint32_t)(seq_ - pending_[oldest].lastSeq)) oldest = i;
    evictions_++;
    return oldest;
  }

  bool complete(Pending& pm, Element& out) {
    const uint8_t hdr[2] = {pm.hdr0, pm.hdr1};
    decodeHeader(hdr, out.opcode, out.lsg, out.func);
    out.bodyLen = pm.got;
    // No copy: pm.buf is stable until this (now inactive) slot is reused, which
    // is >= the "valid until next feed()" contract Element promises.
    out.body = pm.got ? pm.buf : nullptr;
    pm.active = false;
    return true;
  }

  Pending pending_[MaxPending];
  uint8_t shortBuf_[6];  // backs short-frame bodies (<= 6 bytes)
  uint32_t seq_ = 0;
  uint32_t droppedOrphans_ = 0;
  uint32_t evictions_ = 0;
  uint32_t overflowDrops_ = 0;
  uint32_t badDlc_ = 0;
  uint32_t replaced_ = 0;
  uint32_t sequenceDrops_ = 0;
  uint32_t truncatedDrops_ = 0;
  uint32_t timeouts_ = 0;
};

using Assembler = AssemblerT<>;

// ---------------------------------------------------------------------------
// AssemblerSet -- one Assembler per CAN arbitration id
// ---------------------------------------------------------------------------
// A BAP bus carries several arbitration ids, and reassembly state must not be
// shared across them (interleaved group-0 long messages from two ECUs would
// corrupt each other). This keeps a fixed set of N (id -> Assembler) slots and
// routes each frame to the right one -- the intended RX entry point. Ids are
// claimed on first sight; once all N are in use an unknown id is dropped
// (counted in unknownDrops()). Size N to the number of BAP ids you consume.
//
// Cost: N * sizeof(AssemblerT<MaxBody,MaxPending>). Tune all three, e.g.
// AssemblerSetT<4, 160, 4>.
template <uint8_t N, uint16_t MaxBody = 256, uint8_t MaxPending = 4>
class AssemblerSetT {
 public:
  // Route one frame by its CAN id (nowMs: receive time, see AssemblerT::feed).
  // Returns true and fills `out` when that id's stream completes an element.
  bool feed(uint32_t canId, const uint8_t* data, uint8_t dlc, Element& out, uint32_t nowMs) {
    for (uint8_t i = 0; i < N; i++)
      if (used_[i] && id_[i] == canId) return asm_[i].feed(data, dlc, out, nowMs);
    for (uint8_t i = 0; i < N; i++)
      if (!used_[i]) { used_[i] = true; id_[i] = canId; return asm_[i].feed(data, dlc, out, nowMs); }
    unknownDrops_++;
    return false;
  }
  void reset() {
    for (uint8_t i = 0; i < N; i++) { used_[i] = false; asm_[i].reset(); }
  }
  uint32_t unknownDrops() const { return unknownDrops_; }

 private:
  AssemblerT<MaxBody, MaxPending> asm_[N];
  uint32_t id_[N] = {0};
  bool used_[N] = {false};
  uint32_t unknownDrops_ = 0;
};

}  // namespace bap

#endif  // BAP_BAP_H_
