/*
;    Project:       BAP protocol library
;    Subproject:    e-Golf BatteryControl (LSG 0x25) semantics (implementation)
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
; IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
; AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
; OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
; THE SOFTWARE.
*/

#include "egolf/battery_control.h"

#include <cstdio>  // snprintf — status value names

namespace bap {
namespace egolf {

namespace {
// One full profile record read straight off a body cursor: fixed fields in
// order, then the length-prefixed name. All bounds live in the Reader.
bool readProfile(bap::Reader& r, Profile& out) {
  out.operation = r.u8();
  out.operation2 = r.u8();
  out.maxCurrent = r.u8();
  out.minChargeLevel = r.u8();
  out.minRange = r.u16le();
  out.targetChargeLevel = r.u8();
  out.targetChargeDuration = r.u8();
  out.targetChargeRange = r.u16le();
  out.unitRange = r.u8();
  out.rangeCalculationSetup = r.u8();
  out.temperatureRaw = r.u8();
  out.temperatureUnit = r.u8();
  out.leadTime = r.u8();
  out.holdingTimePlug = r.u8();
  out.holdingTimeBattery = r.u8();
  out.providerDataId = r.u16le();
  bool truncated = false;
  out.nameLen = r.strLp(out.name, sizeof(out.name), &truncated);
  out.nameTruncated = truncated;
  return r.ok();
}
}  // namespace

size_t encodeProfile(uint8_t* out, size_t cap, const Profile& p) {
  bap::Writer w(out, cap);
  w.u8(p.operation);
  w.u8(p.operation2);
  w.u8(p.maxCurrent);
  w.u8(p.minChargeLevel);
  w.u16le(p.minRange);
  w.u8(p.targetChargeLevel);
  w.u8(p.targetChargeDuration);
  w.u16le(p.targetChargeRange);
  w.u8(p.unitRange);
  w.u8(p.rangeCalculationSetup);
  w.u8(p.temperatureRaw);
  w.u8(p.temperatureUnit);
  w.u8(p.leadTime);
  w.u8(p.holdingTimePlug);
  w.u8(p.holdingTimeBattery);
  w.u16le(p.providerDataId);
  uint8_t nameLen = (p.nameLen > kProfileMaxName) ? kProfileMaxName : p.nameLen;
  w.strLp(p.name, nameLen);
  return w.ok() ? w.size() : 0;
}

bool decodeProfile(const uint8_t* rec, uint16_t len, Profile& out) {
  bap::Reader r(rec, len);
  out = Profile();
  return readProfile(r, out);
}

size_t decodeProfileArray(const uint8_t* body, uint16_t bodyLen,
                          Profile* out, size_t maxOut, ArrayResult* result) {
  ArrayResult res;
  bap::Reader r(body, bodyLen);
  r.u8();                          // arrayId (0x29 full list / 0x2A changed)
  r.u8();                          // totalElementsInList
  const uint8_t flags = r.u8();
  const bool largeIdx = (flags & 0x80) != 0;
  const bool posTransmit = (flags & 0x40) != 0;
  const uint8_t recordAddr = flags & 0x0F;
  const uint16_t startIndex   = largeIdx ? r.u16le() : r.u8();
  const uint16_t elementCount = largeIdx ? r.u16le() : r.u8();
  res.declared = elementCount;
  if (!r.ok()) { res.malformed = true; if (result) *result = res; return 0; }

  size_t count = 0;
  uint16_t i = 0;
  for (; i < elementCount && r.ok(); i++) {
    if (count >= maxOut) { res.truncated = true; break; }
    uint16_t position = (uint16_t)(startIndex + i);
    if (posTransmit) position = largeIdx ? r.u16le() : r.u8();

    if (recordAddr == 6) {         // compact 4-byte record: not modeled -- skip
      r.skip(4);                   // counted toward `declared`, not written to out[]
      continue;
    }

    Profile& p = out[count];
    p = Profile();
    if (!readProfile(r, p)) { res.malformed = true; break; }
    p.position = position;
    count++;
  }
  res.count = (uint16_t)count;
  if (res.truncated || res.malformed) {
    res.complete = false;
  } else if (i >= elementCount) {
    res.complete = true;                 // consumed every declared record
  } else {
    res.malformed = true;                // reader ran dry mid-array
    res.complete = false;
  }
  if (result) *result = res;
  return count;
}

// ---------------------------------------------------------------------------
// Status property names (PlugState / ChargeState / ClimateState)
// ---------------------------------------------------------------------------

namespace {
// One enum value or bit: its name, and whether it has been seen on the wire in the matching car state.
struct ValueName {
  uint8_t value;
  const char* name;
  bool confirmed;
};

// snprintf into out + len, keeping len <= cap - 1 when the output is truncated.
size_t appendf(char* out, size_t cap, size_t len, const char* fmt, const char* a, const char* b) {
  if (len + 1 >= cap) return len;
  int n = snprintf(out + len, cap - len, fmt, a, b);
  if (n < 0) return len;
  len += (size_t)n;
  return len < cap ? len : cap - 1;
}

size_t nameOf(const ValueName* table, size_t n, uint8_t v, char* out, size_t cap) {
  if (cap == 0) return 0;
  for (size_t i = 0; i < n; i++) {
    if (table[i].value == v)
      return appendf(out, cap, 0, "%s%s", table[i].name, table[i].confirmed ? "" : " (unconfirmed)");
  }
  int len = snprintf(out, cap, "unknown (%u)", (unsigned)v);
  return len < 0 ? 0 : ((size_t)len < cap ? (size_t)len : cap - 1);
}

size_t bitNames(const ValueName* table, size_t n, uint8_t bits, char* out, size_t cap) {
  if (cap == 0) return 0;
  out[0] = '\0';
  if (bits == 0) return appendf(out, cap, 0, "%s%s", "none", "");
  size_t len = 0;
  uint8_t known = 0;
  for (size_t i = 0; i < n; i++) {
    known |= table[i].value;
    if (!(bits & table[i].value)) continue;
    if (len) len = appendf(out, cap, len, "%s%s", ",", "");
    len = appendf(out, cap, len, "%s%s", table[i].name, table[i].confirmed ? "" : " (unconfirmed)");
  }
  if (bits & ~known) {
    char hex[8];
    snprintf(hex, sizeof(hex), "0x%02x", (unsigned)(bits & ~known));
    if (len) len = appendf(out, cap, len, "%s%s", ",", "");
    len = appendf(out, cap, len, "unknown bits %s%s", hex, "");
  }
  return len;
}

#define VWE_N(t) (sizeof(t) / sizeof((t)[0]))

const ValueName kClimateStates[] = {
  {CS_INIT, "init", false},
  {CS_ABORT_BODY_OPEN, "aborted: car body open", false},
  {CS_ABORT_IGNITION_ON, "aborted: ignition on", true},
  {CS_ABORT_NO_FUEL, "aborted: park heater fuel low", false},
  {CS_ABORT_HEATER_MAX_TIME, "aborted: park heater max time", false},
  {CS_ABORT_NO_EXT_SUPPLY, "aborted: no external supply", false},
  {CS_ABORT_DEVICE_ERROR, "aborted: device error", false},
  {CS_ABORT_BATTERY_CONDITION, "aborted: battery condition", false},
  {CS_ABORT_BATTERY_CHARGING, "aborted: battery charging", false},
  {CS_ABORT_BATTERY_LOW, "aborted: battery low", false},
  {CS_IDLE, "idle", true},
  {CS_RUNNING, "running", true},
  {CS_ABORT_NOT_IN_PARK, "aborted: not in park", false},
  {CS_ABORT_POWER_REDUCTION, "aborted: power reduction", false},
  {CS_COMPLETED, "completed", false},
};
// seatheaterWindowState uses a subset of the climateState numbers.
const ValueName kHeaterStates[] = {
  {CS_INIT, "init", false},
  {CS_ABORT_IGNITION_ON, "aborted: ignition on", false},
  {CS_ABORT_DEVICE_ERROR, "aborted: device error", false},
  {CS_ABORT_BATTERY_CONDITION, "aborted: battery condition", false},
  {CS_ABORT_BATTERY_LOW, "aborted: battery low", false},
  {CS_IDLE, "idle", true},
  {CS_RUNNING, "running", true},
  {CS_COMPLETED, "completed", false},
};
const ValueName kClimateModes[] = {
  {CM_CLIMATING, "climating", true},
  {CM_DEFROST, "defrost", false},
  {CM_HEATING, "heating", true},
  {CM_COOLING, "cooling", false},
  {CM_VENTILATION, "ventilation", true},
  {CM_FUEL_HEATING, "fuel heating", false},
};
const ValueName kClimateHeaters[] = {
  {CH_SEAT_FL, "seat front left", false},
  {CH_SEAT_FR, "seat front right", false},
  {CH_SEAT_RL, "seat rear left", false},
  {CH_SEAT_RR, "seat rear right", false},
  {CH_WINDOW_FRONT, "front window", true},
  {CH_WINDOW_REAR, "rear window", false},
};
const ValueName kChargeModes[] = {
  {CHM_OFF, "off", true},
  {CHM_AC, "AC", false},
  {CHM_DC, "DC", false},
  {CHM_CONDITIONING, "conditioning", true},
  {CHM_AC_CONDITIONING, "AC + conditioning", false},
  {CHM_DC_CONDITIONING, "DC + conditioning", false},
  {CHM_INIT, "init", false},
};
const ValueName kChargeStates[] = {
  {CHS_INIT, "init", false},
  {CHS_IDLE, "idle", true},
  {CHS_RUNNING, "running", true},  // [WIRE 2026-10-18: AC charging, 0x12]
  {CHS_CONSERVATION, "conservation", false},
  {CHS_ABORT_TEMP_LOW, "aborted: temperature too low", false},
  {CHS_ABORT_DEVICE_ERROR, "aborted: device error", false},
  {CHS_ABORT_NO_SUPPLY, "aborted: power supply not available", false},
  {CHS_ABORT_NOT_IN_PARK, "aborted: not in park", false},
  {CHS_COMPLETED, "completed", false},
};
const ValueName kPlugLockSetups[] = {
  {0, "unlocked", true},
  {1, "locked", false},
  {15, "init", false},
};
const ValueName kPlugLockStates[] = {
  {2, "auto-lock error", false},
  {3, "unlock error", false},
  {15, "no error", true},
};
const ValueName kPlugSupplies[] = {
  {0, "inactive", true},
  {1, "active", true},     // [WIRE 2026-10-18: while charging]
  {2, "charging station connected", false},
  {15, "init", false},
};
const ValueName kPlugStates[] = {
  {0, "unplugged", true},
  {1, "plugged", true},   // [WIRE 2026-10-18: car plugged in and charging]
  {15, "init", false},
};
}  // namespace

size_t climateStateName(uint8_t v, char* out, size_t cap) {
  return nameOf(kClimateStates, VWE_N(kClimateStates), v, out, cap);
}
size_t heaterStateName(uint8_t v, char* out, size_t cap) {
  return nameOf(kHeaterStates, VWE_N(kHeaterStates), v, out, cap);
}
size_t climateModeNames(uint8_t bits, char* out, size_t cap) {
  return bitNames(kClimateModes, VWE_N(kClimateModes), bits, out, cap);
}
size_t climateHeaterNames(uint8_t bits, char* out, size_t cap) {
  return bitNames(kClimateHeaters, VWE_N(kClimateHeaters), bits, out, cap);
}
size_t chargeModeName(uint8_t v, char* out, size_t cap) {
  return nameOf(kChargeModes, VWE_N(kChargeModes), v, out, cap);
}
size_t chargeStateName(uint8_t v, char* out, size_t cap) {
  return nameOf(kChargeStates, VWE_N(kChargeStates), v, out, cap);
}
size_t plugLockSetupName(uint8_t v, char* out, size_t cap) {
  return nameOf(kPlugLockSetups, VWE_N(kPlugLockSetups), v, out, cap);
}
size_t plugLockStateName(uint8_t v, char* out, size_t cap) {
  return nameOf(kPlugLockStates, VWE_N(kPlugLockStates), v, out, cap);
}
size_t plugSupplyName(uint8_t v, char* out, size_t cap) {
  return nameOf(kPlugSupplies, VWE_N(kPlugSupplies), v, out, cap);
}
size_t plugStateName(uint8_t v, char* out, size_t cap) {
  return nameOf(kPlugStates, VWE_N(kPlugStates), v, out, cap);
}

#undef VWE_N

bool findGetAllValue(const uint8_t* body, uint16_t len, uint8_t func, const uint8_t** val,
                     uint16_t* vlen) {
  // This BCU's GetAll layout: properties in function order; fixedLen 0 = u16le length-prefixed.
  static const struct { uint8_t func; uint8_t fixedLen; } kLayout[] = {
    {0x02, 6}, {0x03, 0}, {0x04, 1}, {0x0D, 1}, {0x0E, 6}, {0x0F, 1}, {FUNC_PLUG_STATE, 2},
    {FUNC_CHARGE_STATE, 0}, {FUNC_CLIMATE_STATE, 0}, {FUNC_TIMER_STATE, 2}, {FUNC_TIMER_1, 0},
    {FUNC_TIMER_2, 0}, {FUNC_TIMER_3, 0},
  };
  if (body == nullptr) return false;
  uint32_t pos = 0;
  bool found = false;
  for (const auto& e : kLayout) {
    uint32_t n = e.fixedLen;
    if (n == 0) {
      if (pos + 2 > len) return false;
      n = (uint32_t)body[pos] | ((uint32_t)body[pos + 1] << 8);
      pos += 2;
    }
    if (pos + n > len) return false;
    if (e.func == func) {
      *val = body + pos;
      *vlen = (uint16_t)n;
      found = true;
    }
    pos += n;
  }
  return found && pos == len;  // the whole body must account for exactly this layout
}

}  // namespace egolf
}  // namespace bap
