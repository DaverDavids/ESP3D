/*
  grbl_bridge.cpp - GRBL aware two sender serial arbiter

  Copyright (c) 2014 Luc Lebosse. All rights reserved.

  This code is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This code is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this code; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include "../../include/esp3d_config.h"

#ifdef GRBL_BRIDGE_FEATURE

#include "../serial/serial_service.h"
#include "grbl_bridge.h"

extern HardwareSerial *Serials[];

ESP3DGrblBridge grbl_bridge;

/* GRBL acknowledges every queued class line with exactly one "ok" or
 * "error:". Counting those is what tells us the controller consumed a
 * command, and it stays accurate across a long job because the sender keeps
 * the receive buffer fed. The "ok" arrives when the line is *parsed*, not
 * when motion finishes, which is exactly why ownership is never derived from
 * a quiet timer. */
static bool isGrblAck(const uint8_t *line, size_t len) {
  if (len == 2 && line[0] == 'o' && line[1] == 'k') {
    return true;
  }
  return len >= 6 && memcmp(line, "error:", 6) == 0;
}

/* A status report looks like:
 *   <Idle|MPos:0.000,0.000,0.000|FS:0,0|WCO:0.000,0.000,0.000>
 *   <Run|...|Bf:15,128|FS:100,0>
 *   <Alarm|MPos:0.000,0.000,0.000|FS:0,0>
 * It is the only authoritative source for whether the machine is moving and
 * whether it is in alarm, so both come from here rather than from what a
 * sender typed or from how long the wire has been quiet. */
static void parseStatusReport(const uint8_t *line, size_t len, bool *inMotion,
                              bool *hold, bool *alarm, bool *sawStatus) {
  if (len < 2 || line[0] != '<') {
    return;
  }
  size_t end = len;
  while (end > 1 && line[end - 1] == '>') {
    end--;
  }
  if (end < 2) {
    return;
  }
  *sawStatus = true;
  *inMotion = false;
  *hold = false;
  *alarm = false;
  /* Fields are '|'-separated and the first one is the state word. */
  size_t fieldStart = 1;
  bool first = true;
  for (size_t i = 1; i <= end; i++) {
    if (i == end || line[i] == '|') {
      size_t n = i - fieldStart;
      if (first) {
        first = false;
        if (n == 4 && memcmp(line + fieldStart, "Idle", 4) == 0) {
          *inMotion = false;
        } else if (n == 3 && memcmp(line + fieldStart, "Run", 3) == 0) {
          *inMotion = true;
        } else if ((n >= 4 && memcmp(line + fieldStart, "Hold", 4) == 0) ||
                   (n >= 4 && memcmp(line + fieldStart, "Door", 4) == 0)) {
          *hold = true;
        } else if (n == 5 && memcmp(line + fieldStart, "Alarm", 5) == 0) {
          *alarm = true;
          *hold = true;
        } else if (n >= 5 && memcmp(line + fieldStart, "Check", 5) == 0) {
          *hold = true;
        } else if (n >= 5 && memcmp(line + fieldStart, "Sleep", 5) == 0) {
          *inMotion = false;
        } else if (n >= 3 && memcmp(line + fieldStart, "Jog", 3) == 0) {
          *inMotion = true;
        }
      }
      fieldStart = i + 1;
    }
  }
}

const char *grblBridgeOwnerName(GrblBridgeOwner o) {
  switch (o) {
    case GrblBridgeOwner::remote:
      return "remote";
    case GrblBridgeOwner::web:
      return "web";
    default:
      return "none";
  }
}

const char *grblBridgeFaultName(GrblBridgeFault f) {
  switch (f) {
    case GrblBridgeFault::ack_timeout:
      return "ack-timeout";
    case GrblBridgeFault::resync_timeout:
      return "resync-timeout";
    case GrblBridgeFault::rx_overflow:
      return "rx-overflow";
    case GrblBridgeFault::queue_rejected:
      return "queue-rejected";
    case GrblBridgeFault::oversized_cmd:
      return "oversized-command";
    case GrblBridgeFault::bad_accounting:
      return "bad-accounting";
    default:
      return "none";
  }
}

/* ------------------------------------------------------------------ setup */

bool ESP3DGrblBridge::begin() {
  if (_active) {
    return true;
  }
  /* The bridge serial can be switched off at runtime through
   * ESP_SERIAL_BRIDGE_ON, in which case begin() on that service returned
   * early without opening the UART. Starting arbitration here would leave the
   * mirror with nowhere to go, so it would fill and latch rx-overflow for
   * good, refusing every web command. Refuse to start instead and let the
   * CNC port behave like a plain serial port, which /bridge reports as
   * "off". */
  if (!serial_bridge_service.started() ||
      !esp3d_serial_service.started()) {
    esp3d_log_e("GRBL bridge not started: a UART is down, arbitration is off");
    return false;
  }

  _cncTx = (uint8_t *)malloc(GRBL_BRIDGE_CNC_RING);
  _remoteTx = (uint8_t *)malloc(GRBL_BRIDGE_REMOTE_RING);
  _cncPrio = (uint8_t *)malloc(GRBL_BRIDGE_CNC_PRIO_RING);
  _stateMutex = xSemaphoreCreateMutex();
  _remoteTxMutex = xSemaphoreCreateMutex();
  if (!_cncTx || !_remoteTx || !_cncPrio || !_stateMutex || !_remoteTxMutex) {
    esp3d_log_e("GRBL bridge allocation failed");
    if (_cncTx) {
      free(_cncTx);
      _cncTx = NULL;
    }
    if (_remoteTx) {
      free(_remoteTx);
      _remoteTx = NULL;
    }
    if (_cncPrio) {
      free(_cncPrio);
      _cncPrio = NULL;
    }
    if (_stateMutex) {
      vSemaphoreDelete(_stateMutex);
      _stateMutex = NULL;
    }
    if (_remoteTxMutex) {
      vSemaphoreDelete(_remoteTxMutex);
      _remoteTxMutex = NULL;
    }
    return false;
  }
  _cncTxUsed = 0;
  _remoteTxUsed = 0;
  _cncPrioUsed = 0;
  /* Default to the web sender so an unmodified install behaves like stock
   * ESP3D. The operator grants the pendant the bus explicitly. */
  _owner = GrblBridgeOwner::web;
  _fault = GrblBridgeFault::none;
  _outstanding = 0;
  _alarm = false;
  _resyncing = false;
  _sawStatus = false;
  _inMotion = false;
  _hold = false;
  _resyncSinceMs = 0;
  _statusAtMs = 0;
  _lastProgressMs = millis();
  _pendingMirrorLoss = 0;
  _refused = 0;
  _queueRejected = 0;
  _rxLost = 0;
  _badAcks = 0;
  _remoteLineLen = 0;
  _remoteDiscard = false;
  _remoteSawCR = false;
  _webLineLen = 0;
  _webDiscard = false;
  _webSawCR = false;
  _respLen = 0;
  _respDiscard = false;
  _active = true;
  esp3d_log("GRBL bridge active, owner=web, hand over with [ESP430]");
  return true;
}

void ESP3DGrblBridge::end() {
  /* Both serial services must already be stopped before this runs, otherwise
   * a receive task could still be inside a callback and touch the buffers
   * after they are freed. Esp3D::end() orders it that way.
   *
   * Clearing _active first makes any late callback return immediately, and
   * taking the mutex proves no task is currently inside a critical section,
   * because nothing nests locks. */
  _active = false;
  if (_stateMutex && xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    _flushCncQueuesLocked();
    xSemaphoreGive(_stateMutex);
  }
  if (_stateMutex) {
    vSemaphoreDelete(_stateMutex);
    _stateMutex = NULL;
  }
  if (_remoteTxMutex) {
    vSemaphoreDelete(_remoteTxMutex);
    _remoteTxMutex = NULL;
  }
  if (_cncTx) {
    free(_cncTx);
    _cncTx = NULL;
  }
  if (_remoteTx) {
    free(_remoteTx);
    _remoteTx = NULL;
  }
  if (_cncPrio) {
    free(_cncPrio);
    _cncPrio = NULL;
  }
  _cncTxUsed = 0;
  _remoteTxUsed = 0;
  _cncPrioUsed = 0;
}

/* ------------------------------------------------------- locked internals */

void ESP3DGrblBridge::_setFaultLocked(GrblBridgeFault f, const char *detail) {
  if (_fault == GrblBridgeFault::none) {
    _fault = f;
    esp3d_log_e("GRBL bridge fault latched: %s (%s)",
                grblBridgeFaultName(f), detail ? detail : "");
  }
}

void ESP3DGrblBridge::_flushCncQueuesLocked() {
  _cncTxUsed = 0;
  _cncPrioUsed = 0;
}

/* A soft reset makes GRBL discard every line it had buffered. Anything we
 * still hold must go too, or it would be delivered after the reset and
 * silently counted against a controller that never saw it. New normal
 * commands are refused until the controller proves it is back with a status
 * report, so an "ok" from before the reset can never be mistaken for an
 * acknowledgement of something admitted after it. */
void ESP3DGrblBridge::_enterResyncLocked(const char *why) {
  _flushCncQueuesLocked();
  _outstanding = 0;
  _resyncing = true;
  _resyncSinceMs = millis();
  esp3d_log("GRBL bridge resyncing: %s", why ? why : "reset");
}

/* All or nothing. A partially queued command would reach the controller as a
 * truncated line, which is how a valid move becomes a different valid move,
 * so on any shortfall nothing at all is copied. */
bool ESP3DGrblBridge::_cncPushLocked(const uint8_t *data, size_t len) {
  if (len == 0) {
    return true;
  }
  if (_cncTxUsed + len > GRBL_BRIDGE_CNC_RING ||
      _cncTxUsed + len > GRBL_BRIDGE_CNC_HIGH_WATER) {
    // refuse the whole command and keep the high water mark bounded
    _queueRejected++;
    esp3d_log_e("GRBL bridge refused %d bytes, queue is full", (int)len);
    return false;
  }
  memcpy(_cncTx + _cncTxUsed, data, len);
  _cncTxUsed += len;
  return true;
}

bool ESP3DGrblBridge::_cncPushRealtimeLocked(uint8_t c) {
  if (_cncPrioUsed >= GRBL_BRIDGE_CNC_PRIO_RING) {
    // The realtime path is tiny and normally drained every loop, so reaching
    // its limit means the UART is genuinely wedged. Latch rather than drop a
    // safety byte silently.
    _setFaultLocked(GrblBridgeFault::queue_rejected,
                    "realtime queue full, UART wedged");
    return false;
  }
  _cncPrio[_cncPrioUsed++] = c;
  return true;
}

void ESP3DGrblBridge::_handleRealtimeLocked(uint8_t c) {
  /* Realtime bytes are never blocked and never counted. They are the only
   * thing that still works while a fault is latched, which is deliberate:
   * feed hold and reset have to remain reachable. */
  _cncPushRealtimeLocked(c);
  if (c == 0x18) {
    _enterResyncLocked("soft reset");
    // motion and alarm are unknown until the controller says otherwise
    _alarm = false;
    _inMotion = false;
    _hold = false;
    _sawStatus = false;
    _statusAtMs = 0;
  }
}

void ESP3DGrblBridge::_drainMirrorLossLocked() {
  if (_pendingMirrorLoss) {
    _rxLost += _pendingMirrorLoss;
    _pendingMirrorLoss = 0;
    _setFaultLocked(GrblBridgeFault::rx_overflow,
                    "CNC to remote mirror overflowed");
  }
}

/* ------------------------------------------------------------------- state */

ESP3DGrblBridge::Status ESP3DGrblBridge::status() const {
  Status s = {};
  s.active = false;
  if (!_stateMutex) {
    return s;
  }
  if (xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(20)) != pdTRUE) {
    // Deliberately reports active=false: a caller that cannot get a coherent
    // view must not be told the bus is free.
    esp3d_log_e("GRBL bridge status contended");
    return s;
  }
  s.active = _active;
  s.faulted = _fault != GrblBridgeFault::none;
  s.fault = _fault;
  s.owner = _owner;
  s.outstanding = _outstanding;
  s.alarm = _alarm;
  s.resyncing = _resyncing;
  s.sawStatus = _sawStatus;
  s.inMotion = _inMotion;
  s.hold = _hold;
  s.refused = _refused;
  s.queueRejected = _queueRejected;
  s.rxLost = _rxLost;
  s.badAcks = _badAcks;
  xSemaphoreGive(_stateMutex);
  return s;
}

bool ESP3DGrblBridge::webTxAllowed() const {
  if (!_stateMutex || xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(20)) != pdTRUE) {
    // Fail closed: a web command whose ownership could not be confirmed is
    // refused rather than admitted on a guess.
    return false;
  }
  bool ok = _active && _fault == GrblBridgeFault::none && !_resyncing &&
            _owner == GrblBridgeOwner::web;
  xSemaphoreGive(_stateMutex);
  return ok;
}

bool ESP3DGrblBridge::requestOwner(GrblBridgeOwner who, const char **why) {
  const char *dummy = nullptr;
  if (!why) {
    why = &dummy;
  }
  if (!_stateMutex || xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
    *why = "state busy";
    return false;
  }
  bool ok = true;
  if (!_active) {
    *why = "bridge not active";
    ok = false;
  } else if (_fault != GrblBridgeFault::none) {
    *why = "a fault is latched, clear it with [ESP431]";
    ok = false;
  } else if (_resyncing) {
    *why = "waiting for the controller to resynchronise after a reset";
    ok = false;
  } else if (_outstanding) {
    // Handing over with commands in flight is exactly the interleave this
    // module exists to prevent.
    *why = "commands are still in flight";
    ok = false;
  } else if (!_sawStatus) {
    // Without a single status report there is no evidence the controller is
    // even connected, let alone idle.
    *why = "no status report received yet from the controller";
    ok = false;
  } else if ((millis() - _statusAtMs) > GRBL_BRIDGE_STATUS_FRESH_MS) {
    // A stale report cannot prove the machine is idle right now. Refusing is
    // the whole point: silence used to be read as "finished".
    *why = "the last status report is too old to prove the machine is idle";
    ok = false;
  } else if (_inMotion) {
    *why = "the controller reports it is running";
    ok = false;
  } else if (_hold) {
    *why = "the controller is on hold";
    ok = false;
  }
  if (ok && who != _owner) {
    _owner = who;
    esp3d_log("GRBL bridge owner changed to %s", grblBridgeOwnerName(who));
  }
  xSemaphoreGive(_stateMutex);
  return ok;
}

bool ESP3DGrblBridge::clearFault() {
  if (!_stateMutex || xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
    return false;
  }
  bool was = _fault != GrblBridgeFault::none || _resyncing;
  _fault = GrblBridgeFault::none;
  _resyncing = false;
  _pendingMirrorLoss = 0;
  // discard anything queued before the fault so it cannot surface later
  _flushCncQueuesLocked();
  _outstanding = 0;
  // drop half received lines: they were part of the failed exchange
  _remoteLineLen = 0;
  _remoteDiscard = false;
  _remoteSawCR = false;
  _webLineLen = 0;
  _webDiscard = false;
  _webSawCR = false;
  _respLen = 0;
  _respDiscard = false;
  _lastProgressMs = millis();
  xSemaphoreGive(_stateMutex);
  if (was) {
    esp3d_log("GRBL bridge fault cleared by operator");
  }
  return true;
}

/* ---------------------------------------------------------------- assemble */

/* Feed bytes from one sender, assembling lines and admitting each complete
 * one exactly once. Counting happens at the terminator, so a command split
 * across several writes, or delivered as CR, LF or CRLF, is accounted once
 * and an incomplete fragment is never counted at all.
 *
 * The caller holds _stateMutex for the whole call, which is what makes the
 * ownership check, the queue copy and the in-flight increment a single
 * transaction with no window in between.
 *
 * A line longer than LINE_MAX is discarded whole and latches a fault rather
 * than being flushed in pieces: splitting it would fabricate several commands
 * out of one and could produce a valid but unintended move. */
void ESP3DGrblBridge::_feedSenderLocked(const uint8_t *data, size_t len,
                                        bool fromWeb) {
  uint8_t *line = fromWeb ? _webLine : _remoteLine;
  size_t *lineLen = fromWeb ? &_webLineLen : &_remoteLineLen;
  bool *discard = fromWeb ? &_webDiscard : &_remoteDiscard;
  bool *sawCR = fromWeb ? &_webSawCR : &_remoteSawCR;
  GrblBridgeOwner me = fromWeb ? GrblBridgeOwner::web : GrblBridgeOwner::remote;

  for (size_t i = 0; i < len; i++) {
    uint8_t c = data[i];

    if (ESP3DGrblBridge::isRealtime(&c, 1)) {
      /* A realtime byte is not line content and does not terminate a partial
       * line. Forwarding it here is also what lets the other sender stop the
       * machine even while it owns nothing. */
      _handleRealtimeLocked(c);
      continue;
    }

    // swallow the LF of a CRLF pair so a CR+LF counts as one terminator
    if (c == '\n' && *sawCR) {
      *sawCR = false;
      continue;
    }
    *sawCR = (c == '\r');

    if (c == '\r' || c == '\n') {
      *sawCR = false;
      if (*discard) {
        *discard = false;
        *lineLen = 0;
        continue;
      }
      if (*lineLen == 0) {
        continue;  // empty line, nothing to send
      }
      // Complete command: append the canonical terminator and admit it.
      uint8_t out[GRBL_BRIDGE_LINE_MAX + 1];
      size_t n = *lineLen;
      memcpy(out, line, n);
      out[n++] = '\n';
      *lineLen = 0;

      if (_fault != GrblBridgeFault::none) {
        _refused++;
        esp3d_log_e("GRBL bridge refused %s command, fault latched (%s)",
                    grblBridgeOwnerName(me),
                    grblBridgeFaultName(_fault));
      } else if (_resyncing) {
        _refused++;
        esp3d_log_e("GRBL bridge refused %s command, resynchronising",
                    grblBridgeOwnerName(me));
      } else if (_owner != me) {
        _refused++;
        esp3d_log_e("GRBL bridge refused %s command, %s owns the bus",
                    grblBridgeOwnerName(me),
                    grblBridgeOwnerName(_owner));
      } else if (_cncPushLocked(out, n)) {
        _outstanding++;
        _lastProgressMs = millis();
      }
      continue;
    }

    if (*lineLen < GRBL_BRIDGE_LINE_MAX) {
      line[(*lineLen)++] = c;
    } else if (!*discard) {
      *discard = true;
      *lineLen = 0;
      _setFaultLocked(GrblBridgeFault::oversized_cmd,
                      "command longer than GRBL_BRIDGE_LINE_MAX");
    }
  }
}

void ESP3DGrblBridge::onRemoteBytes(const uint8_t *data, size_t len) {
  if (!_active || !data || !len || !_stateMutex) {
    return;
  }
  if (xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(20)) != pdTRUE) {
    esp3d_log_e("GRBL bridge remote rx contended, bytes dropped");
    return;
  }
  _feedSenderLocked(data, len, /*fromWeb=*/false);
  xSemaphoreGive(_stateMutex);
}

bool ESP3DGrblBridge::enqueueWebCnc(const uint8_t *data, size_t len) {
  if (!_active) {
    return true;  // not arbitrating, let the caller write straight through
  }
  if (!data || !len || !_stateMutex) {
    return false;
  }
  if (xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(20)) != pdTRUE) {
    esp3d_log_e("GRBL bridge web tx contended, bytes dropped");
    return false;
  }
  _feedSenderLocked(data, len, /*fromWeb=*/true);
  xSemaphoreGive(_stateMutex);
  return true;
}

/* ------------------------------------------------------------------- cnc rx */

/* Caller holds _stateMutex. */
void ESP3DGrblBridge::_parseCncLineLocked() {
  while (_respLen && _resp[_respLen - 1] == '\r') {
    _respLen--;
  }
  if (_respDiscard || _respLen == 0) {
    return;
  }
  _resp[_respLen] = 0;  // in bounds: _resp is LINE_MAX + 1

  bool inMotion = false, hold = false, alarm = false, sawStatus = false;
  parseStatusReport(_resp, _respLen, &inMotion, &hold, &alarm, &sawStatus);

  if (sawStatus) {
    _sawStatus = true;
    _statusAtMs = millis();
    _inMotion = inMotion;
    _hold = hold;
    if (alarm) {
      _alarm = true;
    } else {
      // any non alarm report means the controller is answering again
      _alarm = false;
    }
    // A status report proves the controller is alive and parsing, so a
    // pending resync is satisfied.
    if (_resyncing) {
      _resyncing = false;
      esp3d_log("GRBL bridge resynchronised on status report");
    }
    return;
  }

  if (_respLen >= 4 && memcmp(_resp, "Grbl", 4) == 0) {
    // Startup banner, or the banner after a hard reset.
    _outstanding = 0;
    _alarm = false;
    _inMotion = false;
    _hold = false;
    _sawStatus = false;
    _statusAtMs = 0;
    if (_resyncing) {
      _resyncing = false;
      esp3d_log("GRBL bridge resynchronised on banner");
    }
    return;
  }

  if (_respLen >= 5 && memcmp(_resp, "ALARM", 5) == 0) {
    // Derived from what the controller reported, never from the fact that a
    // resume byte was sent.
    _alarm = true;
    esp3d_log_e("GRBL controller reported ALARM");
    return;
  }

  if (isGrblAck(_resp, _respLen)) {
    if (_resyncing) {
      _resyncing = false;
    }
    if (_outstanding) {
      _outstanding--;
    } else {
      // An ack with nothing outstanding means our accounting and the wire
      // have diverged, which makes ownership meaningless. Latch it rather
      // than guess which sender it belonged to.
      _badAcks++;
      _setFaultLocked(GrblBridgeFault::bad_accounting,
                      "acknowledgement with nothing outstanding");
    }
    _lastProgressMs = millis();
  }
  // Everything else ([MSG:...], [GC:...], settings dumps) is intentionally
  // ignored: it is not an acknowledgement and must not move the count.
}

void ESP3DGrblBridge::onCncBytes(const uint8_t *data, size_t len) {
  if (!_active || !data || !len || !_stateMutex) {
    return;
  }
  // Mirror the untouched stream to the remote first. The remote is an
  // operator display: it must see every byte the CNC emitted, including
  // status reports and realtime answers, regardless of who owns the bus.
  _remoteMirror(data, len);

  if (xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(20)) != pdTRUE) {
    esp3d_log_e("GRBL bridge cnc rx contended, state not updated");
    return;
  }
  for (size_t i = 0; i < len; i++) {
    uint8_t c = data[i];
    if (ESP3DGrblBridge::isRealtime(&c, 1)) {
      /* GRBL answers a realtime byte without a terminator ('~', '?', '!'),
       * so it must never enter the line assembler: it would be glued onto
       * the front of the next line and corrupt the acknowledgement. Flush
       * whatever was half collected and let the answer arrive as its own
       * line. */
      _respLen = 0;
      _respDiscard = false;
      continue;
    }
    if (c == '\n') {
      _parseCncLineLocked();
      // Reset for the next line, and clear the oversized flag so a long line
      // is discarded exactly once rather than re-triggering per character.
      _respLen = 0;
      _respDiscard = false;
    } else if (_respDiscard) {
      // swallow the rest of an oversized line until its terminator
      continue;
    } else if (_respLen < GRBL_BRIDGE_LINE_MAX) {
      _resp[_respLen++] = c;
    } else {
      _respDiscard = true;
      _respLen = 0;
    }
  }
  xSemaphoreGive(_stateMutex);
}

/* -------------------------------------------------------------------- pump */

/* Pops under the lock, writes outside it. A UART write may block for a whole
 * character time, and nothing may hold _stateMutex across that. */
void ESP3DGrblBridge::_pumpCncTx() {
  if (!esp3d_serial_service.started() || !_stateMutex) {
    return;
  }
  uint8_t index = esp3d_serial_service.serialIndex();
  uint8_t scratch[64];

  /* Realtime first, always. A feed hold or reset must not wait behind a
   * backlog of normal traffic. */
  for (;;) {
    if (xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
      return;
    }
    bool have = _cncPrioUsed > 0;
    uint8_t c = 0;
    if (have) {
      c = _cncPrio[0];
      memmove(_cncPrio, _cncPrio + 1, _cncPrioUsed - 1);
      _cncPrioUsed--;
    }
    xSemaphoreGive(_stateMutex);
    if (!have) {
      break;
    }
    Serials[index]->write(&c, 1);
  }

  for (size_t i = 0; i < GRBL_BRIDGE_PUMP_MAX; i += sizeof(scratch)) {
    if (xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
      return;
    }
    size_t n = (_cncTxUsed < sizeof(scratch)) ? _cncTxUsed : sizeof(scratch);
    if (n) {
      memcpy(scratch, _cncTx, n);
      memmove(_cncTx, _cncTx + n, _cncTxUsed - n);
      _cncTxUsed -= n;
    }
    xSemaphoreGive(_stateMutex);
    if (!n) {
      return;
    }
    size_t got = Serials[index]->write(scratch, n);
    if (got != n) {
      /* HardwareSerial should not come up short on a blocking write, but if
       * it does the bytes must go back to the head of the queue rather than
       * being dropped: a truncated G code line is an unintended move. */
      if (xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (got < n && _cncTxUsed + (n - got) <= GRBL_BRIDGE_CNC_RING) {
          memmove(_cncTx + (n - got), _cncTx, _cncTxUsed);
          memcpy(_cncTx, scratch + got, n - got);
          _cncTxUsed += (n - got);
        }
        xSemaphoreGive(_stateMutex);
      }
      esp3d_log_e("GRBL bridge short CNC write %d/%d, rest requeued", (int)got,
                  (int)n);
      return;
    }
  }
}

size_t ESP3DGrblBridge::_remotePop(uint8_t *out, size_t max) {
  if (!_remoteTxMutex ||
      xSemaphoreTake(_remoteTxMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
    return 0;
  }
  size_t n = (_remoteTxUsed < max) ? _remoteTxUsed : max;
  if (n) {
    memcpy(out, _remoteTx, n);
    memmove(_remoteTx, _remoteTx + n, _remoteTxUsed - n);
    _remoteTxUsed -= n;
  }
  xSemaphoreGive(_remoteTxMutex);
  return n;
}

void ESP3DGrblBridge::_remoteMirror(const uint8_t *data, size_t len) {
  if (!_remoteTxMutex ||
      xSemaphoreTake(_remoteTxMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
    /* Losing the lock here means losing bytes. Record it without reaching
     * for _stateMutex, which is what keeps the lock hierarchy one way. */
    _pendingMirrorLoss += len;
    return;
  }
  if (_remoteTxUsed + len > GRBL_BRIDGE_REMOTE_RING) {
    // The mirror is a display feed. Dropping is the only option, but it must
    // be counted and latched rather than passing silently.
    esp3d_log_e("GRBL bridge remote mirror overflow, %d bytes lost", (int)len);
    _pendingMirrorLoss += len;
  } else {
    memcpy(_remoteTx + _remoteTxUsed, data, len);
    _remoteTxUsed += len;
  }
  xSemaphoreGive(_remoteTxMutex);
}

void ESP3DGrblBridge::_pumpRemoteTx() {
  if (!serial_bridge_service.started() || !_remoteTxMutex) {
    return;
  }
  uint8_t index = serial_bridge_service.serialIndex();
  uint8_t scratch[64];
  for (size_t i = 0; i < GRBL_BRIDGE_PUMP_MAX; i += sizeof(scratch)) {
    size_t n = _remotePop(scratch, sizeof(scratch));
    if (!n) {
      return;
    }
    size_t got = Serials[index]->write(scratch, n);
    if (got != n) {
      _remoteRestore(scratch + got, n - got);
      esp3d_log_e("GRBL bridge short remote write %d/%d", (int)got, (int)n);
      return;
    }
  }
}

void ESP3DGrblBridge::_remoteRestore(const uint8_t *data, size_t len) {
  if (!_remoteTxMutex ||
      xSemaphoreTake(_remoteTxMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
    _pendingMirrorLoss += len;
    return;
  }
  if (_remoteTxUsed + len > GRBL_BRIDGE_REMOTE_RING) {
    _pendingMirrorLoss += len;
  } else {
    memmove(_remoteTx + len, _remoteTx, _remoteTxUsed);
    memcpy(_remoteTx, data, len);
    _remoteTxUsed += len;
  }
  xSemaphoreGive(_remoteTxMutex);
}

/* ------------------------------------------------------------------ handle */

void ESP3DGrblBridge::handle() {
  if (!_active || !_stateMutex) {
    return;
  }
  uint32_t now = millis();
  if (xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
    return;
  }
  _drainMirrorLossLocked();
  if (_resyncing && (now - _resyncSinceMs) > GRBL_BRIDGE_RESYNC_TIMEOUT_MS) {
    _setFaultLocked(GrblBridgeFault::resync_timeout,
                    "no status report after the reset");
    _resyncing = false;
  }
  /* Stalled progress latches a fault. It never releases the bus, because "no
   * ok for a while" is indistinguishable from "mid cut" and handing the bus
   * over there is what this module must never do. */
  if ((_fault == GrblBridgeFault::none) && _outstanding &&
      (now - _lastProgressMs) > GRBL_BRIDGE_ACK_TIMEOUT_MS) {
    _setFaultLocked(GrblBridgeFault::ack_timeout,
                    "a command was never acknowledged");
  }
  xSemaphoreGive(_stateMutex);
  _pumpCncTx();
  _pumpRemoteTx();
}

#endif  // GRBL_BRIDGE_FEATURE
