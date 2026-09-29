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

/* GRBL realtime characters, per the Grbl 1.1 protocol:
 *   '?'  status poll     '!' feed hold     '~' cycle start / resume
 *   0x18 soft reset      0x80-0xA4 realtime set (safety door, jog cancel, ...)
 * These are never buffered by GRBL, are never answered with ok/error, and
 * are safe from any sender.
 *
 * Deliberately not esp3d_string::isRealTimeCommand(): that helper is gated
 * on the runtime firmware target setting, which seeds from DEFAULT_FW but can
 * be changed in the UI and then persists in NVS. A stale value would quietly
 * buffer realtime bytes as line content and wedge the remote. This module is
 * GRBL only, so it carries its own table. The public entry point is
 * ESP3DGrblBridge::isRealtime(). */
static bool isGrblRealtime(uint8_t c) {
  return ESP3DGrblBridge::isRealtime(&c, 1);
}

/* GRBL is lock step: every queued class line is answered by exactly one
 * "ok" or "error:". Counting those is what tells us the bus is free, and
 * it stays accurate during a multi minute job because the sender keeps the
 * receive buffer fed. */
static bool isGrblAck(const uint8_t *line, size_t len, size_t *lines) {
  if (len == 2 && line[0] == 'o' && line[1] == 'k') {
    *lines = 1;
    return true;
  }
  if (len >= 6 && memcmp(line, "error:", 6) == 0) {
    *lines = 1;
    return true;
  }
  return false;
}

bool ESP3DGrblBridge::begin() {
  if (_active) {
    return true;
  }
  _cncTx = (uint8_t *)malloc(GRBL_BRIDGE_CNC_RING);
  _remoteTx = (uint8_t *)malloc(GRBL_BRIDGE_REMOTE_RING);
  _cncTxMutex = xSemaphoreCreateMutex();
  _remoteTxMutex = xSemaphoreCreateMutex();
  _stateMutex = xSemaphoreCreateMutex();
  _lineMutex = xSemaphoreCreateMutex();
  if (!_cncTx || !_remoteTx || !_cncTxMutex || !_remoteTxMutex ||
      !_stateMutex || !_lineMutex) {
    esp3d_log_e("GRBL bridge allocation failed");
    if (_cncTx) {
      free(_cncTx);
      _cncTx = NULL;
    }
    if (_remoteTx) {
      free(_remoteTx);
      _remoteTx = NULL;
    }
    if (_cncTxMutex) {
      vSemaphoreDelete(_cncTxMutex);
      _cncTxMutex = NULL;
    }
    if (_remoteTxMutex) {
      vSemaphoreDelete(_remoteTxMutex);
      _remoteTxMutex = NULL;
    }
    if (_stateMutex) {
      vSemaphoreDelete(_stateMutex);
      _stateMutex = NULL;
    }
    if (_lineMutex) {
      vSemaphoreDelete(_lineMutex);
      _lineMutex = NULL;
    }
    return false;
  }
  _cncTxUsed = 0;
  _remoteTxUsed = 0;
  _owner = GrblBridgeOwner::none;
  _outstanding = 0;
  _alarm = false;
  _busQuiet = false;
  _busQuietMs = 0;
  _remoteLineLen = 0;
  _respLen = 0;
  _ownerLastMs = millis();
  _remoteLastByteMs = millis();
  _remoteDropped = 0;
  _cncOverflow = 0;
  _remoteOverflow = 0;
  _active = true;
  esp3d_log("GRBL bridge active, %d/%d byte rings", GRBL_BRIDGE_CNC_RING,
            GRBL_BRIDGE_REMOTE_RING);
  return true;
}

void ESP3DGrblBridge::end() {
  /* Stop accepting callbacks first, then let any receive callback that is
   * already inside this object finish before the buffers and mutexes go
   * away. Without the yield an UART task could have passed the _active
   * check, been preempted, and then touch freed memory. Shutdown only, so a
   * short block here costs nothing. */
  _active = false;
  vTaskDelay(pdMS_TO_TICKS(2));
  if (_cncTxMutex) {
    vSemaphoreDelete(_cncTxMutex);
    _cncTxMutex = NULL;
  }
  if (_remoteTxMutex) {
    vSemaphoreDelete(_remoteTxMutex);
    _remoteTxMutex = NULL;
  }
  if (_stateMutex) {
    vSemaphoreDelete(_stateMutex);
    _stateMutex = NULL;
  }
  if (_lineMutex) {
    vSemaphoreDelete(_lineMutex);
    _lineMutex = NULL;
  }
  if (_cncTx) {
    free(_cncTx);
    _cncTx = NULL;
  }
  if (_remoteTx) {
    free(_remoteTx);
    _remoteTx = NULL;
  }
  _cncTxUsed = 0;
  _remoteTxUsed = 0;
}

size_t ESP3DGrblBridge::_cncPush(const uint8_t *data, size_t len) {
  if (xSemaphoreTake(_cncTxMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
    return 0;
  }
  size_t room = GRBL_BRIDGE_CNC_RING - _cncTxUsed;
  size_t n = (len < room) ? len : room;
  if (n) {
    memcpy(_cncTx + _cncTxUsed, data, n);
    _cncTxUsed += n;
  }
  if (n < len) {
    _cncOverflow += (len - n);
    esp3d_log_e("GRBL bridge CNC tx overflow, %d bytes lost", len - n);
  }
  xSemaphoreGive(_cncTxMutex);
  return n;
}

size_t ESP3DGrblBridge::_cncPop(uint8_t *out, size_t max) {
  if (xSemaphoreTake(_cncTxMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
    return 0;
  }
  size_t n = (_cncTxUsed < max) ? _cncTxUsed : max;
  if (n) {
    memcpy(out, _cncTx, n);
    _cncTxUsed -= n;
    if (_cncTxUsed) {
      memmove(_cncTx, _cncTx + n, _cncTxUsed);
    }
  }
  xSemaphoreGive(_cncTxMutex);
  return n;
}

size_t ESP3DGrblBridge::_remotePush(const uint8_t *data, size_t len) {
  if (xSemaphoreTake(_remoteTxMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
    return 0;
  }
  size_t room = GRBL_BRIDGE_REMOTE_RING - _remoteTxUsed;
  size_t n = (len < room) ? len : room;
  if (n) {
    memcpy(_remoteTx + _remoteTxUsed, data, n);
    _remoteTxUsed += n;
  }
  if (n < len) {
    _remoteOverflow += (len - n);
    esp3d_log_e("GRBL bridge remote tx overflow, %d bytes lost", len - n);
  }
  xSemaphoreGive(_remoteTxMutex);
  return n;
}

size_t ESP3DGrblBridge::_remotePop(uint8_t *out, size_t max) {
  if (xSemaphoreTake(_remoteTxMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
    return 0;
  }
  size_t n = (_remoteTxUsed < max) ? _remoteTxUsed : max;
  if (n) {
    memcpy(out, _remoteTx, n);
    _remoteTxUsed -= n;
    if (_remoteTxUsed) {
      memmove(_remoteTx, _remoteTx + n, _remoteTxUsed);
    }
  }
  xSemaphoreGive(_remoteTxMutex);
  return n;
}

/* ---------------------------------------------------------------- lease */

/* Both helpers are only ever called with _stateMutex already held, so the
 * owner change and the outstanding count are observed as one unit. */

void ESP3DGrblBridge::_takeLease(GrblBridgeOwner who) {
  if (_owner != GrblBridgeOwner::none) {
    return;
  }
  _owner = who;
  _ownerLastMs = millis();
  _busQuiet = false;
  esp3d_log("GRBL bridge bus taken by %s",
            who == GrblBridgeOwner::remote ? "remote" : "web");
}

void ESP3DGrblBridge::_releaseLease() {
  if (_owner == GrblBridgeOwner::none) {
    return;
  }
  esp3d_log("GRBL bridge bus released by %s",
            _owner == GrblBridgeOwner::remote ? "remote" : "web");
  _owner = GrblBridgeOwner::none;
  _outstanding = 0;
  _busQuiet = false;
}

/* ---------------------------------------------------------------- CNC RX */

void ESP3DGrblBridge::_noteRealtime(uint8_t c) {
  if (_stateMutex && xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    if (c == 0x18) {
      // soft reset: GRBL drops every queued line, answers none of them
      _outstanding = 0;
      _alarm = false;
    } else if (c == '~') {
      // cycle start resume clears alarm
      _alarm = false;
    }
    xSemaphoreGive(_stateMutex);
  }
}

void ESP3DGrblBridge::_parseCncLine() {
  // strip trailing CR
  while (_respLen && (_resp[_respLen - 1] == '\r')) {
    _respLen--;
  }
  if (_respLen == 0) {
    return;
  }
  _resp[_respLen] = 0;
  if (_resp[0] == 'G' && _respLen >= 4 && memcmp(_resp, "Grbl", 4) == 0) {
    // welcome banner, implies a fresh controller
    if (_stateMutex &&
        xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      _outstanding = 0;
      _alarm = false;
      xSemaphoreGive(_stateMutex);
    }
  } else if (_resp[0] == 'A' && _respLen >= 5 &&
             memcmp(_resp, "ALARM", 5) == 0) {
    if (_stateMutex &&
        xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      _alarm = true;
      xSemaphoreGive(_stateMutex);
    }
  } else {
    size_t lines = 0;
    if (isGrblAck(_resp, _respLen, &lines) && _stateMutex &&
        xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      if (_outstanding) {
        _outstanding = (uint16_t)(_outstanding - lines);
      }
      xSemaphoreGive(_stateMutex);
    }
  }
}

void ESP3DGrblBridge::onCncBytes(const uint8_t *data, size_t len) {
  if (!_active || !data || !len) {
    return;
  }
  // Mirror the untouched stream to the remote first. The remote is an
  // operator display: it must see every byte the CNC emitted, including
  // status reports and realtime answers, regardless of who owns the bus.
  _remotePush(data, len);
  // Then account for acknowledgements. This runs before any ESP3D line
  // splitting so the counters match exactly what left the UART.
  for (size_t i = 0; i < len; i++) {
    uint8_t c = data[i];
    if (c == '\n') {
      _parseCncLine();
      _respLen = 0;
    } else {
      if (_respLen < GRBL_BRIDGE_LINE_MAX) {
        _resp[_respLen++] = c;
      } else {
        // oversized line, resync on the next newline
        _respLen = 0;
      }
    }
  }
}

/* ------------------------------------------------------------- remote RX */

void ESP3DGrblBridge::_commitRemoteLine(bool terminated) {
  if (_remoteLineLen == 0) {
    return;
  }
  /* Decide ownership and count the line as one atomic step. If these were
   * separate, the loop task could observe owner=remote with outstanding=0
   * and release the lease, leaving a remote line in flight that the web
   * would then be allowed to splice into. */
  bool refuse = false;
  if (xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
    _remoteLineLen = 0;
    return;
  }
  if (_owner != GrblBridgeOwner::none && _owner != GrblBridgeOwner::remote) {
    /* The web is mid job. GRBL would splice this line into the middle of
     * the running program, so it is dropped rather than queued. Realtime
     * bytes still get through, so feed hold and reset remain available. */
    refuse = true;
    _remoteDropped++;
  } else {
    _takeLease(GrblBridgeOwner::remote);
    _outstanding++;
    _ownerLastMs = millis();
  }
  xSemaphoreGive(_stateMutex);

  if (refuse) {
    esp3d_log_e("GRBL bridge dropped remote line, web owns the bus");
    _remoteLineLen = 0;
    return;
  }
  size_t n = _remoteLineLen;
  if (terminated && n < sizeof(_remoteLine)) {
    _remoteLine[n++] = '\n';
  }
  _remoteLineLen = 0;
  if (_cncPush(_remoteLine, n) != n) {
    // ring was full, the line never reached the bus, so undo the count
    if (xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      if (_outstanding) {
        _outstanding--;
      }
      xSemaphoreGive(_stateMutex);
    }
  }
}

void ESP3DGrblBridge::onRemoteBytes(const uint8_t *data, size_t len) {
  if (!_active || !data || !len) {
    return;
  }
  if (!_lineMutex || xSemaphoreTake(_lineMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
    return;
  }
  _remoteLastByteMs = millis();
  for (size_t i = 0; i < len; i++) {
    uint8_t c = data[i];
    if (isGrblRealtime(c)) {
      /* Realtime class: GRBL never buffers these, never answers them, and
       * they are safe from any sender. They bypass the lease entirely so
       * stop, reset and jog cancel keep working mid job. */
      _cncPush(&c, 1);
      _noteRealtime(c);
      // a realtime byte does not terminate a partial queued line
      continue;
    }
    if (c == '\n') {
      _commitRemoteLine(true);
      continue;
    }
    if (_remoteLineLen < GRBL_BRIDGE_LINE_MAX) {
      _remoteLine[_remoteLineLen++] = c;
    } else {
      // overlong line, send what we have rather than stalling the remote
      _commitRemoteLine(false);
      _remoteLine[_remoteLineLen++] = c;
    }
  }
  xSemaphoreGive(_lineMutex);
}

void ESP3DGrblBridge::_flushIdleRemoteLine() {
  if (!_lineMutex || xSemaphoreTake(_lineMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
    return;
  }
  if (_remoteLineLen == 0) {
    xSemaphoreGive(_lineMutex);
    return;
  }
  if ((millis() - _remoteLastByteMs) < GRBL_BRIDGE_IDLE_LINE_FLUSH_MS) {
    xSemaphoreGive(_lineMutex);
    return;
  }
  _commitRemoteLine(false);
  xSemaphoreGive(_lineMutex);
}

/* ---------------------------------------------------------------- web TX */

bool ESP3DGrblBridge::enqueueWebCnc(const uint8_t *data, size_t len) {
  if (!_active) {
    return true;  // not arbitrating, let the caller write straight through
  }
  if (!data || !len) {
    return false;
  }
  if (len == 1 && isGrblRealtime(data[0])) {
    _cncPush(data, len);
    _noteRealtime(data[0]);
    return true;
  }
  /* Same rule as the remote side: the ownership check and the outstanding
   * count have to be one atomic step against the receive tasks. */
  size_t lines = 0;
  for (size_t i = 0; i < len; i++) {
    if (data[i] == '\n') {
      lines++;
    }
  }
  if (lines == 0) {
    // partial line, count it once so the lease cannot leak
    lines = 1;
  }
  if (xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
    return false;
  }
  if (_owner != GrblBridgeOwner::none && _owner != GrblBridgeOwner::web) {
    xSemaphoreGive(_stateMutex);
    esp3d_log_e("GRBL bridge refused web command, remote owns the bus");
    return false;
  }
  _takeLease(GrblBridgeOwner::web);
  _outstanding = (uint16_t)(_outstanding + lines);
  _ownerLastMs = millis();
  xSemaphoreGive(_stateMutex);
  if (_cncPush(data, len) == len) {
    return true;
  }
  /* Partial or refused push: those lines never reached the bus, so undo the
   * count. Leaving it inflated would hold the lease until the 5s fault
   * timeout and lock the web out of its own machine. */
  if (xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    if (_outstanding >= lines) {
      _outstanding = (uint16_t)(_outstanding - lines);
    } else {
      _outstanding = 0;
    }
    xSemaphoreGive(_stateMutex);
  }
  return false;
}

/* ------------------------------------------------------------------ pump */

/* A HardwareSerial write can come up short. Put the unsent tail back at the
 * head of the queue instead of dropping it, so a transient back pressure
 * stall never silently truncates a GRBL line. */
void ESP3DGrblBridge::_cncRestore(const uint8_t *data, size_t len) {
  if (xSemaphoreTake(_cncTxMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
    _cncOverflow += len;
    esp3d_log_e("GRBL bridge cannot restore %d CNC bytes", len);
    return;
  }
  if (_cncTxUsed + len > GRBL_BRIDGE_CNC_RING) {
    _cncOverflow += len;
    esp3d_log_e("GRBL bridge restore overflow, %d bytes lost", len);
  } else {
    memmove(_cncTx + len, _cncTx, _cncTxUsed);
    memcpy(_cncTx, data, len);
    _cncTxUsed += len;
  }
  xSemaphoreGive(_cncTxMutex);
}

void ESP3DGrblBridge::_remoteRestore(const uint8_t *data, size_t len) {
  if (xSemaphoreTake(_remoteTxMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
    _remoteOverflow += len;
    esp3d_log_e("GRBL bridge cannot restore %d remote bytes", len);
    return;
  }
  if (_remoteTxUsed + len > GRBL_BRIDGE_REMOTE_RING) {
    _remoteOverflow += len;
    esp3d_log_e("GRBL bridge restore overflow, %d bytes lost", len);
  } else {
    memmove(_remoteTx + len, _remoteTx, _remoteTxUsed);
    memcpy(_remoteTx, data, len);
    _remoteTxUsed += len;
  }
  xSemaphoreGive(_remoteTxMutex);
}

void ESP3DGrblBridge::_pumpCncTx() {
  if (!esp3d_serial_service.started()) {
    return;
  }
  uint8_t index = esp3d_serial_service.serialIndex();
  uint8_t scratch[64];
  for (size_t i = 0; i < GRBL_BRIDGE_PUMP_MAX; i += sizeof(scratch)) {
    size_t n = _cncPop(scratch, sizeof(scratch));
    if (!n) {
      return;
    }
    size_t got = Serials[index]->write(scratch, n);
    if (got != n) {
      _cncRestore(scratch + got, n - got);
      esp3d_log_e("GRBL bridge short CNC write %d/%d, %d requeued", got, n,
                  n - got);
      return;
    }
  }
}

void ESP3DGrblBridge::_pumpRemoteTx() {
  if (!serial_bridge_service.started()) {
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
      esp3d_log_e("GRBL bridge short remote write %d/%d, %d requeued", got, n,
                  n - got);
      return;
    }
  }
}

void ESP3DGrblBridge::handle() {
  if (!_active) {
    return;
  }
  _flushIdleRemoteLine();
  /* Evaluate and act on the lease in one critical section, so a receive
   * task cannot take ownership between the test and the release. */
  if (xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    if (_outstanding == 0 && !_alarm) {
      /* The bus is free, but hold the lease for a moment before handing it
       * over. A streaming sender has zero outstanding lines for a few
       * milliseconds between "ok" for line N and line N+1 arriving over the
       * network, and releasing on zero alone would let the other sender
       * splice a command into the middle of a running job. */
      if (_owner == GrblBridgeOwner::none) {
        _busQuiet = false;
      } else if (!_busQuiet) {
        _busQuiet = true;
        _busQuietMs = millis();
      } else if ((millis() - _busQuietMs) >= GRBL_BRIDGE_HANDOFF_HOLD_MS) {
        _releaseLease();
        _busQuiet = false;
      }
    } else {
      _busQuiet = false;
      if (_owner != GrblBridgeOwner::none &&
          (millis() - _ownerLastMs) > GRBL_BRIDGE_LEASE_TIMEOUT_MS) {
        /* GRBL went quiet without acknowledging, an alarm or a dropped
         * link. Force the lease open so neither sender stays locked out
         * forever. */
        esp3d_log_e("GRBL bridge lease timeout, releasing bus");
        _releaseLease();
      }
    }
    xSemaphoreGive(_stateMutex);
  }
  _pumpCncTx();
  _pumpRemoteTx();
}

#endif  // GRBL_BRIDGE_FEATURE
