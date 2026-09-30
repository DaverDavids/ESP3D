/*
  grbl_bridge.h - GRBL aware two sender serial arbiter

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
#ifndef _GRBL_BRIDGE_H
#define _GRBL_BRIDGE_H

#include "../../include/esp3d_config.h"

// GRBL_BRIDGE_FEATURE is defined in esp3d_config.h, which is included above.
#ifdef GRBL_BRIDGE_FEATURE

#include <Arduino.h>

/* Fail the build rather than silently fall back to board defaults. A left
 * over -1 would let the build succeed while the two UARTs quietly land on
 * the wrong pads, which is the failure mode that is hardest to diagnose on
 * the bench. */
static_assert(ESP_RX_PIN != -1 && ESP_TX_PIN != -1,
              "GRBL bridge: set ESP_RX_PIN/ESP_TX_PIN in configuration.h");
static_assert(ESP_BRIDGE_RX_PIN != -1 && ESP_BRIDGE_TX_PIN != -1,
              "GRBL bridge: set ESP_BRIDGE_RX_PIN/ESP_BRIDGE_TX_PIN in "
              "configuration.h");
static_assert(!(ESP_RX_PIN == ESP_BRIDGE_RX_PIN ||
                ESP_RX_PIN == ESP_BRIDGE_TX_PIN ||
                ESP_TX_PIN == ESP_BRIDGE_RX_PIN ||
                ESP_TX_PIN == ESP_BRIDGE_TX_PIN),
              "GRBL bridge: CNC and remote UARTs must not share a pin");
static_assert(ESP_SERIAL_OUTPUT != ESP_SERIAL_BRIDGE_OUTPUT,
              "GRBL bridge: the CNC and the remote cannot be the same UART");

/* Byte capacity of the two transit buffers.
 * CNC direction must absorb a full GRBL status report burst plus a
 * response line without blocking the UART0 receive task.
 */
#define GRBL_BRIDGE_CNC_RING 2048
#define GRBL_BRIDGE_REMOTE_RING 1024

/* Longest partial line held while waiting for '\n'. */
#define GRBL_BRIDGE_LINE_MAX 512

/* Force-release the lease if GRBL stops acknowledging for this long.
 * Only applies when a sender has gone silent mid line (alarm, lost ok).
 * A healthy sender releases the lease after GRBL_BRIDGE_HANDOFF_HOLD_MS
 * of quiet, so this timeout is never seen during normal streaming.
 */
#define GRBL_BRIDGE_LEASE_TIMEOUT_MS 5000

/* How long the lease is held after the last outstanding line is
 * acknowledged, before the bus is offered to the other sender.
 *
 * This is the one setting that trades off the two failure modes:
 *   too short -> a streamed job momentarily has zero outstanding lines
 *               between "ok" for line N and line N+1 arriving over WiFi.
 *               If that gap is wide enough, the other sender can take the
 *               bus and splice a line into the middle of a running job.
 *   too long  -> after a job ends, the other sender waits this long to
 *               take over, which is dead time for the operator.
 *
 * It does NOT add any delay to the sender that currently owns the bus:
 * that sender is free to stream the whole time. It only delays handover.
 * Raise it if the offline pendant is allowed to interrupt a job; lower it
 * if you switch senders by hand and want a snappier takeover.
 */
#define GRBL_BRIDGE_HANDOFF_HOLD_MS 1500

/* Safety net for a remote that omits the terminating '\n' on a queued
 * command. A physical pendant emits whole lines atomically, so this
 * only fires for hand typed or non conforming senders. Realtime bytes
 * are never held back by this.
 */
#define GRBL_BRIDGE_IDLE_LINE_FLUSH_MS 250

/* Upper bound on bytes moved per handle() call so network tasks are not
 * starved by a saturated link. */
#define GRBL_BRIDGE_PUMP_MAX 256

enum class GrblBridgeOwner : uint8_t {
  none = 0,
  remote = 1,
  web = 2,
};

class ESP3DGrblBridge final {
 public:
  bool begin();
  void end();
  // Called from the Arduino loop. Pumps both directions and maintains the
  // lease. Never blocks.
  void handle();

  // Raw CNC -> remote mirror plus GRBL acknowledgement accounting.
  // Called from the UART0 receive task with the bytes as read, before any
  // line splitting or filtering, so ESP3D and the remote see the same
  // stream. Must not touch the UART0 receive buffer.
  void onCncBytes(const uint8_t *data, size_t len);

  // Remote -> CNC. Called from the UART1 receive task. Sole consumer of the
  // remote UART: bytes are never handed to the ESP3D command pipeline, so
  // no [ESPxxx] interception and no newline rewriting can occur.
  void onRemoteBytes(const uint8_t *data, size_t len);

  // Web / gcode host -> CNC. Replaces the direct UART write so that every
  // producer shares one ordered transmit path. Returns false when the
  // remote currently owns the bus.
  bool enqueueWebCnc(const uint8_t *data, size_t len);

  /* The single authoritative GRBL realtime classifier. Callers that need
   * to decide "is this a safe pass through command" must use this rather
   * than esp3d_string::isRealTimeCommand(), which is gated on the runtime
   * firmware target setting. Using the two inconsistently would let a gate
   * refuse a status poll that the bridge itself would happily forward. */
  static bool isRealtime(const uint8_t *data, size_t len) {
    if (!data || len != 1) {
      return false;
    }
    uint8_t c = data[0];
    if (c == '?' || c == '!' || c == '~' || c == 0x18) {
      return true;
    }
    return c >= 0x80 && c <= 0xA4;
  }
  bool active() const { return _active; }
  GrblBridgeOwner owner() const { return _owner; }
  uint16_t outstanding() const { return _outstanding; }
  bool alarm() const { return _alarm; }
  // Unsynchronised single word reads, used only for display and for the
  // early pre-check in the dispatch path. These are atomic on RV32 for
  // naturally aligned words, and a stale read is harmless: enqueueWebCnc()
  // repeats the check under _stateMutex and is the real gate.
  bool webTxAllowed() const { return _owner != GrblBridgeOwner::remote; }
  // Set when the remote tried to send a queued class command while the web
  // held the lease. Surfaced through the [ESP420] status block.
  uint32_t remoteDropped() const { return _remoteDropped; }
  uint32_t cncOverflow() const { return _cncOverflow; }
  uint32_t remoteOverflow() const { return _remoteOverflow; }

 private:
  bool _active;
  GrblBridgeOwner _owner;
  uint16_t _outstanding;
  bool _alarm;
  uint32_t _ownerLastMs;
  /* Set when the outstanding count falls to zero. The lease is held for
   * GRBL_BRIDGE_HANDOFF_HOLD_MS from this point before the bus is offered
   * to the other sender, which covers the gap between one "ok" and the
   * next streamed line arriving. */
  bool _busQuiet;
  uint32_t _busQuietMs;
  /* Set when the remote sent M2/M30, meaning the program from its SD card
   * has finished and the bus can be handed over as soon as the last line is
   * acknowledged, without waiting out GRBL_BRIDGE_HANDOFF_HOLD_MS. */
  bool _programEnded;
  uint32_t _remoteDropped;
  uint32_t _cncOverflow;
  uint32_t _remoteOverflow;

  // partial remote line, +1 so a full length line can still be terminated
  uint8_t _remoteLine[GRBL_BRIDGE_LINE_MAX + 1];
  size_t _remoteLineLen;
  uint32_t _remoteLastByteMs;

  // partial CNC response line, used only for acknowledgement accounting
  uint8_t _resp[GRBL_BRIDGE_LINE_MAX];
  size_t _respLen;

  uint8_t *_cncTx;
  uint8_t *_remoteTx;
  size_t _cncTxUsed;
  size_t _remoteTxUsed;
  SemaphoreHandle_t _cncTxMutex;
  SemaphoreHandle_t _remoteTxMutex;
  /* Guards _owner/_outstanding/_alarm/_ownerLastMs and the counters.
   * The loop task (handle, enqueueWebCnc) and both UART receive tasks
   * (onCncBytes, onRemoteBytes) touch this state concurrently, so the
   * lease decision and the outstanding increment must be atomic together.
   * Lock order is always _stateMutex then a ring mutex, never the reverse. */
  /* Guards _remoteLine/_remoteLineLen. Assembly runs on the UART1 receive
   * task, but the idle line flush runs on the loop task, so both sides need
   * to be serialised or a partial line can be memcpy'd while it is still
   * being appended to. */
  SemaphoreHandle_t _lineMutex;
  SemaphoreHandle_t _stateMutex;

  void _pumpCncTx();
  void _pumpRemoteTx();
  void _releaseLease();
  void _takeLease(GrblBridgeOwner who);
  void _commitRemoteLine(bool terminated);
  void _flushIdleRemoteLine();
  void _parseCncLine();
  void _noteRealtime(uint8_t c);
  size_t _cncPush(const uint8_t *data, size_t len);
  size_t _remotePush(const uint8_t *data, size_t len);
  size_t _cncPop(uint8_t *out, size_t max);
  size_t _remotePop(uint8_t *out, size_t max);
  void _cncRestore(const uint8_t *data, size_t len);
  void _remoteRestore(const uint8_t *data, size_t len);
};

extern ESP3DGrblBridge grbl_bridge;

#endif  // GRBL_BRIDGE_FEATURE
#endif  // _GRBL_BRIDGE_H
