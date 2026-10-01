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

/* GRBL_BRIDGE_FEATURE must be opted into explicitly in configuration.h.
 * It is deliberately NOT auto enabled by esp3d_config.h: the bridge changes
 * the meaning of the serial port and must never silently activate on a
 * configuration that was never reviewed for it. */
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

/* Normal command bytes in transit to the CNC.
 * Admission is all or nothing, so this must comfortably hold at least one
 * maximum length line plus whatever a sender streams ahead of the acks. */
#define GRBL_BRIDGE_CNC_RING 2048

/* CNC -> remote mirror. Absorbs status report bursts plus a response line. */
#define GRBL_BRIDGE_REMOTE_RING 1024

/* Separate small queue carrying only realtime bytes, so a feed hold or a
 * reset can never sit behind a large backlog of normal traffic. */
#define GRBL_BRIDGE_CNC_PRIO_RING 64

/* Longest normal command accepted from either sender. Anything longer is
 * rejected outright rather than split, because splitting would turn one
 * command into several malformed ones. */
#define GRBL_BRIDGE_LINE_MAX 512

/* Bytes of normal traffic we are willing to have queued ahead of the wire.
 * Admitting beyond this is refused so the normal path cannot build a backlog
 * that the realtime path would have to queue behind. */
#define GRBL_BRIDGE_CNC_HIGH_WATER 1024

/* A sender that stops making progress for this long latches a fault. It does
 * NOT hand the bus to the other sender: a long move legitimately produces no
 * "ok" for a long time, and opening the bus there would splice another
 * sender into a running cut. Recovery is explicit. */
#define GRBL_BRIDGE_ACK_TIMEOUT_MS 15000

/* How long to wait for the controller's post-resync proof after a soft reset
 * before latching a fault. */
#define GRBL_BRIDGE_RESYNC_TIMEOUT_MS 3000

/* A handover is only accepted on a status report this fresh. Without it the
 * controller might be mid move and the silence would be the only evidence,
 * which is exactly the reasoning that made the previous timeout release
 * unsafe. */
#define GRBL_BRIDGE_STATUS_FRESH_MS 2000

/* Upper bound on bytes moved per handle() call so network tasks are not
 * starved by a saturated link. */
#define GRBL_BRIDGE_PUMP_MAX 256

enum class GrblBridgeOwner : uint8_t {
  none = 0,
  remote = 1,
  web = 2,
};

/* Latched transport or accounting failure. While set, normal commands are
 * refused from both senders and only realtime bytes still pass, so a feed
 * hold or reset always remains available. Cleared explicitly by the
 * operator, never by a timer. */
enum class GrblBridgeFault : uint8_t {
  none = 0,
  ack_timeout = 1,     // a command was never acknowledged
  resync_timeout = 2,  // no status report after a reset
  rx_overflow = 3,     // bytes were lost on the mirror side
  queue_rejected = 4,  // a command did not fit and was refused
  oversized_cmd = 5,   // a sender sent a line longer than LINE_MAX
  bad_accounting = 6,  // an ack arrived with nothing outstanding
};

const char *grblBridgeOwnerName(GrblBridgeOwner o);
const char *grblBridgeFaultName(GrblBridgeFault f);

class ESP3DGrblBridge final {
 public:
  bool begin();
  void end();
  // Called from the Arduino loop. Pumps both directions, tracks controller
  // state and maintains the fault latch. Never blocks on a UART write.
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
  // producer shares one ordered transmit path. Always reports the bytes as
  // accepted, because a partial line is held until its terminator arrives;
  // refusals are visible through status(), [ESP420] and the web badge.
  bool enqueueWebCnc(const uint8_t *data, size_t len);

  /* Explicit ownership handover. Refused, with a reason, unless it is safe:
   * no latched fault, nothing outstanding, a fresh status report saying the
   * controller is idle. There is no automatic handover and no timeout based
   * release, because neither can distinguish "finished" from "mid cut". */
  bool requestOwner(GrblBridgeOwner who, const char **why);
  bool clearFault();

  /* The single authoritative GRBL realtime classifier. Callers deciding
   * "is this a safe pass through command" must use this rather than
   * esp3d_string::isRealTimeCommand(), which is gated on the runtime
   * firmware target setting and can therefore disagree with what the bridge
   * would actually forward.
   *
   * Note that $X is deliberately NOT realtime: it is a normal line, so it
   * obeys ownership and its "ok" is accounted for. */
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

  // Snapshot. Takes _stateMutex, so a caller on any task gets a coherent
  // view rather than a racy single word read.
  struct Status {
    bool active;
    bool faulted;
    GrblBridgeFault fault;
    GrblBridgeOwner owner;
    uint16_t outstanding;
    bool alarm;
    bool resyncing;
    bool sawStatus;
    bool inMotion;
    bool hold;
    uint32_t refused;
    uint32_t queueRejected;
    uint32_t rxLost;
    uint32_t badAcks;
  };
  Status status() const;

  // True when a web originated normal command may be admitted. Takes the
  // state mutex itself, so it is not derived from status().
  bool webTxAllowed() const;

 private:
  /* Locking.
   *
   * _stateMutex guards the whole ownership/fault/accounting block, both CNC
   * transmit rings, and all three line assembly buffers. It is the only
   * lock on the CNC path, which is what makes admission and acknowledgement
   * counting a single transaction: the check, the copy and the increment all
   * happen without the lock ever being released in between.
   *
   * _remoteTxMutex guards only the CNC to remote mirror ring, and is never
   * held while taking _stateMutex. That direction matters: the mirror is
   * written from the UART0 task, and if it reached back for _stateMutex
   * while another task held _stateMutex and wanted the mirror lock, the two
   * would deadlock. Instead an overflow records a pending fault here and
   * handle() promotes it, keeping the lock hierarchy strictly one way.
   *
   * Nothing takes a lock while holding another. In particular no UART write
   * is ever performed under a lock.
   */
  mutable SemaphoreHandle_t _stateMutex;
  SemaphoreHandle_t _remoteTxMutex;

  bool _active;
  GrblBridgeOwner _owner;
  GrblBridgeFault _fault;
  uint16_t _outstanding;
  bool _alarm;
  bool _resyncing;
  bool _sawStatus;
  bool _inMotion;
  bool _hold;
  uint32_t _resyncSinceMs;
  uint32_t _statusAtMs;
  uint32_t _lastProgressMs;
  // Written by whichever task loses bytes on the mirror path, promoted into
  // _fault by handle(). A diagnostic counter, so a lost increment on a rare
  // collision only under-reports; the latch is what actually stops traffic.
  volatile uint32_t _pendingMirrorLoss;

  uint32_t _refused;
  uint32_t _queueRejected;
  uint32_t _rxLost;
  uint32_t _badAcks;

  // Partial normal command being assembled for the CNC. Held separately for
  // each sender so a line split across several writes is counted once, at
  // its terminator, rather than once per write.
  uint8_t _remoteLine[GRBL_BRIDGE_LINE_MAX];
  size_t _remoteLineLen;
  bool _remoteDiscard;
  bool _remoteSawCR;

  uint8_t _webLine[GRBL_BRIDGE_LINE_MAX];
  size_t _webLineLen;
  bool _webDiscard;
  bool _webSawCR;

  // Partial CNC response line, for acknowledgement and state accounting.
  // One byte longer than the maximum so the terminator can be stored and the
  // line can be NUL terminated for inspection without writing out of bounds.
  uint8_t _resp[GRBL_BRIDGE_LINE_MAX + 1];
  size_t _respLen;
  bool _respDiscard;

  uint8_t *_cncTx;    // normal traffic
  uint8_t *_remoteTx; // CNC -> remote mirror
  uint8_t *_cncPrio;  // realtime, drained first
  size_t _cncTxUsed;
  size_t _remoteTxUsed;
  size_t _cncPrioUsed;

  // All of the following require _stateMutex to be held by the caller.
  void _setFaultLocked(GrblBridgeFault f, const char *detail);
  void _enterResyncLocked(const char *why);
  void _flushCncQueuesLocked();
  void _handleRealtimeLocked(uint8_t c);

  // All or nothing queue admission. Returns false and copies nothing when
  // the request does not fit, so a command is never truncated on the wire.
  bool _cncPushLocked(const uint8_t *data, size_t len);
  // Realtime always gets through, on the priority ring, unless the priority
  // ring itself is full, which is a fault rather than a silent drop.
  bool _cncPushRealtimeLocked(uint8_t c);

  // Assemble bytes from one sender and admit each completed line. The caller
  // must hold _stateMutex for the whole call.
  void _feedSenderLocked(const uint8_t *data, size_t len, bool fromWeb);

  void _parseCncLineLocked();
  void _drainMirrorLossLocked();

  void _pumpCncTx();
  void _pumpRemoteTx();

  size_t _remotePop(uint8_t *out, size_t max);
  void _remoteMirror(const uint8_t *data, size_t len);
  void _remoteRestore(const uint8_t *data, size_t len);
};

extern ESP3DGrblBridge grbl_bridge;

#endif  // GRBL_BRIDGE_FEATURE
#endif  // _GRBL_BRIDGE_H
