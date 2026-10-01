/*
 ESP430.cpp - ESP3D command class

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

#include "../../modules/grbl_bridge/grbl_bridge.h"
#include "../esp3d_commands.h"

/* GRBL two sender bus ownership.
 *
 * This command exists because ownership can no longer be derived from
 * silence. A pendant that stops talking for a few seconds is
 * indistinguishable from a pendant in the middle of a long cut, so the only
 * safe handover is an explicit request from a human, accepted only when
 * nothing is in flight and the controller reports that it is not moving.
 *
 *   [ESP430]                                  report the current state
 *   [ESP430] owner=web                        grant the WebUI the bus
 *   [ESP430] owner=remote                     grant the pendant the bus
 *   [ESP430] owner=none                       hand the bus to nobody
 *
 * Requiring admin is deliberate: whoever can take the bus can start a cut
 * from the pendant, so a guest session must not be able to take it away from
 * the operator standing at the machine.
 */
#define COMMAND_ID 430
void ESP3DCommands::ESP430(int cmd_params_pos, ESP3DMessage *msg) {
  ESP3DClientType target = msg->origin;
  msg->target = target;
  msg->origin = ESP3DClientType::command;
  bool json = hasTag(msg, cmd_params_pos, "json");
  String owner = get_param(msg, cmd_params_pos, "owner=");
  if (owner.length()) {
    owner.toLowerCase();
    owner.trim();
  }
  bool hasError = false;
  String error_msg = "Invalid parameters";
  String ok_msg = "";

#if defined(AUTHENTICATION_FEATURE)
  if (msg->authentication_level != ESP3DAuthenticationLevel::admin) {
    msg->authentication_level = ESP3DAuthenticationLevel::not_authenticated;
    dispatchAuthenticationError(msg, COMMAND_ID, json);
    return;
  }
#endif  // AUTHENTICATION_FEATURE

  ESP3DGrblBridge::Status s = grbl_bridge.status();

  if (!s.active) {
    hasError = true;
    error_msg = "GRBL bridge is not active";
  } else if (!owner.length()) {
    // Pure query, so a caller can read the state before deciding.
    ok_msg = String("owner=") + grblBridgeOwnerName(s.owner);
    ok_msg += ", inflight=" + String(static_cast<int>(s.outstanding));
    ok_msg += ", webtx=";
    ok_msg += grbl_bridge.webTxAllowed() ? "ok" : "LOCKED";
    if (s.resyncing) {
      ok_msg += ", resync";
    }
    if (s.hold) {
      ok_msg += ", hold";
    }
    if (s.inMotion) {
      ok_msg += ", motion";
    }
    if (s.alarm) {
      ok_msg += ", ALARM";
    }
    if (s.faulted) {
      ok_msg += String(", FAULT=") + grblBridgeFaultName(s.fault);
      ok_msg += ", clear with [ESP431]";
    }
  } else {
    GrblBridgeOwner want = GrblBridgeOwner::none;
    if (owner == "web") {
      want = GrblBridgeOwner::web;
    } else if (owner == "remote" || owner == "pendant") {
      want = GrblBridgeOwner::remote;
    } else if (owner == "none" || owner == "idle") {
      want = GrblBridgeOwner::none;
    } else {
      hasError = true;
      error_msg = "owner must be web, remote or none";
    }
    if (!hasError) {
      const char *why = nullptr;
      if (!grbl_bridge.requestOwner(want, &why)) {
        hasError = true;
        error_msg = why ? why : "ownership refused";
        esp3d_log_e("ESP430 refused: %s", error_msg.c_str());
      } else {
        esp3d_log("ESP430 owner is now %s", grblBridgeOwnerName(want));
        ok_msg = String("owner=") + grblBridgeOwnerName(want);
        if (want == GrblBridgeOwner::none) {
          ok_msg += ", both senders are now refused";
        }
      }
    }
  }

  if (!dispatchAnswer(msg, COMMAND_ID, json, hasError,
                      hasError ? error_msg.c_str() : ok_msg.c_str())) {
    esp3d_log_e("Error sending response to clients");
  }
}
#endif  // GRBL_BRIDGE_FEATURE
