/*
 ESP431.cpp - ESP3D command class

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

/* Clear a latched GRBL bridge fault.
 *
 *   [ESP431]
 *
 * A fault means the bridge lost track: bytes were dropped, a command was
 * never acknowledged, an acknowledgement arrived with nothing outstanding,
 * or a reset never resynchronised. In every one of those cases the accounting
 * that decides who may talk to the CNC can no longer be trusted, so the
 * bridge refuses normal commands until a human looks at the machine and
 * says to continue. That is the whole reason the latch exists, and it is why
 * this is an explicit admin command rather than something that clears itself
 * after a pause.
 *
 * Clearing discards anything still queued and resets the in-flight count, so
 * the next command sent is treated as the first one.
 */
#define COMMAND_ID 431
void ESP3DCommands::ESP431(int cmd_params_pos, ESP3DMessage *msg) {
  ESP3DClientType target = msg->origin;
  msg->target = target;
  msg->origin = ESP3DClientType::command;
  bool json = hasTag(msg, cmd_params_pos, "json");
  bool hasError = false;
  String error_msg = "";
  String ok_msg = "";

#if defined(AUTHENTICATION_FEATURE)
  if (msg->authentication_level != ESP3DAuthenticationLevel::admin) {
    msg->authentication_level = ESP3DAuthenticationLevel::not_authenticated;
    dispatchAuthenticationError(msg, COMMAND_ID, json);
    return;
  }
#endif  // AUTHENTICATION_FEATURE

  ESP3DGrblBridge::Status before = grbl_bridge.status();
  if (!before.active) {
    hasError = true;
    error_msg = "GRBL bridge is not active";
  } else if (!before.faulted && !before.resyncing) {
    // Nothing to clear, but saying so is more useful than a bare ok.
    hasError = true;
    error_msg = "no fault is latched";
  } else {
    String what = before.faulted ? String(grblBridgeFaultName(before.fault))
                                 : String("resync");
    if (!grbl_bridge.clearFault()) {
      hasError = true;
      error_msg = "could not clear the fault, state busy";
      esp3d_log_e("ESP431 failed to clear fault %s", what.c_str());
    } else {
      esp3d_log_e("ESP431 cleared fault %s by operator", what.c_str());
      ok_msg = String("cleared ") + what;
      ok_msg += ", queue flushed, in-flight reset to 0";
    }
  }

  if (!dispatchAnswer(msg, COMMAND_ID, json, hasError,
                      hasError ? error_msg.c_str() : ok_msg.c_str())) {
    esp3d_log_e("Error sending response to clients");
  }
}
#endif  // GRBL_BRIDGE_FEATURE
