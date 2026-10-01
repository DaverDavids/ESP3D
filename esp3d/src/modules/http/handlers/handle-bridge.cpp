/*
  handle-bridge.cpp - GRBL bus arbitration state for the web UI indicator

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
#include "../../../include/esp3d_config.h"

#if defined(HTTP_FEATURE) && defined(GRBL_BRIDGE_FEATURE)

#include "../http_server.h"
#if defined(ARDUINO_ARCH_ESP32)
#include <WebServer.h>
#endif  // ARDUINO_ARCH_ESP32
#if defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WebServer.h>
#endif  // ARDUINO_ARCH_ESP8266
#include "../../grbl_bridge/grbl_bridge.h"
#include "../../authentication/authentication_service.h"

/* Tiny plain text state probe polled by the badge script that is injected
 * into the embedded page. Kept deliberately minimal: one word, no JSON, no
 * parsing, so the injected script stays a few lines.
 *
 * States, and what the operator is expected to do about each:
 *   remote  pendant owns the CNC, web commands are refused
 *   web     WebUI owns the CNC, pendant commands are refused
 *   fault   something was lost or miscounted, nothing normal passes
 *   alarm   controller reported <Alarm|..>, needs $X from whoever owns it
 *   hold    controller reported a Hold, motion suspended
 *   resync  a reset happened, waiting for the controller to prove it is back
 *   none    no sender owns the CNC, so both are refused
 *   off     the bridge did not come up, arbitration is not running
 * Anything the bridge cannot classify is reported as "unknown" rather than
 * being folded into "none", so a fault or a new state can never be
 * displayed as "go ahead". The injected script shows every state except
 * "web" as locked out, and only "web" as green. */
void HTTP_Server::handle_bridge_status() {
  // Same guard as the other handlers. The page is only ever shown after a
  // successful login, so a 401 here simply leaves the badge hidden.
  if (AuthenticationService::getAuthenticatedLevel() ==
      ESP3DAuthenticationLevel::guest) {
    set_http_headers();
    _webserver->send(401, "text/plain", "unknown");
    return;
  }
  set_http_headers();
  const char *state = "off";
  ESP3DGrblBridge::Status s = grbl_bridge.status();
  if (s.active) {
    if (s.faulted) {
      state = "fault";
    } else if (s.alarm) {
      state = "alarm";
    } else if (s.hold) {
      state = "hold";
    } else if (s.resyncing) {
      state = "resync";
    } else {
      switch (s.owner) {
        case GrblBridgeOwner::remote:
          state = "remote";
          break;
        case GrblBridgeOwner::web:
          state = "web";
          break;
        default:
          state = "none";
          break;
      }
    }
  }
  _webserver->send(200, "text/plain", state);
}

#endif  // HTTP_FEATURE && GRBL_BRIDGE_FEATURE
