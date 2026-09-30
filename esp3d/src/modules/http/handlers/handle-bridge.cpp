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
 * Anything that is not exactly "remote" is treated by the script as
 * "client commands allowed", so a 404 or an error here fails safe and
 * simply leaves the indicator hidden. */
void HTTP_Server::handle_bridge_status() {
  // Same guard as the other handlers. The page is only ever shown after a
  // successful login, so a 401 here simply leaves the badge hidden.
  if (AuthenticationService::getAuthenticatedLevel() ==
      ESP3DAuthenticationLevel::guest) {
    set_http_headers();
    _webserver->send(401, "text/plain", "free");
    return;
  }
  set_http_headers();
  const char *state = "free";
  if (grbl_bridge.active()) {
    switch (grbl_bridge.owner()) {
      case GrblBridgeOwner::remote:
        state = grbl_bridge.alarm() ? "alarm" : "remote";
        break;
      case GrblBridgeOwner::web:
        state = "web";
        break;
      default:
        state = "free";
        break;
    }
  }
  _webserver->send(200, "text/plain", state);
}

#endif  // HTTP_FEATURE && GRBL_BRIDGE_FEATURE
