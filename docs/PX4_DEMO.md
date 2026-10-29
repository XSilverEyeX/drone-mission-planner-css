# VENATOR PX4 demo integration

Changes were made to the local modified project, not a GitHub checkout. Existing desktop_app edits and radio_test/mavlink_check.py were left untouched.

## Start the updated version

Keep props OFF for initial bench tests. Leave the vehicle disarmed and independently safe before stopping the old bridge/web terminals. Existing running processes continue using the old code until restarted.

From C:\Users\cordo\Documents\VENATOR\drone-mission-planner-css:

```powershell
python .\python_bridge\drone_bridge.py
```

In a second terminal:

```powershell
cd C:\Users\cordo\Documents\VENATOR\drone-mission-planner-css\csharp
dotnet run --project DroneNavigation.Web
```

Open the localhost address printed by the web server. Refresh the page. Select the detected SiK port (COM4 on the currently confirmed setup), select 57600 baud, and click Connect. No port is automatically selected or opened by the bridge. Only one application should own the radio port.

## Read-only bench check

1. Leave the operator checkbox unchecked and demo controls locked. Connect and verify actual vehicle system/component IDs, fresh heartbeat, PX4 mode, battery, GPS, estimator status, altitude, and attitude.
2. With no valid GPS/position indoors, navigation must remain disabled. Unknown/missing telemetry is displayed as Unknown rather than simulated values.
3. The map centers on the first valid position. The planning origin comes from PX4 home when already available, otherwise the first valid live position; that fallback does not change the autopilot's home. PX4 home is shown independently and is required for Go's home-relative altitude.
4. Verify Disconnect/reconnect while the vehicle is confirmed disarmed. Disconnect is rejected when armed or heartbeat state is unknown. If the link cannot recover, independently make the vehicle safe before restarting the bridge.
5. Keep props off. Do not use takeoff, Go, or yaw as a bench test; these require confirmed in-air state and an appropriate flight setup.

## Incremental command checks

Use PX4 SITL or another controlled simulation before live airborne testing. Flight commands were not sent to the physical vehicle during implementation.

- Explicitly check the operator confirmation and unlock only when all telemetry/preflight gates pass. Missing pre-arm checks cannot be overridden by VENATOR. No force-arm/disarm or parameter changes are sent.
- Arm separately. An accepted COMMAND_ACK does not mean armed: wait for the heartbeat armed flag. On-ground Disarm is provided for bench recovery, with no forced disarm.
- Takeoff requests 1–5 m above current ground AMSL. PX4 receives NAV_TAKEOFF with an absolute AMSL target, not an ArduPilot GUIDED command. Check the configured PX4 takeoff minimums and observed altitude in simulation; onboard settings can constrain behavior.
- Wait for observed in-air state. Hold requests PX4 AUTO.LOITER. Select a map destination, review its coordinates, then click Go. Go is a direct route, not automatic obstacle avoidance or execution of the planner path. Targets are limited to 30 m from PX4 home and 1–5 m above home.
- Yaw requests a heading clockwise from North while keeping current position/altitude, using DO_REPOSITION with yaw in radians. Verify the actual heading in telemetry. Heading 360 is normalized to 0.
- Land and Abort request NAV_LAND at the current location. Abort locks normal controls and can interrupt an outstanding navigation ACK wait. It does not kill motors or guarantee landing if the radio fails. Manual takeover and onboard PX4 failsafes remain necessary.
- A lost final ACK has unknown outcome: no automatic command retry. Normal controls lock, and the operator must inspect vehicle state before re-enabling.

The planner remains available as a preview; the previous unchecked automatic Fly sequence is disabled on both client and server. Planner coordinates are relative to the live planning origin, and a new telemetry session resets planner/target state.

## Implementation and validation

Changed: Python bridge, shared C# DroneClient, web API and request DTOs, and the existing Leaflet page. Added hardware-free bridge regression tests.

Passed:
- Python syntax compilation.
- 21 bridge tests: gates, source filtering, ACK acceptance/rejection/progress/timeout, command units/frames, takeoff AMSL, yaw radians, demo radius, landed-only disarm, reboot reset, emergency landing during pending navigation, and loopback JSON protocol.
- Web build in bin/IntegrationCheck: zero warnings/errors. The ordinary build output was locked by the user's already-running web process; it was not stopped.
- 8 end-to-end API checks against a synthetic vehicle on bridge port 5556 and web port 5036, including preserved path planning, command gating, and cross-origin rejection.
- Browser telemetry display, explicit unlock, separate arm/takeoff/navigation gates, and browser script checks. Preview image uses synthetic telemetry, not the physical vehicle.

Not verified:
- Physical arm/takeoff/hold/Go/yaw/land. No hardware flight commands were transmitted.
- Existing C# xUnit suite: NuGet restore failed with TLS authentication / 'No credentials are available in the security package', even after network access was granted. The no-restore invocation did not actually discover tests and is not counted as a passing suite.
- Live GPS/preflight readiness, airframe configuration, minimum takeoff settings, and vehicle link-loss behavior must be checked on the actual vehicle and in PX4 simulation.

## Protocol references

- https://mavlink.io/en/messages/common.html#MAV_CMD_DO_REPOSITION (COMMAND_INT coordinates, frame-dependent altitude, yaw radians)
- https://mavlink.io/en/services/command.html (command acknowledgments)
- https://docs.px4.io/v1.17/en/msg_docs/VehicleCommand (takeoff AMSL)
- https://raw.githubusercontent.com/PX4/PX4-Autopilot/v1.17.0/src/modules/navigator/navigator_main.cpp (PX4 reposition handling)

The API and bridge remain local tools, not an authenticated remote ground-control service. The bridge binds loopback; web mutation endpoints reject cross-origin browser requests. The Bridge:Port setting exists for hardware-free integration tests and defaults to 5555.


## Indoor demo dashboard

There is one Run brief motor test button, enabled by the props-removed and secured confirmation. PX4 normal arming checks remain active. The bridge observes armed state and requests normal disarm; the confirmation resets after each test.

The indoor dashboard shows attitude, heading, and valid downward LiDAR distance. Check connection refreshes telemetry. Capture baseline records sensor values while disarmed; Clear baseline resets comparison. Save telemetry exports a local JSON snapshot. The planner and outdoor map remain collapsed for later GPS testing.

The earlier authorized physical test successfully armed and disarmed, and the user observed motor rotation. No additional physical motor commands were sent during this UI update. Live LiDAR data is currently unavailable; the synthetic preview uses a 1.25 m test reading.

Refresh http://localhost:5037/ for the updated UI. Validation: web build passed with zero warnings/errors; 27 hardware-free bridge tests passed. Browser checks passed for the single-button simulated motor sequence, confirmation reset, connection check, baseline comparison, and sensor display. The export button was exercised, but its browser download event timed out, so the saved file was not independently verified. The C# test-suite restore limitation above remains.
## Additional indoor controls

Map opens on refresh. Props-off Arm requests normal PX4 arming with explicit props-removed confirmation and fresh on-ground state, retaining onboard checks and leaving navigation locked. It remains armed until normal Disarm. The automatic bench sequence holds for two extra seconds after observing arming before normal disarm, unless telemetry or ground state changes. Takeoff is visible but keeps the existing position/preflight gates. Health messages can be shown directly. LiDAR is attached to TELEM3; PX4 driver configuration must publish valid downward DISTANCE_SENSOR data. No driver parameters were changed.

29 hardware-free tests and browser syntax checks passed. No physical commands were sent for this change. The live bridge had stale heartbeat, so it was not restarted; the new Python behaviors require a safe bridge restart before use.
