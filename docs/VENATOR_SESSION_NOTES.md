# Accepted VENATOR setup — October 1, 2026

The user approved the current UI with “looks good” and requested that this state be remembered. Preserve these preferences in future work:

- Map at the top, open on refresh. Preserve the planner.
- Simple flight-mode dropdown with a single Apply mode button, including selectable Offboard. Avoid duplicate Offboard buttons.
- Applying a mode must not automatically arm/disarm. Arm, Motor Test and Disarm remain separate.
- Indoor telemetry tools: attitude, heading, downward range, connection check, capture/clear baseline, telemetry export, PX4 health messages.
- Current scope is indoor props-off bench demonstrations; outdoor GPS commands will be developed later.
- Keep wording concise, understandable and free of repeated warnings. Keep actual vehicle-state gating and normal PX4 checks; do not force-arm or invent position.

Hardware: Pixhawk 6X, PX4 1.17.0; SiK radio at 57600 through TELEM2. COM4 was confirmed locally but must remain selectable, not assumed. Downward Benewake sensor described by user as “tf mini luna” is wired to TELEM3; exact model and driver setup remain unverified. No valid downward range has been confirmed live.

Current development runners: localhost:5037 website, localhost:5557 bridge. Normal bridge default remains 5555. Only one application may own the serial port; QGroundControl can cause COM4 access denied. Dead serial sessions can now be released/reconnected. These runners are temporary and may not survive app/reboot.

Offboard Apply mode starts a continuous zero-body-rate/zero-thrust SET_ATTITUDE_TARGET bench stream before requesting PX4 Offboard. It verifies mode by heartbeat, leaves arm/disarm separate and navigation locked. This is not indoor flight or hover support. Other modes: Stabilized, Altitude, Position, Hold, Return. Onboard mode/arming requirements still apply. Manual modes previously rejected arming because manual controller input was absent.

Validation: 35 hardware-free bridge tests passed, browser script syntax checked; latest C# ModeSelector build passed. Real SiK telemetry reconnected. The user previously saw normal arm/disarm motor rotation. Persistent Offboard Apply and subsequent physical Offboard arming have NOT been validated. No physical mode-change or motor commands were sent during the latest dropdown update. Existing C# xUnit restore had TLS errors; telemetry export download was not independently verified.

Implementation: python_bridge/drone_bridge.py, python_bridge/offboard_bench.py, python_bridge/test_drone_bridge.py; C# web Program/DTOs/shared DroneClient; existing wwwroot/index.html. Preserve unrelated local desktop_app and radio_test edits.

Future work: inspect these local files first, verify live telemetry before actions, test Offboard rejection/link-loss/cleanup further, confirm sensor model/configuration, and validate outdoor GPS operations separately. A prior props-off confirmation is historical, not evidence about a future physical setup.
