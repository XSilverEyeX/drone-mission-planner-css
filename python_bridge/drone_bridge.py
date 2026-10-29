"""VENATOR local JSON/TCP bridge. Starts disconnected; no automatic flight actions.

One receiver owns MAVLink reads. Flight commands are acknowledged, never treated
as completed maneuvers. PX4 retains all onboard arming checks and failsafes.
"""
import json
import math
import socketserver
import threading
import time
import uuid
from collections import deque

from pymavlink import mavutil
from serial.tools import list_ports

M = mavutil.mavlink
FRESH_SECONDS = 3.0
CONNECT_TIMEOUT_SECONDS = 10.0
MAX_ALTITUDE_M = 5.0
MAX_DISTANCE_M = 30.0


def response(status, message='', data=None):
    return dict(status=status, message=message, data=data)


def number(value, name, minimum, maximum):
    if isinstance(value, bool):
        raise ValueError(f'{name} must be a number')
    value = float(value)
    if not math.isfinite(value) or not minimum <= value <= maximum:
        raise ValueError(f'{name} must be between {minimum} and {maximum}')
    return value


def distance(a, b):
    lat1, lat2 = math.radians(a[0]), math.radians(b[0])
    dlat, dlon = lat2-lat1, math.radians(b[1]-a[1])
    h = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
    return 6378137 * 2 * math.asin(math.sqrt(min(1, h)))


class DroneBridge:
    def __init__(self):
        self.lock = threading.RLock()
        self.operation = threading.Lock()
        self.emergency = threading.Lock()
        self.wire = threading.Lock()
        self.condition = threading.Condition(self.lock)
        self.master = None
        self.worker = None
        self.stop = threading.Event()
        self.reset()

    def reset(self):
        self.session_id = uuid.uuid4().hex
        self.messages = {}
        self.distance_sources = {}
        self.distance_history = {}  # Short LiDAR history for rate/update calculations.
        self.acks = deque(maxlen=100)
        self.logs = deque(maxlen=15)
        self.enabled = False
        self.link_error = None
        self.offboard_stream = False
        self.port = None
        self.baud = None
        self.system = None
        self.component = None
        self.last_command = None
        self.reference = None
        self.reference_source = None

    def fresh(self, kind):
        item = self.messages.get(kind)
        return item[1] if item and time.monotonic()-item[0] < FRESH_SECONDS else None

    def ingest(self, msg):
        kind = msg.get_type()
        if kind == 'BAD_DATA':
            return
        with self.condition:
            if self.system is None:
                if kind != 'HEARTBEAT' or msg.autopilot != M.MAV_AUTOPILOT_PX4:
                    return
                if msg.type not in (M.MAV_TYPE_QUADROTOR, M.MAV_TYPE_HEXAROTOR,
                                    M.MAV_TYPE_OCTOROTOR, M.MAV_TYPE_TRICOPTER):
                    return
                self.system, self.component = msg.get_srcSystem(), msg.get_srcComponent()
            if (msg.get_srcSystem(), msg.get_srcComponent()) != (self.system, self.component):
                return
            now = time.monotonic()
            previous = self.messages.get('HEARTBEAT')
            if kind == 'HEARTBEAT' and previous and now-previous[0] >= FRESH_SECONDS:
                self.enabled = False
            if kind == 'GLOBAL_POSITION_INT':
                old = self.messages.get(kind)
                if old and hasattr(msg, 'time_boot_ms') and hasattr(old[1], 'time_boot_ms'):
                    if msg.time_boot_ms < old[1].time_boot_ms:
                        # Reboot/boot clock rollover: reacquire all safety inputs and origin.
                        self.messages.clear()
                        self.distance_sources.clear()
                        self.distance_history.clear()
                        self.acks.clear()
                        self.enabled = False
                        self.reference = None
                        self.reference_source = None
            self.messages[kind] = (now, msg)
            if kind == 'DISTANCE_SENSOR':
                # Save the newest reading and a short history for the LiDAR display.
                self.distance_sources[msg.id] = (now, msg)
                history = self.distance_history.setdefault(msg.id, deque(maxlen=30))
                history.append((now, msg.current_distance/100.0))
            if kind == 'COMMAND_ACK':
                if getattr(msg, 'target_system', 0) in (0, 255) and getattr(msg, 'target_component', 0) in (0, 190):
                    self.acks.append((now, msg))
            if kind == 'STATUSTEXT':
                self.logs.append(str(msg.text).rstrip('\x00'))
            self.condition.notify_all()

    def receive(self):
        last_heartbeat = 0
        try:
            while not self.stop.is_set():
                if time.monotonic()-last_heartbeat >= 1:
                    with self.wire:
                        self.master.mav.heartbeat_send(M.MAV_TYPE_GCS, M.MAV_AUTOPILOT_INVALID, 0, 0, M.MAV_STATE_ACTIVE)
                    last_heartbeat = time.monotonic()
                if self.offboard_stream:
                    if not self.fresh('HEARTBEAT'):
                        self.offboard_stream = False
                    else:
                        with self.wire:
                            self.master.mav.set_attitude_target_send(0, self.system, self.component,
                                M.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE, [1, 0, 0, 0], 0, 0, 0, 0)
                msg = self.master.recv_match(blocking=True, timeout=0.1)
                if msg:
                    self.ingest(msg)
                with self.lock:
                    if not self.fresh('HEARTBEAT'):
                        self.enabled = False
        except Exception as exc:
            with self.lock:
                self.link_error = str(exc)
                self.enabled = False

    def close(self):
        self.stop.set()
        if self.worker:
            self.worker.join(timeout=2)
        if self.master:
            self.master.close()
        if self.worker and self.worker.is_alive():
            self.worker.join(timeout=2)
        self.master = None
        self.worker = None
        with self.lock:
            self.reset()

    def connect(self, port, baud):
        if self.master:
            raise ValueError('Disconnect the current telemetry session first')
        if port not in [p.device for p in list_ports.comports()]:
            raise ValueError('Select a currently detected serial port')
        baud = int(number(baud, 'baud', 1200, 921600))
        try:
            self.master = mavutil.mavserial(device=port, baud=baud, source_system=255, source_component=190)
            self.port, self.baud = port, baud
            self.stop.clear()
            self.worker = threading.Thread(target=self.receive, daemon=True)
            self.worker.start()
            deadline = time.monotonic()+CONNECT_TIMEOUT_SECONDS
            with self.condition:
                while not self.fresh('HEARTBEAT') and not self.link_error:
                    remaining = deadline-time.monotonic()
                    if remaining <= 0:
                        raise TimeoutError('No PX4 multicopter heartbeat within 10 seconds')
                    self.condition.wait(remaining)
                if self.link_error:
                    raise IOError(self.link_error)
            # Request telemetry only: these commands never arm or change flight mode.
            for message_id, rate in [(1, 1), (24, 1), (33, 2), (30, 10), (245, 1), (242, 1), (230, 1), (132, 5)]:
                self.send_long(M.MAV_CMD_SET_MESSAGE_INTERVAL, [message_id, 1e6/rate])
            return response('ok', 'PX4 heartbeat confirmed; flight controls remain locked', self.status())
        except Exception:
            self.close()
            raise

    def status(self):
        with self.lock:
            hb, gps, pos = self.fresh('HEARTBEAT'), self.fresh('GPS_RAW_INT'), self.fresh('GLOBAL_POSITION_INT')
            sys, ext, att = self.fresh('SYS_STATUS'), self.fresh('EXTENDED_SYS_STATE'), self.fresh('ATTITUDE')
            estimator = self.fresh('ESTIMATOR_STATUS')
            # Find the newest downward-facing range sensor. This is the TFmini Plus
            # mounted under the drone for floor/altitude distance.
            now = time.monotonic()
            downward = [item for item in self.distance_sources.values()
                        if item[1].orientation == M.MAV_SENSOR_ROTATION_PITCH_270]
            latest_range = max(downward, key=lambda item: item[0]) if downward and hb else None
            range_time, latest_range_msg = latest_range if latest_range else (None, None)
            range_age = now-range_time if range_time is not None else None

            # Keep the PX4 min/max check only for the status label. The raw
            # LiDAR reading is still sent to the GUI even when it is outside
            # that range so we can see exactly what the sensor is reporting.
            range_in_bounds = bool(latest_range_msg and
                                   latest_range_msg.min_distance <= latest_range_msg.current_distance <= latest_range_msg.max_distance and
                                   latest_range_msg.current_distance > 0)

            # Extra read-only values used by the GUI. They do not affect PX4 control.
            range_rate = None
            range_update_rate = None
            range_signal_quality = None
            if latest_range_msg is not None:
                history = list(self.distance_history.get(latest_range_msg.id, ()))
                recent = [(t, d) for t, d in history if now-t <= 5.0]
                if len(recent) >= 2 and recent[-1][0] > recent[0][0]:
                    range_update_rate = (len(recent)-1)/(recent[-1][0]-recent[0][0])

                # Use the raw recent readings for the trend too. This means the
                # trend can still update when the LiDAR is below its rated minimum.
                rate_samples = [(t, d) for t, d in history if now-t <= 2.0 and d >= 0]
                if len(rate_samples) >= 2 and rate_samples[-1][0] > rate_samples[0][0]:
                    range_rate = ((rate_samples[-1][1]-rate_samples[0][1]) /
                                  (rate_samples[-1][0]-rate_samples[0][0]))

                quality = getattr(latest_range_msg, 'signal_quality', None)
                if isinstance(quality, (int, float)) and 1 <= quality <= 100:
                    range_signal_quality = int(quality)

            if latest_range_msg is None:
                range_status = 'no_sensor'
            elif range_age is not None and range_age >= FRESH_SECONDS:
                range_status = 'stale'
            elif not range_in_bounds:
                range_status = 'out_of_range'
            else:
                range_status = 'valid'
            required_flags = M.ESTIMATOR_ATTITUDE | M.ESTIMATOR_POS_HORIZ_ABS | M.ESTIMATOR_POS_VERT_ABS
            estimator_valid = bool(estimator and estimator.flags & required_flags == required_flags and
                                   not estimator.flags & (M.ESTIMATOR_GPS_GLITCH | M.ESTIMATOR_ACCEL_ERROR))
            connected = self.master is not None and hb is not None and not self.link_error
            if not connected:
                self.enabled = False
            armed = bool(hb and hb.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED)
            gps_valid = bool(gps and gps.fix_type >= 3 and gps.eph != 65535 and gps.eph <= 250)
            position_valid = bool(connected and gps_valid and estimator_valid and pos and -900000000 <= pos.lat <= 900000000 and -1800000000 <= pos.lon <= 1800000000)
            prearm = 1 << 28
            preflight = bool(sys and sys.onboard_control_sensors_present & prearm and sys.onboard_control_sensors_enabled & prearm and sys.onboard_control_sensors_health & prearm)
            if sys:
                unhealthy = sys.onboard_control_sensors_enabled & ~sys.onboard_control_sensors_health
                preflight = preflight and unhealthy == 0
            healthy_state = bool(hb and hb.system_status in (M.MAV_STATE_STANDBY, M.MAV_STATE_ACTIVE))
            home_item = self.messages.get('HOME_POSITION')
            if home_item and connected:
                h = home_item[1]
                if -900000000 <= h.latitude <= 900000000 and -1800000000 <= h.longitude <= 1800000000:
                    home = dict(lat=h.latitude/1e7, lon=h.longitude/1e7, altitudeAmsl=h.altitude/1000)
                    # The planning origin is frozen per connection. PX4 home can update independently.
                    if self.reference is None:
                        self.reference, self.reference_source = home, 'PX4 home'
                else:
                    home = None
            else:
                home = None
            if position_valid and self.reference is None:
                self.reference = dict(lat=pos.lat/1e7, lon=pos.lon/1e7, altitudeAmsl=pos.alt/1000)
                self.reference_source = 'First valid position (planning origin only)'
            reasons = []
            if not connected: reasons.append('Fresh PX4 heartbeat required')
            if not gps_valid: reasons.append('Fresh 3D GPS fix with HDOP <= 2.5 required')
            if not estimator_valid: reasons.append('Fresh valid PX4 absolute horizontal/vertical position estimate required')
            if not position_valid: reasons.append('Fresh global position required')
            if not preflight: reasons.append('PX4 pre-arm/sensor checks not confirmed healthy')
            if not healthy_state: reasons.append('PX4 state is not standby/active')
            if ext is None: reasons.append('Fresh landed state required')
            ready = not reasons
            can_fly = ready and self.enabled and armed and ext.landed_state == M.MAV_LANDED_STATE_IN_AIR
            age = time.monotonic()-self.messages['HEARTBEAT'][0] if 'HEARTBEAT' in self.messages else None
            result = dict(connected=connected, radioConnected=self.master is not None, sessionId=self.session_id, port=self.port, baud=self.baud,
                heartbeatAge=age, systemId=self.system, componentId=self.component, armed=armed,
                mode=mavutil.mode_string_v10(hb) if hb else 'Unknown', gpsValid=gps_valid, positionValid=position_valid, estimatorValid=estimator_valid,
                gpsFix=gps.fix_type if gps else None, satellites=gps.satellites_visible if gps and gps.satellites_visible != 255 else None,
                hdop=gps.eph/100 if gps and gps.eph != 65535 else None,
                preflightReady=ready, blockers=reasons, demoEnabled=self.enabled,
                canArm=ready and self.enabled and not armed and ext.landed_state == M.MAV_LANDED_STATE_ON_GROUND,
                canTakeoff=ready and self.enabled and armed and ext.landed_state == M.MAV_LANDED_STATE_ON_GROUND,
                canBenchTest=bool(connected and not armed and ext and ext.landed_state == M.MAV_LANDED_STATE_ON_GROUND),
                canNavigate=can_fly, canLand=connected and armed, canDisconnect=bool((connected and not armed) or self.link_error),
                landedState=ext.landed_state if ext else None, home=home, planningOrigin=self.reference,
                originSource=self.reference_source, batteryPercent=sys.battery_remaining if sys and sys.battery_remaining >= 0 else None,
                batteryVoltage=sys.voltage_battery/1000 if sys and sys.voltage_battery != 65535 else None,
                roll=math.degrees(att.roll) if att else None, pitch=math.degrees(att.pitch) if att else None,
                heading=(math.degrees(att.yaw)+360)%360 if att else None,
                # Always send the latest raw LiDAR value. The GUI uses the
                # status separately to show whether PX4 considers it in range.
                downwardDistance=latest_range_msg.current_distance/100 if latest_range_msg is not None else None,
                downwardStatus=range_status,
                downwardSensorId=latest_range_msg.id if latest_range_msg is not None else None,
                downwardMaxDistance=latest_range_msg.max_distance/100 if latest_range_msg is not None else None,
                downwardSignalQuality=range_signal_quality,
                downwardAge=range_age,
                downwardUpdateRate=range_update_rate,
                downwardRate=range_rate,
                statusText=list(self.logs), lastCommand=self.last_command, message=self.link_error,
                limits=dict(maxAltitude=MAX_ALTITUDE_M, maxDistance=MAX_DISTANCE_M))
            result.update(lat=pos.lat/1e7 if position_valid else None, lon=pos.lon/1e7 if position_valid else None,
                altitude=pos.relative_alt/1000 if position_valid else None, altitudeAmsl=pos.alt/1000 if position_valid else None)
            return result

    def send_long(self, command, params):
        with self.wire:
            self.master.mav.command_long_send(self.system, self.component, command, 0, *(params+[0]*(7-len(params))))

    def acknowledged(self, command, send, require_enabled=True):
        with self.lock:
            if require_enabled and not self.enabled:
                return response('error', 'Controls locked before command transmission')
            if not self.fresh('HEARTBEAT') or self.link_error:
                return response('error', 'Heartbeat lost before command transmission')
            started = time.monotonic()
            send()
        deadline = started+5
        with self.condition:
            while time.monotonic() < deadline:
                matches = [m for t, m in self.acks if t >= started and m.command == command]
                if matches:
                    ack = matches[-1]
                    if ack.result != M.MAV_RESULT_IN_PROGRESS:
                        label = M.enums['MAV_RESULT'][ack.result].name if ack.result in M.enums['MAV_RESULT'] else str(ack.result)
                        ok = ack.result == M.MAV_RESULT_ACCEPTED
                        self.last_command = dict(command=command, result=label, accepted=ok)
                        return response('ok' if ok else 'error', (f'{label}: PX4 accepted the request; verify vehicle state in telemetry' if ok else f'{label}: PX4 rejected the request; inspect system health messages'), self.last_command)
                if not self.fresh('HEARTBEAT') or self.link_error:
                    break
                self.condition.wait(min(0.1, max(0, deadline-time.monotonic())))
        self.enabled = False
        self.last_command = dict(command=command, accepted=None, result='UNKNOWN')
        return response('error', 'No final ACK: outcome unknown. Controls locked; inspect telemetry before retrying.', self.last_command)

    def reposition(self, lat, lon, amsl, yaw=float('nan')):
        def send():
            with self.wire:
                self.master.mav.command_int_send(self.system, self.component, M.MAV_FRAME_GLOBAL_INT,
                    M.MAV_CMD_DO_REPOSITION, 0, 0, 1.0, 1, 0, yaw, round(lat*1e7), round(lon*1e7), amsl)
        return self.acknowledged(M.MAV_CMD_DO_REPOSITION, send)

    def handle(self, req):
        cmd = req.get('cmd')
        if cmd == 'get_ports':
            return response('ok', data=dict(ports=[dict(device=p.device, description=p.description) for p in list_ports.comports()]))
        if cmd in ('get_status', 'get_position'):
            return response('ok', data=self.status())
        if cmd == 'enable_demo' and req.get('enabled') is False:
            with self.lock:
                self.enabled = False
            return response('ok', 'Demo controls locked')
        # Landing can interrupt an ordinary command ACK wait. It locks normal actions
        # before transmission, so an unsent navigation request cannot follow it.
        if cmd in ('land', 'abort'):
            if not self.emergency.acquire(blocking=False):
                return response('error', 'Landing request already in progress; inspect telemetry')
            try:
                with self.lock:
                    state = self.status()
                    if not state['connected'] or not state['armed']:
                        return response('error', 'Landing requires a fresh armed PX4 heartbeat')
                    self.enabled = False
                return self.acknowledged(M.MAV_CMD_NAV_LAND, lambda: self.send_long(M.MAV_CMD_NAV_LAND,
                    [0, 0, 0, float('nan'), float('nan'), float('nan'), float('nan')]), require_enabled=False)
            except Exception as exc:
                return response('error', f'Landing request failed; outcome may be unknown: {exc}')
            finally:
                self.emergency.release()
        # Never queue duplicate browser actions. Telemetry remains available while waiting for ACK.
        if not self.operation.acquire(blocking=False):
            return response('error', 'Another connection/command is in progress')
        try:
            if cmd == 'connect':
                return self.connect(req['port'], req['baud'])
            state = self.status()
            if cmd == 'disconnect':
                if self.master and not self.link_error and (not state['connected'] or state['armed']):
                    raise ValueError('Disconnect requires a fresh disarmed heartbeat; retain telemetry while armed or state unknown')
                self.close()
                return response('ok', 'Telemetry disconnected')
            if cmd == 'enable_demo':
                if req.get('enabled') is False:
                    self.enabled = False
                elif req.get('enabled') is True and req.get('confirmed') is True and state['preflightReady']:
                    self.enabled = True
                else:
                    raise ValueError('Explicit operator confirmation and all preflight gates required')
                return response('ok', 'Demo controls unlocked' if self.enabled else 'Demo controls locked')
            if not state['connected']:
                raise ValueError('Fresh PX4 heartbeat required')
            if cmd == 'set_mode':
                modes = {'stabilized': (7, 0), 'altitude': (2, 0), 'position': (3, 0), 'hold': (4, 3), 'return': (4, 5), 'offboard': (6, 0)}
                selected = req.get('mode')
                if selected not in modes:
                    raise ValueError('Unsupported flight mode')
                if state['armed'] or state['landedState'] != M.MAV_LANDED_STATE_ON_GROUND:
                    raise ValueError('Mode selection requires fresh disarmed/on-ground telemetry')
                if selected in ('position', 'hold', 'return') and not (state['positionValid'] and state['estimatorValid']):
                    raise ValueError('Selected mode requires valid position tracking')
                if selected == 'return' and not state['home']:
                    raise ValueError('Return requires a valid PX4 home')
                self.enabled = False
                main, sub = modes[selected]
                was_streaming = self.offboard_stream
                if selected == 'offboard':
                    if not self.fresh('ATTITUDE') or not self.worker or not self.worker.is_alive():
                        raise ValueError('Offboard requires fresh attitude and a running telemetry receiver')
                    self.offboard_stream = True
                    time.sleep(1.2)
                    if not self.status()['connected']:
                        self.offboard_stream = False
                        raise ValueError('Connection lost before Offboard request')
                reply = self.acknowledged(M.MAV_CMD_DO_SET_MODE,
                    lambda: self.send_long(M.MAV_CMD_DO_SET_MODE, [M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, main, sub]), require_enabled=False)
                if reply['status'] != 'ok':
                    self.offboard_stream = was_streaming
                    return reply
                deadline = time.monotonic()+2
                with self.condition:
                    while time.monotonic() < deadline:
                        hb = self.fresh('HEARTBEAT')
                        if hb and (hb.custom_mode >> 16) & 255 == main and (hb.custom_mode >> 24) & 255 == sub:
                            if selected != 'offboard':
                                self.offboard_stream = False
                            return response('ok', 'Flight mode confirmed: '+self.status()['mode'])
                        self.condition.wait(.1)
                return response('error', 'Mode request accepted, but selected mode not observed. Check PX4 health messages.')
            if cmd == 'offboard_bench':
                from offboard_bench import run
                return run(self, req)
            if cmd == 'actuator_bench_test':
                if req.get('propsOffConfirmed') is not True or state['armed'] or state['landedState'] != M.MAV_LANDED_STATE_ON_GROUND:
                    raise ValueError('Actuator test requires props removed confirmation and fresh disarmed/on-ground state')
                self.enabled = False
                return self.acknowledged(M.MAV_CMD_ACTUATOR_TEST,
                    lambda: self.send_long(M.MAV_CMD_ACTUATOR_TEST, [0.05, 1, 0, 0, M.ACTUATOR_OUTPUT_FUNCTION_MOTOR1, 0, 0]), require_enabled=False)
            if cmd == 'bench_arm':
                if req.get('propsOffConfirmed') is not True:
                    raise ValueError('Explicit props-removed and secured-vehicle confirmation required')
                if state['armed'] or state['landedState'] != M.MAV_LANDED_STATE_ON_GROUND:
                    raise ValueError('Bench test requires a fresh disarmed/on-ground state')
                # This separate motor bench test never unlocks flight/navigation.
                # PX4 evaluates normal arming checks. Param2 stays zero (no force).
                self.enabled = False
                arm_reply = self.acknowledged(M.MAV_CMD_COMPONENT_ARM_DISARM,
                    lambda: self.send_long(M.MAV_CMD_COMPONENT_ARM_DISARM, [1, 0]), require_enabled=False)
                if arm_reply.get('data') and arm_reply['data'].get('accepted') is False:
                    return arm_reply
                observed_armed = False
                deadline = time.monotonic()+2
                with self.condition:
                    while time.monotonic() < deadline:
                        observed = self.status()
                        if observed['connected'] and observed['armed']:
                            observed_armed = True
                            break
                        self.condition.wait(.1)
                if observed_armed:
                    hold_until = time.monotonic()+2
                    with self.condition:
                        while time.monotonic() < hold_until:
                            current = self.status()
                            if not current['connected'] or not current['armed'] or current['landedState'] != M.MAV_LANDED_STATE_ON_GROUND:
                                break
                            self.condition.wait(min(.1, max(0, hold_until-time.monotonic())))
                observed = self.status()
                if not observed['connected'] or observed['landedState'] != M.MAV_LANDED_STATE_ON_GROUND:
                    return response('error', 'Bench result uncertain: cannot confirm on-ground state for disarm. Check vehicle immediately.',
                        dict(arm=arm_reply, armedObserved=observed_armed))
                disarm_reply = self.acknowledged(M.MAV_CMD_COMPONENT_ARM_DISARM,
                    lambda: self.send_long(M.MAV_CMD_COMPONENT_ARM_DISARM, [0, 0]), require_enabled=False)
                deadline = time.monotonic()+2
                with self.condition:
                    while time.monotonic() < deadline:
                        observed = self.status()
                        if observed['connected'] and not observed['armed']:
                            break
                        self.condition.wait(.1)
                observed = self.status()
                safe = observed['connected'] and not observed['armed']
                return response('ok' if safe and observed_armed else 'error',
                    'Armed heartbeat observed; normal disarm sent and disarmed heartbeat confirmed.' if safe and observed_armed else
                    'No armed heartbeat observed; normal disarm sent and vehicle currently disarmed.' if safe else
                    'Disarm not confirmed. Check vehicle immediately.',
                    dict(arm=arm_reply, disarm=disarm_reply, armedObserved=observed_armed, disarmedConfirmed=safe))
            if cmd == 'disarm':
                if state['landedState'] != M.MAV_LANDED_STATE_ON_GROUND:
                    raise ValueError('Disarm only allowed with fresh on-ground state')
                self.enabled = False
                return self.acknowledged(M.MAV_CMD_COMPONENT_ARM_DISARM, lambda: self.send_long(M.MAV_CMD_COMPONENT_ARM_DISARM, [0, 0]), require_enabled=False)
            if cmd == 'arm' and req.get('propsOffConfirmed') is True:
                if state['armed'] or state['landedState'] != M.MAV_LANDED_STATE_ON_GROUND:
                    raise ValueError('Props-off arm requires a fresh disarmed/on-ground state')
                self.enabled = False
                return self.acknowledged(M.MAV_CMD_COMPONENT_ARM_DISARM, lambda: self.send_long(M.MAV_CMD_COMPONENT_ARM_DISARM, [1, 0]), require_enabled=False)
            if cmd == 'arm':
                if not state['canArm']: raise ValueError('Arm blocked: verify preflight, on-ground state, and demo unlock')
                return self.acknowledged(M.MAV_CMD_COMPONENT_ARM_DISARM, lambda: self.send_long(M.MAV_CMD_COMPONENT_ARM_DISARM, [1, 0]))
            if cmd == 'takeoff':
                if not state['canTakeoff']: raise ValueError('Takeoff requires confirmed armed/on-ground state and all preflight gates')
                alt = number(req['altitude'], 'altitude', 1, MAX_ALTITUDE_M)
                # NAV_TAKEOFF altitude is AMSL. Use current ground AMSL, never a fixed home altitude.
                return self.acknowledged(M.MAV_CMD_NAV_TAKEOFF, lambda: self.send_long(M.MAV_CMD_NAV_TAKEOFF, [0, 0, 0, float('nan'), float('nan'), float('nan'), state['altitudeAmsl']+alt]))
            if cmd in ('hold', 'goto', 'yaw'):
                if not state['canNavigate']: raise ValueError('Navigation requires fresh valid telemetry, preflight gates, demo unlock, and confirmed in-air state')
                if cmd == 'hold':
                    # PX4 AUTO.LOITER via DO_SET_MODE: custom main 4, submode 3.
                    return self.acknowledged(M.MAV_CMD_DO_SET_MODE, lambda: self.send_long(M.MAV_CMD_DO_SET_MODE, [M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 3]))
                lat, lon, amsl = state['lat'], state['lon'], state['altitudeAmsl']
                yaw = float('nan')
                if cmd == 'goto':
                    lat, lon = number(req['lat'], 'latitude', -90, 90), number(req['lon'], 'longitude', -180, 180)
                    alt = number(req['altitude'], 'altitude above PX4 home', 1, MAX_ALTITUDE_M)
                    if not state['home']: raise ValueError('PX4 HOME_POSITION required for home-relative target altitude')
                    amsl = state['home']['altitudeAmsl']+alt
                else:
                    # DO_REPOSITION yaw is radians (NAV_TAKEOFF yaw uses degrees).
                    yaw = math.radians(number(req['heading'], 'heading', 0, 360) % 360)
                origin = state['home'] or state['planningOrigin']
                if distance((origin['lat'], origin['lon']), (lat, lon)) > MAX_DISTANCE_M:
                    raise ValueError(f'Target exceeds {MAX_DISTANCE_M} m demo radius')
                if not 0.5 <= amsl-origin['altitudeAmsl'] <= MAX_ALTITUDE_M:
                    raise ValueError('Target altitude exceeds low-altitude demo envelope')
                return self.reposition(lat, lon, amsl, yaw)
            raise ValueError(f'Unsupported command: {cmd}')
        except (ValueError, KeyError, TypeError, OSError) as exc:
            return response('error', str(exc))
        except Exception as exc:
            self.enabled = False
            return response('error', f'Command failed; controls locked: {exc}')
        finally:
            self.operation.release()


bridge = DroneBridge()


class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        self.request.settimeout(30)
        while True:
            try:
                line = self.rfile.readline(65537)
                if not line: return
                if len(line) > 65536: return
                req = json.loads(line)
                result = bridge.handle(req) if isinstance(req, dict) else response('error', 'JSON object required')
            except (json.JSONDecodeError, ValueError) as exc:
                result = response('error', str(exc))
            except (TimeoutError, ConnectionError, OSError):
                return
            self.wfile.write((json.dumps(result, allow_nan=False)+'\n').encode())
            self.wfile.flush()


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


if __name__ == '__main__':
    print('[bridge] Listening on 127.0.0.1:5555; select telemetry port in VENATOR. Props off for bench tests.')
    try:
        with Server(('127.0.0.1', 5555), Handler) as server:
            server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.close()