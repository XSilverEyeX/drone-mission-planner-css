"""Bounded props-off Offboard bench sequence. Never requests positive thrust."""
import threading
import time
from drone_bridge import M, response


def run(b, req):
    state = b.status()
    if req.get('propsOffConfirmed') is not True or not state['connected'] or state['armed'] or state['landedState'] != M.MAV_LANDED_STATE_ON_GROUND or not b.fresh('ATTITUDE'):
        return response('error', 'Props-off confirmation, fresh attitude and disarmed/on-ground telemetry required')
    b.enabled = False
    original_mode = b.fresh('HEARTBEAT').custom_mode
    stopped = threading.Event()
    stream_error = []
    stream_started = time.monotonic()

    def stream():
        try:
            while not stopped.is_set() and time.monotonic()-stream_started < 25:
                with b.wire:
                    # Ignore quaternion; use zero body rates and zero thrust. No position target.
                    b.master.mav.set_attitude_target_send(0, b.system, b.component,
                        M.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE, [1, 0, 0, 0], 0, 0, 0, 0)
                stopped.wait(.05)
        except Exception as exc:
            stream_error.append(str(exc))

    def observed(predicate, seconds=2):
        deadline = time.monotonic()+seconds
        with b.condition:
            while time.monotonic() < deadline:
                current = b.status()
                if not current['connected'] or stream_error:
                    return False
                if predicate(current):
                    return True
                b.condition.wait(.05)
        return False

    worker = threading.Thread(target=stream, daemon=True)
    worker.start()
    attempted_arm = False
    outcome = response('error', 'Offboard bench sequence incomplete')
    try:
        # PX4 requires more than one second of setpoints before entering Offboard.
        stopped.wait(1.2)
        if stream_error or not observed(lambda s: not s['armed'] and s['landedState'] == 1):
            raise ValueError('Setpoint stream or ground telemetry failed; no arm sent')
        mode_reply = b.acknowledged(M.MAV_CMD_DO_SET_MODE,
            lambda: b.send_long(M.MAV_CMD_DO_SET_MODE, [M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6, 0]), require_enabled=False)
        if mode_reply['status'] != 'ok' or not observed(lambda s: (b.fresh('HEARTBEAT').custom_mode >> 16) & 255 == 6):
            raise ValueError('PX4 did not confirm Offboard; no arm sent: '+mode_reply['message'])
        if not observed(lambda s: not s['armed'] and s['landedState'] == 1):
            raise ValueError('Ground state changed; no arm sent')
        attempted_arm = True
        arm_reply = b.acknowledged(M.MAV_CMD_COMPONENT_ARM_DISARM,
            lambda: b.send_long(M.MAV_CMD_COMPONENT_ARM_DISARM, [1, 0]), require_enabled=False)
        if arm_reply['status'] == 'ok' and observed(lambda s: s['armed'] and s['landedState'] == 1):
            # Brief observation only. Do not refresh arming or retry rejected commands.
            deadline = time.monotonic()+1
            while time.monotonic() < deadline:
                current = b.status()
                if not current['connected'] or current['landedState'] != 1 or stream_error:
                    break
                stopped.wait(.05)
            outcome = response('ok', 'Offboard armed state observed; cleaning up with normal disarm')
        else:
            outcome = response('error', 'Offboard arming rejected or not observed', arm_reply)
    except Exception as exc:
        outcome = response('error', 'Offboard bench failed: '+str(exc))
    finally:
        current = b.status()
        safe = current['connected'] and not current['armed']
        if attempted_arm and current['connected'] and current['landedState'] == 1:
            try:
                b.acknowledged(M.MAV_CMD_COMPONENT_ARM_DISARM,
                    lambda: b.send_long(M.MAV_CMD_COMPONENT_ARM_DISARM, [0, 0]), require_enabled=False)
                safe = observed(lambda s: not s['armed'])
            except Exception:
                safe = False
        if not safe:
            outcome.update(status='error', message='Disarm/state not confirmed. Check vehicle immediately; Offboard stream will stop and PX4 link-loss behavior applies.')
        elif b.fresh('HEARTBEAT') and ((b.fresh('HEARTBEAT').custom_mode >> 16) & 255) == 6:
            try:
                restored = b.acknowledged(M.MAV_CMD_DO_SET_MODE,
                    lambda: b.send_long(M.MAV_CMD_DO_SET_MODE, [M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                        (original_mode >> 16) & 255, (original_mode >> 24) & 255]), require_enabled=False)
                if restored['status'] != 'ok':
                    outcome['message'] += '; previous mode restoration was not accepted'
            except Exception:
                outcome['message'] += '; previous mode restoration unconfirmed'
        stopped.set()
        worker.join(timeout=1)
        if safe:
            outcome['message'] += '; vehicle confirmed disarmed'
    return outcome
