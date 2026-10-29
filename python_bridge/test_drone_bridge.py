"""Hardware-free safety and PX4 packet regression tests."""
import json
import math
import socket
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import drone_bridge as db

M = db.M

class Message(SimpleNamespace):
    def get_type(self): return self.kind
    def get_srcSystem(self): return getattr(self, 'source_system', 1)
    def get_srcComponent(self): return getattr(self, 'source_component', 1)

class Wire:
    def __init__(self, bridge):
        self.bridge = bridge
        self.sent = []
        self.result = M.MAV_RESULT_ACCEPTED
        self.no_ack = set()
    def command_long_send(self, *args):
        self.sent.append(('long', args))
        self.ack(args[2])
    def command_int_send(self, *args):
        self.sent.append(('int', args))
        self.ack(args[3])
    def ack(self, command):
        if command not in self.no_ack:
            self.bridge.ingest(Message(kind='COMMAND_ACK', command=command, result=self.result, target_system=255, target_component=190))


def fixture(armed=False, airborne=False):
    b = db.DroneBridge()
    b.master = SimpleNamespace(mav=Wire(b), close=lambda: None)
    b.port, b.baud = 'TEST_ONLY', 57600
    messages = [
        Message(kind='HEARTBEAT', autopilot=M.MAV_AUTOPILOT_PX4, type=M.MAV_TYPE_QUADROTOR,
                base_mode=M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | (M.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0),
                custom_mode=(4 << 16) | (3 << 24), system_status=M.MAV_STATE_ACTIVE if armed else M.MAV_STATE_STANDBY),
        Message(kind='GPS_RAW_INT', fix_type=3, eph=90, satellites_visible=12),
        Message(kind='GLOBAL_POSITION_INT', lat=410000000, lon=-870000000, alt=205000 if airborne else 203000, relative_alt=2000 if airborne else 0),
        Message(kind='HOME_POSITION', latitude=410000000, longitude=-870000000, altitude=203000),
        Message(kind='ESTIMATOR_STATUS', flags=M.ESTIMATOR_ATTITUDE | M.ESTIMATOR_POS_HORIZ_ABS | M.ESTIMATOR_POS_VERT_ABS),
        Message(kind='SYS_STATUS', onboard_control_sensors_present=1<<28, onboard_control_sensors_enabled=1<<28,
                onboard_control_sensors_health=1<<28, battery_remaining=90, voltage_battery=16000),
        Message(kind='EXTENDED_SYS_STATE', landed_state=M.MAV_LANDED_STATE_IN_AIR if airborne else M.MAV_LANDED_STATE_ON_GROUND),
    ]
    for msg in messages: b.ingest(msg)
    return b

class SafetyTests(unittest.TestCase):
    def test_startup_and_read_only_never_arm(self):
        b = db.DroneBridge()
        self.assertFalse(b.status()['connected'])
        self.assertEqual(b.handle({'cmd':'arm'})['status'], 'error')
        b = fixture()
        for _ in range(10): b.handle({'cmd':'get_status'})
        self.assertEqual(b.master.mav.sent, [])
        self.assertFalse(b.status()['demoEnabled'])

    def test_confirmation_and_unknown_preflight_fail_closed(self):
        b = fixture()
        self.assertTrue(b.status()['preflightReady'])
        self.assertEqual(b.handle({'cmd':'enable_demo', 'enabled':True, 'confirmed':False})['status'], 'error')
        del b.messages['SYS_STATUS']
        self.assertFalse(b.status()['preflightReady'])
        self.assertEqual(b.handle({'cmd':'enable_demo', 'enabled':True, 'confirmed':True})['status'], 'error')

    def test_invalid_gps_and_estimator_block_navigation(self):
        b = fixture(True, True)
        b.enabled = True
        b.messages['GPS_RAW_INT'][1].fix_type = 2
        self.assertFalse(b.status()['canNavigate'])
        b.messages['GPS_RAW_INT'][1].fix_type = 3
        b.messages['ESTIMATOR_STATUS'][1].flags |= M.ESTIMATOR_GPS_GLITCH
        self.assertFalse(b.status()['positionValid'])
        self.assertEqual(b.handle({'cmd':'goto','lat':41,'lon':-87,'altitude':2})['status'], 'error')
        self.assertEqual(b.master.mav.sent, [])

    def test_stale_telemetry_blocks_arm(self):
        b = fixture()
        b.enabled = True
        t, m = b.messages['HEARTBEAT']
        b.messages['HEARTBEAT'] = (t-10, m)
        self.assertFalse(b.status()['canArm'])
        self.assertEqual(b.handle({'cmd':'arm'})['status'], 'error')
        self.assertEqual(b.master.mav.sent, [])

    def test_foreign_messages_and_acks_are_ignored(self):
        b = fixture()
        b.ingest(Message(kind='GLOBAL_POSITION_INT', source_system=2, lat=0, lon=0))
        self.assertEqual(b.status()['lat'], 41)
        b.ingest(Message(kind='COMMAND_ACK', command=400, result=0, target_system=42))
        self.assertEqual(len(b.acks), 0)

    def test_arm_does_not_force_or_assume_state(self):
        b = fixture()
        b.enabled = True
        reply = b.handle({'cmd':'arm'})
        self.assertEqual(reply['status'], 'ok')
        args = b.master.mav.sent[0][1]
        self.assertEqual(args[2], M.MAV_CMD_COMPONENT_ARM_DISARM)
        self.assertEqual(args[4:6], (1,0))
        self.assertFalse(b.status()['armed'])
        self.assertFalse(b.status()['canTakeoff'])

    def test_takeoff_amsl_and_user_altitude(self):
        b = fixture(True)
        b.enabled = True
        self.assertEqual(b.handle({'cmd':'takeoff','altitude':2.5})['status'], 'ok')
        args = b.master.mav.sent[-1][1]
        self.assertEqual(args[2], M.MAV_CMD_NAV_TAKEOFF)
        self.assertEqual(args[-1], 205.5)
        self.assertTrue(math.isnan(args[8]))
        self.assertEqual(b.handle({'cmd':'takeoff','altitude':float('nan')})['status'], 'error')
        self.assertEqual(b.handle({'cmd':'takeoff','altitude':10})['status'], 'error')

    def test_goto_uses_px4_command_int_amsl_and_radius(self):
        b = fixture(True, True)
        b.enabled = True
        self.assertEqual(b.handle({'cmd':'goto','lat':41.00005,'lon':-87,'altitude':2})['status'], 'ok')
        args = b.master.mav.sent[-1][1]
        self.assertEqual(args[2:4], (M.MAV_FRAME_GLOBAL_INT, M.MAV_CMD_DO_REPOSITION))
        self.assertEqual(args[7], 1)
        self.assertEqual(args[-3:], (410000500, -870000000, 205))
        count = len(b.master.mav.sent)
        self.assertEqual(b.handle({'cmd':'goto','lat':42,'lon':-87,'altitude':2})['status'], 'error')
        self.assertEqual(len(b.master.mav.sent), count)

    def test_yaw_is_radians_and_keeps_current_position(self):
        b = fixture(True, True)
        b.enabled = True
        self.assertEqual(b.handle({'cmd':'yaw','heading':90})['status'], 'ok')
        args = b.master.mav.sent[-1][1]
        self.assertAlmostEqual(args[9], math.pi/2)
        self.assertEqual(args[-3:], (410000000, -870000000, 205))

    def test_hold_is_px4_auto_loiter(self):
        b = fixture(True, True)
        b.enabled = True
        self.assertEqual(b.handle({'cmd':'hold'})['status'], 'ok')
        args = b.master.mav.sent[-1][1]
        self.assertEqual(args[2], M.MAV_CMD_DO_SET_MODE)
        self.assertEqual(args[4:7], (M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 3))

    def test_land_works_when_gps_and_normal_controls_fail(self):
        b = fixture(True, True)
        del b.messages['GPS_RAW_INT']
        self.assertEqual(b.handle({'cmd':'abort'})['status'], 'ok')
        self.assertEqual(b.master.mav.sent[-1][1][2], M.MAV_CMD_NAV_LAND)
        self.assertFalse(b.enabled)
        self.assertTrue(b.status()['armed'])
        self.assertEqual(b.handle({'cmd':'disarm'})['status'], 'error')

    def test_abort_interrupts_navigation_ack_wait(self):
        b = fixture(True, True)
        b.enabled = True
        b.master.mav.no_ack.add(M.MAV_CMD_DO_REPOSITION)
        result = []
        worker = threading.Thread(target=lambda: result.append(b.handle({'cmd':'yaw','heading':90})))
        worker.start()
        deadline = time.monotonic()+1
        while not b.master.mav.sent and time.monotonic() < deadline: time.sleep(.005)
        self.assertTrue(b.operation.locked())
        started = time.monotonic()
        self.assertEqual(b.handle({'cmd':'abort'})['status'], 'ok')
        self.assertLess(time.monotonic()-started, 1)
        b.ingest(Message(kind='COMMAND_ACK', command=M.MAV_CMD_DO_REPOSITION, result=0, target_system=255, target_component=190))
        worker.join(timeout=1)
        self.assertFalse(worker.is_alive())
        self.assertFalse(b.enabled)

    def test_command_rejection_is_visible(self):
        b = fixture()
        b.enabled = True
        b.master.mav.result = M.MAV_RESULT_DENIED
        self.assertEqual(b.handle({'cmd':'arm'})['status'], 'error')
        self.assertEqual(b.last_command['result'], 'MAV_RESULT_DENIED')

    def test_missing_home_blocks_home_relative_navigation(self):
        b = fixture(True, True)
        b.enabled = True
        del b.messages['HOME_POSITION']
        self.assertEqual(b.handle({'cmd':'goto','lat':41,'lon':-87,'altitude':2})['status'], 'error')
        self.assertEqual(b.master.mav.sent, [])

    def test_disconnect_requires_fresh_disarmed_state(self):
        b = fixture(True)
        self.assertEqual(b.handle({'cmd':'disconnect'})['status'], 'error')
        b = fixture()
        self.assertEqual(b.handle({'cmd':'disconnect'})['status'], 'ok')
        self.assertIsNone(b.master)
        self.assertIsNone(b.reference)

    def test_tcp_protocol_no_hardware(self):
        b = fixture()
        with patch.object(db, 'bridge', b), db.Server(('127.0.0.1', 0), db.Handler) as server:
            worker = threading.Thread(target=server.serve_forever, daemon=True)
            worker.start()
            with socket.create_connection(server.server_address, timeout=2) as client:
                f = client.makefile('rwb')
                f.write(b'{"cmd":"get_status"}\n'); f.flush()
                result = json.loads(f.readline())
                self.assertEqual(result['status'], 'ok')
                self.assertTrue(result['data']['connected'])
                f.write(b'not json\n'); f.flush()
                self.assertEqual(json.loads(f.readline())['status'], 'error')
            server.shutdown()
            worker.join(timeout=2)
            self.assertEqual(b.master.mav.sent, [])



class RecoveryTests(unittest.TestCase):
    def test_reboot_invalidates_safety_state_and_origin(self):
        b=fixture(True,True)
        b.enabled=True
        b.status()
        b.messages['GLOBAL_POSITION_INT'][1].time_boot_ms=10000
        b.ingest(Message(kind='GLOBAL_POSITION_INT',time_boot_ms=10,lat=410000000,lon=-870000000,alt=203000,relative_alt=0))
        self.assertFalse(b.enabled)
        self.assertFalse(b.status()['preflightReady'])
        self.assertIsNone(b.reference)
        self.assertNotIn('HOME_POSITION',b.messages)

    def test_ack_timeout_locks_controls_and_does_not_retry(self):
        b=fixture()
        b.enabled=True
        b.master.mav.no_ack.add(M.MAV_CMD_COMPONENT_ARM_DISARM)
        reply=b.handle({'cmd':'arm'})
        self.assertEqual(reply['status'],'error')
        self.assertIn('outcome unknown',reply['message'])
        self.assertFalse(b.enabled)
        self.assertEqual(len(b.master.mav.sent),1)

    def test_progress_ack_is_not_final_acceptance(self):
        b=fixture()
        b.enabled=True
        b.master.mav.result=M.MAV_RESULT_IN_PROGRESS
        def finish():
            time.sleep(.05)
            b.ingest(Message(kind='COMMAND_ACK',command=400,result=M.MAV_RESULT_ACCEPTED,target_system=255,target_component=190))
        worker=threading.Thread(target=finish)
        worker.start()
        reply=b.handle({'cmd':'arm'})
        worker.join()
        self.assertEqual(reply['status'],'ok')
        self.assertFalse(b.status()['armed'])




class ConnectionTests(unittest.TestCase):
    def test_gui_selected_port_only_and_telemetry_only_startup(self):
        b=db.DroneBridge()
        seed=fixture()
        incoming=[m for _,m in seed.messages.values()]
        closed=[]
        def receive(**kwargs):
            if incoming: return incoming.pop(0)
            time.sleep(.005)
            return None
        wire=Wire(b)
        wire.heartbeat_send=lambda *args: None
        serial=SimpleNamespace(mav=wire,recv_match=receive,close=lambda:closed.append(True))
        with patch.object(db.list_ports,'comports',return_value=[SimpleNamespace(device='COM8',description='Test')]), patch.object(db.mavutil,'mavserial',return_value=serial) as open_serial:
            self.assertEqual(b.handle({'cmd':'connect','port':'COM4','baud':57600})['status'],'error')
            open_serial.assert_not_called()
            self.assertEqual(b.handle({'cmd':'connect','port':'COM8','baud':115200})['status'],'ok')
            open_serial.assert_called_once_with(device='COM8',baud=115200,source_system=255,source_component=190)
            self.assertTrue(all(args[2]==M.MAV_CMD_SET_MESSAGE_INTERVAL for _,args in wire.sent))
            self.assertFalse(b.enabled)
            self.assertEqual(b.handle({'cmd':'disconnect'})['status'],'ok')
        self.assertTrue(closed)

    def test_no_px4_heartbeat_cleans_up_and_allows_new_attempt(self):
        b=db.DroneBridge()
        closed=[]
        wire=Wire(b)
        wire.heartbeat_send=lambda *args: None
        def receive(**kwargs):
            time.sleep(.005)
            return Message(kind='HEARTBEAT',autopilot=M.MAV_AUTOPILOT_INVALID,type=M.MAV_TYPE_GCS)
        serial=SimpleNamespace(mav=wire,recv_match=receive,close=lambda:closed.append(True))
        with patch.object(db,'CONNECT_TIMEOUT_SECONDS',.03), patch.object(db.list_ports,'comports',return_value=[SimpleNamespace(device='COM8')]), patch.object(db.mavutil,'mavserial',return_value=serial):
            self.assertEqual(b.handle({'cmd':'connect','port':'COM8','baud':57600})['status'],'error')
        self.assertIsNone(b.master)
        self.assertIsNone(b.worker)
        self.assertFalse(b.enabled)
        self.assertTrue(closed)




class BenchTests(unittest.TestCase):
    def test_offboard_apply_does_not_arm_or_disarm(self):
        b=fixture()
        b.worker=SimpleNamespace(is_alive=lambda:True)
        b.ingest(Message(kind='ATTITUDE',roll=0,pitch=0,yaw=0))
        wire=b.master.mav
        original=wire.command_long_send
        def send(*args):
            original(*args)
            hb=b.messages['HEARTBEAT'][1]
            hb.custom_mode=(int(args[5])<<16)|(int(args[6])<<24)
            b.ingest(hb)
        wire.command_long_send=send
        with patch.object(db.time,'sleep'):
            reply=b.handle({'cmd':'set_mode','mode':'offboard'})
        self.assertEqual(reply['status'],'ok',reply)
        self.assertTrue(b.offboard_stream)
        self.assertFalse(b.status()['armed'])
        self.assertTrue(all(a[2]==M.MAV_CMD_DO_SET_MODE for _,a in wire.sent))


    def test_mode_selection_requires_disarmed_ground_and_position(self):
        b=fixture(armed=True)
        self.assertEqual(b.handle({'cmd':'set_mode','mode':'stabilized'})['status'],'error')
        b=fixture()
        b.messages.pop('GLOBAL_POSITION_INT')
        self.assertEqual(b.handle({'cmd':'set_mode','mode':'hold'})['status'],'error')
        self.assertEqual(b.handle({'cmd':'set_mode','mode':'offboard'})['status'],'error')
        self.assertEqual(b.master.mav.sent,[])

    def test_mode_selection_verifies_observed_heartbeat(self):
        b=fixture()
        wire=b.master.mav
        original=wire.command_long_send
        def send(*args):
            original(*args)
            hb=b.messages['HEARTBEAT'][1]
            hb.custom_mode=(int(args[5])<<16)|(int(args[6])<<24)
            b.ingest(hb)
        wire.command_long_send=send
        self.assertEqual(b.handle({'cmd':'set_mode','mode':'stabilized'})['status'],'ok')
        self.assertEqual(wire.sent[0][1][4:7],(M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,7,0))
        self.assertFalse(b.enabled)


    def test_offboard_confirmation_and_attitude_required(self):
        b=fixture()
        self.assertEqual(b.handle({'cmd':'offboard_bench'})['status'],'error')
        self.assertEqual(b.handle({'cmd':'offboard_bench','propsOffConfirmed':True})['status'],'error')
        self.assertEqual(b.master.mav.sent,[])

    def test_offboard_zero_thrust_sequence_and_cleanup(self):
        b=fixture()
        b.ingest(Message(kind='ATTITUDE',roll=0,pitch=0,yaw=0))
        wire=b.master.mav
        targets=[]
        def target(*args):
            targets.append(args)
            for _, msg in list(b.messages.values()): b.ingest(msg)
        wire.set_attitude_target_send=target
        original=wire.command_long_send
        def send(*args):
            original(*args)
            hb=b.messages['HEARTBEAT'][1]
            if args[2]==M.MAV_CMD_DO_SET_MODE: hb.custom_mode=(int(args[5])<<16)|(int(args[6])<<24)
            if args[2]==M.MAV_CMD_COMPONENT_ARM_DISARM:
                hb.base_mode=(hb.base_mode|M.MAV_MODE_FLAG_SAFETY_ARMED) if args[4] else (hb.base_mode & ~M.MAV_MODE_FLAG_SAFETY_ARMED)
            b.ingest(hb)
        wire.command_long_send=send
        reply=b.handle({'cmd':'offboard_bench','propsOffConfirmed':True})
        self.assertEqual(reply['status'],'ok',reply)
        self.assertFalse(b.status()['armed'])
        self.assertFalse(b.enabled)
        self.assertGreater(len(targets),20)
        self.assertTrue(all(a[-1]==0 and a[-4:-1]==(0,0,0) for a in targets))
        self.assertEqual([a[4:6] for _,a in wire.sent if a[2]==M.MAV_CMD_COMPONENT_ARM_DISARM],[(1,0),(0,0)])


    def test_failed_serial_connection_can_be_released(self):
        b=fixture()
        b.link_error='Serial device removed'
        b.master.close=lambda: None
        self.assertFalse(b.status()['connected'])
        self.assertTrue(b.status()['canDisconnect'])
        self.assertEqual(b.handle({'cmd':'disconnect'})['status'],'ok')
        self.assertIsNone(b.master)


    def test_props_off_arm_without_gps_keeps_navigation_locked(self):
        b=fixture()
        b.messages['GPS_RAW_INT'][1].fix_type=0
        reply=b.handle({'cmd':'arm','propsOffConfirmed':True})
        self.assertEqual(reply['status'],'ok')
        self.assertEqual(b.master.mav.sent[0][1][4:6],(1,0))
        self.assertFalse(b.enabled)
        self.assertFalse(b.status()['canTakeoff'])

    def test_props_off_arm_rejects_unknown_ground_state(self):
        b=fixture()
        b.messages.pop('EXTENDED_SYS_STATE')
        self.assertEqual(b.handle({'cmd':'arm','propsOffConfirmed':True})['status'],'error')
        self.assertEqual(b.master.mav.sent,[])

    def test_bench_requires_props_confirmation(self):
        b=fixture()
        self.assertEqual(b.handle({'cmd':'bench_arm'})['status'],'error')
        self.assertEqual(b.master.mav.sent,[])

    def test_bench_rejection_is_normal_not_forced_and_leaves_flight_locked(self):
        b=fixture()
        b.messages['GPS_RAW_INT'][1].fix_type=0
        b.master.mav.result=M.MAV_RESULT_DENIED
        reply=b.handle({'cmd':'bench_arm','propsOffConfirmed':True})
        self.assertEqual(reply['status'],'error')
        self.assertEqual(b.master.mav.sent[0][1][4:6],(1,0))
        self.assertFalse(b.enabled)

    def test_bench_observes_arming_then_disarms(self):
        b=fixture()
        b.messages['GPS_RAW_INT'][1].fix_type=0
        wire=b.master.mav
        original=wire.command_long_send
        def send(*args):
            original(*args)
            if args[2]==M.MAV_CMD_COMPONENT_ARM_DISARM:
                hb=b.messages['HEARTBEAT'][1]
                hb.base_mode = (hb.base_mode | M.MAV_MODE_FLAG_SAFETY_ARMED) if args[4] else (hb.base_mode & ~M.MAV_MODE_FLAG_SAFETY_ARMED)
                b.ingest(hb)
        wire.command_long_send=send
        reply=b.handle({'cmd':'bench_arm','propsOffConfirmed':True})
        self.assertEqual(reply['status'],'ok')
        self.assertTrue(reply['data']['armedObserved'])
        self.assertTrue(reply['data']['disarmedConfirmed'])
        self.assertEqual([args[4:6] for _,args in wire.sent],[(1,0),(0,0)])
        self.assertFalse(b.enabled)



class RangeTests(unittest.TestCase):
    def test_downward_range_is_converted_to_meters(self):
        b=fixture()
        b.ingest(Message(kind='DISTANCE_SENSOR',id=0,orientation=M.MAV_SENSOR_ROTATION_PITCH_270,min_distance=10,max_distance=800,current_distance=125))
        self.assertEqual(b.status()['downwardDistance'],1.25)
        b.ingest(Message(kind='DISTANCE_SENSOR',id=1,orientation=M.MAV_SENSOR_ROTATION_NONE,min_distance=10,max_distance=800,current_distance=300))
        self.assertEqual(b.status()['downwardDistance'],1.25)
    def test_invalid_or_stale_range_is_unknown(self):
        b=fixture()
        msg=Message(kind='DISTANCE_SENSOR',id=0,orientation=M.MAV_SENSOR_ROTATION_PITCH_270,min_distance=10,max_distance=800,current_distance=900)
        b.ingest(msg)
        self.assertIsNone(b.status()['downwardDistance'])
        msg.current_distance=100; b.ingest(msg)
        t,m=b.distance_sources[0];b.distance_sources[0]=(t-10,m)
        self.assertIsNone(b.status()['downwardDistance'])
    def test_range_never_unlocks_gps_flight(self):
        b=fixture()
        b.messages['GPS_RAW_INT'][1].fix_type=0
        b.ingest(Message(kind='DISTANCE_SENSOR',id=0,orientation=M.MAV_SENSOR_ROTATION_PITCH_270,min_distance=10,max_distance=800,current_distance=125))
        b.enabled=True
        self.assertFalse(b.status()['canTakeoff'])
        self.assertFalse(b.status()['canNavigate'])

if __name__ == '__main__': unittest.main()
