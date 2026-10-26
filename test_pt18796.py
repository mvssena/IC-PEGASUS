import sys, time, types, unittest
from unittest.mock import MagicMock
mavutil = types.SimpleNamespace(mavlink=types.SimpleNamespace(MAV_CMD_COMPONENT_ARM_DISARM=400))
sys.modules.setdefault('pymavlink', types.SimpleNamespace(mavutil=mavutil))
from mavlink_interface import MAVLinkInterface
from mission_fsm import TakeoffState, TransitState, ApproachTargetState, LandState
from perception_consumer import PerceptionConsumer

class Tests(unittest.TestCase):
    def test_gps(self):
        m = MAVLinkInterface(); self.assertFalse(m.verify_gps_readiness())
        m.gps_received_at = time.monotonic(); m.gps_fix = 3; m.satellites = 10; m.hdop = 1.4
        self.assertTrue(m.verify_gps_readiness())
        m.hdop = float('inf'); self.assertFalse(m.verify_gps_readiness())
        m.hdop = 1.0; m.gps_received_at -= 3; self.assertFalse(m.verify_gps_readiness())
        m.master = MagicMock(); self.assertFalse(m.arm()); m.master.mav.command_long_send.assert_not_called()
    def test_estados(self):
        c = types.SimpleNamespace(mav=MagicMock(), perception=MagicMock())
        c.mav.current_altitude = 2.5; c.mav.is_armed = True; c.mav.arm.return_value = True
        a = TakeoffState(); a.on_enter(c); self.assertEqual(a.execute(c), 'DESLOCAR')
        b = TransitState(); b.on_enter(c); c.perception.get_latest_detection.return_value = {'estavel': True, 'confidence': .95}
        self.assertEqual(b.execute(c), 'APROXIMAR_ALVO')
        d = ApproachTargetState(); d.on_enter(c); c.mav.current_altitude = .75
        c.perception.get_latest_detection.return_value = {'estavel': True, 'err_x': .02, 'err_y': .03}
        self.assertEqual(d.execute(c), 'POUSAR')
        for state in (a,b,d):
            state.start_time = time.time() - state.timeout_sec - 1
            self.assertEqual(state.execute(c), 'POUSAR')
        e = LandState(); e.on_enter(c); e.start_time = time.time() - e.timeout_sec - 1
        self.assertEqual(e.execute(c), 'FALHA_POUSO')
        c.mav.is_armed = False; self.assertEqual(e.execute(c), 'CONCLUIDO')
    def test_percepcao(self):
        p = PerceptionConsumer(timeout_ms=10)
        p.feed_contract_payload({'confidence': .2, 'estavel': True}); self.assertIsNone(p.get_latest_detection())
        p.feed_contract_payload({'confidence': .9, 'estavel': False}); self.assertFalse(p.get_latest_detection()['estavel'])
        time.sleep(.02); self.assertIsNone(p.get_latest_detection())
if __name__ == '__main__': unittest.main()
