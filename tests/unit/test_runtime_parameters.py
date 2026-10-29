"""Shared settings must reach consumers and retain explicit protection bounds."""
import json
import tempfile
import unittest
from pathlib import Path
from snowbot_hardware.parameters import load_parameters


class RuntimeParameterTests(unittest.TestCase):
    def load(self, value):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'parameters.json';path.write_text(json.dumps(value))
            return load_parameters(path)

    def test_default_snapshot_preserves_existing_policy(self):
        cfg=load_parameters()
        self.assertEqual(cfg['pico']['prepare_command_period_s'],.04)
        self.assertEqual(cfg['pico']['prepare_command_ttl_s'],.25)
        self.assertEqual(cfg['gripper']['vr_input_max_age_s'],.25)
        self.assertEqual(cfg['execution']['stop_confirmation_window_s'],.1)
        self.assertEqual(cfg['diagnostics']['pdo_capacity'],1200)
        self.assertEqual(cfg['display']['poll_ms'],150)

    def test_partial_override_and_independent_snapshots(self):
        cfg=self.load({'pico':{'prepare_command_period_s':.02},'display':{'poll_ms':250}})
        self.assertEqual(cfg['pico']['prepare_command_period_s'],.02)
        self.assertEqual(cfg['pico']['prepare_command_ttl_s'],.25)
        cfg['pico']['prepare_command_ttl_s']=.3
        self.assertEqual(load_parameters()['pico']['prepare_command_ttl_s'],.25)

    def test_numeric_overrides_have_ros_compatible_types(self):
        cfg=self.load({'pico':{'pair_max_skew_s':1,'rpc_timeout_s':2},'display':{'poll_ms':200}})
        self.assertIs(type(cfg['pico']['pair_max_skew_s']),float)
        self.assertIs(type(cfg['pico']['rpc_timeout_s']),float)
        self.assertIs(type(cfg['display']['poll_ms']),int)

    def test_unknown_fields_types_and_nonfinite_values_are_rejected(self):
        for bad in ({'other':{}},{'schema_version':True},{'pico':{'other':1}},
                    *({'pico':{'rpc_timeout_s':v}} for v in (0,-1,True,'2',float('nan'),float('inf'),10**400)),
                    {'display':{'poll_ms':150.5}}):
            with self.subTest(value=bad),self.assertRaises(ValueError):self.load(bad)

    def test_related_budgets_and_hysteresis_are_checked_together(self):
        for bad in ({'pico':{'prepare_command_period_s':.25}},
                    {'pico':{'prepare_command_ttl_s':.5}},
                    {'pico':{'trigger_release_threshold':.8}},
                    {'diagnostics':{'pdo_sample_hz':200}},
                    {'execution':{'startup_hold_ttl_s':.5}}):
            with self.subTest(value=bad),self.assertRaises(ValueError):self.load(bad)

    def test_configured_button_hold_and_trigger_edges_reach_state_machine(self):
        from tests.support.pico import Rig
        from snowbot_teleop.control import PicoControl
        from bindu_teleoperation.vr.receiver import ButtonCounters
        cfg=self.load({'pico':{'prepare_button_hold_s':2.,'trigger_press_threshold':.6,'trigger_release_threshold':.2}})
        rig=Rig();rig.core=PicoControl(initial_load='empty',settings=cfg['pico'])
        rig.counters=[ButtonCounters(.6,.2),ButtonCounters(.6,.2)]
        rig.step();rig.triggers[0]=.65;rig.step()
        self.assertNotEqual(rig.core.reason,'PICO_BUTTON_STATE_COUNTER_MISMATCH')
        self.assertEqual(rig.counters[0].counts[6],1)
        # The trigger above intentionally starts a separate operation. Use an
        # idle coordinator for the independent A-button hold requirement.
        rig=Rig();rig.core=PicoControl(initial_load='empty',settings=cfg['pico'])
        rig.step();rig.step();rig.buttons[1]=1
        effects=[]
        for _ in range(60):effects+=rig.step()
        self.assertFalse(any(x.operation=='prepare' for x in effects))
        for _ in range(45):effects+=rig.step()
        self.assertTrue(any(x.operation=='prepare' for x in effects))

    def test_diagnostic_transition_sampler_uses_selected_trigger_thresholds(self):
        from snowbot_bringup.teleop.snowbot_diagnostics import DiagnosticSampler
        cfg=self.load({'pico':{'trigger_press_threshold':.6,'trigger_release_threshold':.2}})
        sampler=DiagnosticSampler(cfg['diagnostics'],cfg['pico'])
        self.assertTrue(sampler.keep('vr-raw',{'side':'left','trigger':.55},1.))
        self.assertTrue(sampler.keep('vr-raw',{'side':'left','trigger':.65},1.001))

    def test_publication_manifest_uses_actual_protocol_limit(self):
        from tools.deployment.prepare_position_session import contract_ttl_ms
        self.assertEqual(contract_ttl_ms('maximum_target_ttl'),400)
        self.assertEqual(contract_ttl_ms('maximum_target_pause_ttl'),2000)

    def test_independent_hand_consumes_configured_grip_thresholds(self):
        import numpy as np
        from bindu_contracts.teleoperation import VRFrame
        from bindu_teleoperation.independent_session import IndependentArmsSession
        session=IndependentArmsSession(dict(input_max_age=.25,engage_seconds=.1,position_scale=1.,
            grip_press_threshold=.6,grip_release_threshold=.2),1,0.)
        pose=tuple(np.eye(4).ravel())
        for seq,now,left in ((1,1.,0.),(2,1.02,.65),(3,1.15,.65)):
            frames=[VRFrame('vr',seq,now,side,pose,left if side=='left' else 0.) for side in ('left','right')]
            session.ingest(*frames,now)
        self.assertEqual(session.hands[0].mode,'follow')
        self.assertEqual(session.hands[1].mode,'idle')
