"""Shared settings must reach consumers and retain explicit protection bounds."""
import json
import tempfile
import unittest
from pathlib import Path
from snowbot_hardware.parameters import load_parameters, validate_pico_timing, apply_tool_timing, braking_lead


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

    def test_shared_timing_preserves_standalone_defaults_and_selected_overrides(self):
        from bindu_contracts.resources import package_share
        base=package_share('snowbot_bringup')/'config/robots/snowbot'
        for name in ('bimanual_snowbot_real.json',):
            original=json.loads((base/name).read_text())
            effective=apply_tool_timing(original,load_parameters())
            for key in ('input_max_age','feedback_max_age','command_max_age',
                        'solve_timeout_us','worker_timeout','worker_startup_timeout'):
                self.assertEqual(effective[key],original[key],(name,key))
            selected=self.load({'teleop':{'input_max_age_s':.3,'feedback_max_age_s':.15,'command_ttl_s':.35},
                                'solver':{'timeout_s':.1,'worker_timeout_s':.4,'startup_timeout_s':10.}})
            changed=apply_tool_timing(original,selected)
            self.assertEqual([changed[k] for k in ('input_max_age','feedback_max_age','command_max_age',
                              'solve_timeout_us','worker_timeout','worker_startup_timeout')],
                             [.3,.15,.35,100000,.4,10.])
            self.assertIs(type(changed['solve_timeout_us']),int)
            self.assertEqual(original['input_max_age'],.25)

    def test_shared_input_deadline_reaches_actual_session(self):
        import numpy as np
        from bindu_contracts.teleoperation import VRFrame
        from bindu_teleoperation.independent_session import IndependentArmsSession
        cfg=apply_tool_timing(dict(engage_seconds=.1,position_scale=1.),
                             self.load({'teleop':{'input_max_age_s':.3}}))
        pose=tuple(np.eye(4).ravel())
        frames=[VRFrame('vr',1,1.,side,pose,0.) for side in ('left','right')]
        session=IndependentArmsSession(cfg,1,0.)
        session.ingest(*frames,1.28)
        with self.assertRaises(ValueError):session.ingest(*frames,1.31)

    def test_migrated_braking_has_one_source_and_rejects_conflicts(self):
        cfg=self.load({'execution':{'target_stream_brake_lead_s':.04}})
        self.assertEqual(braking_lead(cfg),.04)
        self.assertEqual(braking_lead(cfg,'0.04'),.04)
        for legacy in (.12,True,'nan'):
            with self.assertRaisesRegex(ValueError,'SNOWBOT_BRAKE_LEAD_MOVED'):
                braking_lead(cfg,legacy)
        with self.assertRaisesRegex(ValueError,'SAMPLE_GAP'):
            apply_tool_timing({'integration_max_sample_gap':.25},
                             self.load({'teleop':{'input_max_age_s':.2}}))
        with self.assertRaisesRegex(ValueError,'BRAKE_LEAD_EXCEEDS_TTL'):
            validate_pico_timing(self.load({'teleop':{'input_max_age_s':.1,'command_ttl_s':.1}}),brake_lead_s=.12)

    def test_installed_tool_loader_applies_snapshot_before_generic_node_uses_config(self):
        import ast
        from importlib.util import find_spec
        from types import SimpleNamespace as S
        from unittest.mock import patch
        base_source=Path(find_spec('bindu_runtime').origin).parent/'bimanual_tool_node.py'
        app_source=Path(find_spec('snowbot_teleop').origin).parent/'tool_node.py'
        loader=next(n for n in ast.parse(app_source.read_text()).body if getattr(n,'name','')=='load_tool')
        namespace={}
        exec(compile(ast.Module(body=[loader],type_ignores=[]),str(app_source),'exec'),namespace)
        cls=next(n for n in ast.parse(base_source.read_text()).body if getattr(n,'name','')=='BimanualToolNode')
        init=next(n for n in cls.body if getattr(n,'name','')=='__init__')
        start=next(i for i,n in enumerate(init.body) if isinstance(n,ast.Assign) and
                   isinstance(n.targets[0],ast.Attribute) and n.targets[0].attr=='cfg')
        with tempfile.TemporaryDirectory() as tmp:
            parameters=Path(tmp)/'parameters.json';tool=Path(tmp)/'tool.json'
            parameters.write_text(json.dumps({'solver':{'timeout_s':.1,'worker_timeout_s':.4}}))
            tool.write_text(json.dumps({'input_max_age':9.,'command_max_age':9.}))
            node=S(get_parameter=lambda name:S(value=str(tool)),
                   declare_parameter=lambda *a,**kw:S(value=str(parameters)))
            modules={'rcl_interfaces':S(),'rcl_interfaces.msg':S(ParameterDescriptor=lambda **kw:kw)}
            with patch.dict('sys.modules',modules):
                exec(compile(ast.Module(body=init.body[start:start+2],type_ignores=[]),str(base_source),'exec'),
                     dict(self=node,json=json,Path=Path,tool_config_loader=namespace['load_tool']))
            self.assertEqual(node.cfg['command_max_age'],.4)
            self.assertEqual(node.cfg['solve_timeout_us'],100000)
            self.assertEqual(node.cfg['worker_timeout'],.4)
            self.assertEqual(node.runtime_parameters['solver']['timeout_s'],.1)

    def test_primary_and_example_configurations_resolve_without_duplicate_tool_files(self):
        from bindu_contracts.resources import package_share, resource_file
        from bindu_runtime.robot_setup import RobotSetup
        from bindu_runtime.teleop_deployment import load_deployment
        share=package_share('snowbot_bringup');config=share/'config'
        self.assertEqual(len(list((config/'robots/snowbot').glob('*.json'))),10)
        self.assertEqual({p.name for p in (config/'deployments').glob('*.json')},
                         {'snowbot_vr_real.json','snowbot_vr_sim.json'})
        selections=[]
        for name,policy in (('snowbot_vr_real.json','operator'),('snowbot_vr_sim.json','calibrated')):
            _,args=load_deployment(config/'deployments'/name,package_share('bindu_runtime'))
            robot=RobotSetup.load(args['robot_config'])
            selections.append((robot.path,robot.profile_path,robot.tool_config))
            pico=json.loads(Path(args['pico_config']).read_text())
            self.assertEqual(pico['tool_grasp_confirmation'],policy)
        self.assertEqual(selections[0],selections[1])
        example=config/'examples/snowbot'
        RobotSetup.load(example/'robot_snowbot.json')
        for name in ('snowbot_real_teleop.json','teleop_pico_snowbot_sim.json'):
            load_deployment(example/name,package_share('bindu_runtime'))

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

    def test_selected_pico_timing_uses_actual_braking_and_lease_budgets(self):
        cfg=load_parameters();validate_pico_timing(cfg,brake_lead_s=.12)
        cfg['pico']['prepare_command_period_s']=.2
        validate_pico_timing(cfg)  # Simulation has no target-stream pre-brake.
        validate_pico_timing(cfg,brake_lead_s=.04)
        with self.assertRaisesRegex(ValueError,'SNOWBOT_PREPARE_TIMING'):
            validate_pico_timing(cfg,brake_lead_s=.12)
        cfg=load_parameters();cfg['gripper']['lease_ttl_s']=.15
        with self.assertRaisesRegex(ValueError,'SNOWBOT_PICO_RENEWAL_TIMING'):
            validate_pico_timing(cfg)
        cfg['gripper']['lease_ttl_s']=4.;cfg['pico']['renewal_period_s']=2.
        with self.assertRaisesRegex(ValueError,'SNOWBOT_PICO_RENEWAL_TIMING'):
            validate_pico_timing(cfg)

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
                    {'execution':{'startup_hold_ttl_s':.5}},
                    {'teleop':{'command_ttl_s':.5}},
                    {'solver':{'timeout_s':.3}},
                    {'execution':{'target_stream_brake_lead_s':.4}}):
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
