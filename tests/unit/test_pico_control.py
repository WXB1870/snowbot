"""Physical button identity, coalesced events and guarded operator transitions."""
from importlib.util import find_spec
from bindu_contracts.resources import package_share
from dataclasses import replace
import unittest
import json
from pathlib import Path
import numpy as np
from bindu_contracts.teleoperation import VRFrame
from bindu_teleoperation.vr.receiver import decode_controller, ButtonCounters
from snowbot_teleop.control import PicoControl, PicoFeedback, PICO_MAPPING, temporary_capture_refusal
from bindu_teleoperation.bimanual_session import BimanualSession


from tests.support.pico import Rig


class ScenarioRuntimeTests(unittest.IsolatedAsyncioTestCase):
    def test_ground_lift_requires_relative_fault_long_x_and_released_controls(self):
        r=Rig(load='held',guided=True,dual=True);r.core.teleop_mode='bimanual_tool'
        r.core.ground_recovery_enabled=True
        r.core.lock('PICO_TOOL_SESSION_ENDED:BIMANUAL_MEASURED_GRASP_ERROR',r.now)
        r.core.drain();r.complete();r.step()
        r.buttons[0]=1
        self.assertEqual(r.step(),[])
        effects=[]
        for _ in range(52):effects+=r.step()
        self.assertEqual([e.operation for e in effects],['ground_recovery'])
        self.assertEqual(r.core.phase,'ground_recovery')
        r.complete();self.assertEqual(r.core.phase,'locked')
        for _ in range(60):self.assertEqual(r.step(),[])
        r.buttons[0]=0;r.step();r.buttons[0]=1
        effects=[]
        for _ in range(55):effects+=r.step()
        self.assertEqual([e.operation for e in effects],['ground_recovery'])
        r.buttons[0]=0
        self.assertEqual([e.operation for e in r.step()],['stop'])
        self.assertEqual(r.core.phase,'locked')

    def test_ground_lift_never_overrides_device_fault_y_stop_or_tracking_loss(self):
        for reason in ('PICO_Y_STOP','DEVICE_FAULT','SNOWBOT_PDO_STALE','PICO_TRACKING_LOST'):
            r=Rig(load='held',guided=True,dual=True);r.core.teleop_mode='bimanual_tool'
            r.core.ground_recovery_enabled=True
            r.core.lock(reason,r.now);r.core.drain();r.complete();r.step();r.buttons[0]=1
            self.assertTrue(all(not any(e.operation=='ground_recovery' for e in r.step()) for _ in range(60)))
        r=Rig(load='held',guided=True,dual=True);r.core.teleop_mode='bimanual_tool'
        r.core.ground_recovery_enabled=True
        r.core.lock('BIMANUAL_MEASURED_GRASP_ERROR',r.now);r.core.drain();r.complete();r.step()
        r.grips[0]=1.;r.buttons[0]=1
        self.assertTrue(all(not any(e.operation=='ground_recovery' for e in r.step()) for _ in range(60)))

    def test_b_recovery_hint_preserves_relative_fault_manual_lift_entry(self):
        r=Rig(load='held',guided=True,dual=True);r.core.teleop_mode='bimanual_tool'
        r.core.ground_recovery_enabled=True
        r.core.lock('BIMANUAL_MEASURED_GRASP_ERROR',r.now);r.core.drain();r.complete();r.step()
        r.grips=[1.,0.];r.step();r.click(1,1)
        self.assertEqual(r.core.reason,'RECOVERY_RELEASE_CONTROLS')
        self.assertTrue(r.core.ground_recovery_candidate())
        r.grips=[0.,0.];r.step();r.click(1,1)
        r.complete(False,'BIMANUAL_WORKER_TIMEOUT')
        self.assertTrue(r.core.ground_recovery_candidate())
        r.step();r.buttons[0]=1
        effects=[]
        for _ in range(55):effects+=r.step()
        self.assertEqual([e.operation for e in effects],['ground_recovery'])
        r.core.recovery_required='device_inspection'
        self.assertFalse(r.core.ground_recovery_candidate())

    def test_ground_lift_rejects_increasing_measured_error_or_downward_motion(self):
        node=self.production('accept_ground_observation')
        node.tool_cfg={'ground_recovery':dict(relative_position_margin_m=.01,relative_rotation_margin_rad=.05)}
        initial=dict(height_m=-.01,relative_position_error_m=.07,relative_rotation_error_rad=.04)
        node.ground_recovery_plan={'initial':initial}
        node.accept_ground_observation(dict(initial,height_m=.01,relative_position_error_m=.02))
        node.accept_ground_observation(dict(initial,height_m=-.013))
        for value in (dict(initial,height_m=-.03),dict(initial,relative_position_error_m=.081),
                      dict(initial,relative_rotation_error_rad=.091)):
            with self.assertRaisesRegex(RuntimeError,'GEOMETRY_WORSENED'):node.accept_ground_observation(value)

    async def test_ground_lift_stream_starts_at_authenticated_hold_and_cancellation_releases(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock,AsyncMock
        for changed in (None,'hold','start'):
            node=self.production('move');node.current=Mock();node.names=('joint',);node.groups=('arms',)
            lease=S(ok=True,lease_id='lease',epoch=2,profile_hash='profile')
            state=S(instance_id='executor',lease_active=True,lease_epoch=2,revision=1,
                    command_id='',state='CANCELED',joints=S(name=['joint'],velocity=[0.]))
            node.state=state;node.reading=lambda:(state,{'joint':.08});node.feedback=lambda:S(stopped=True)
            node.profile=S(digest='profile',limits={'joint':(-1.,1.)},feedback_tolerances={'joint_position':.002})
            node.cfg={};node.instance_id='pico';node.runtime_mode='real';node.position_contract=True
            node.simulation=False;node.event=Mock();node.now=lambda:0.;node.monitor_ground_recovery=Mock()
            node.ground_recovery_plan=dict(hold={'positions':{'joint':.1}},measured_start=[.08])
            node.prepare_hold_reference=AsyncMock(return_value=node.ground_recovery_plan['hold'])
            if changed=='hold':node.prepare_hold_reference.side_effect=RuntimeError('HOLD_CHANGED')
            elif changed=='start':
                async def shifted(*args,**kwargs):
                    node.reading=lambda:(state,{'joint':.09})
                    return node.ground_recovery_plan['hold']
                node.prepare_hold_reference.side_effect=shifted
            node.leases=S(call_async=lambda request:lease);node.wait=AsyncMock(side_effect=lambda value:value)
            node.release_lease=AsyncMock();node.submit=S(call_async=Mock(return_value=S(done=lambda:True,result=lambda:S(accepted=True,revision=2))))
            node.reset_grasp_reference=AsyncMock()
            scope=node.move.__func__.__globals__
            node.pause=AsyncMock(side_effect=scope['Superseded']('released X'))
            curve=Mock(valid=lambda *a:True,sample=lambda t:S(q=[.1],v=[0.],a=[0.]))
            scope.update(Curve=S(between=Mock(return_value=curve)),State=S(rest=lambda q:S(q=q)),
                Lease=S(Request=S),SubmitMotion=S(Request=S),MotionCommand=S,JointTrajectoryPoint=S,Duration=S,
                stamp=lambda t:S(sec=0,nanosec=0),uuid=S(uuid4=lambda:S(hex='command')))
            with self.assertRaisesRegex(RuntimeError,{'hold':'HOLD_CHANGED','start':'START_CHANGED'}.get(changed,'released X')):
                await node.move(S(token=1,operation='ground_recovery'),{'joint':.15},5.)
            node.release_lease.assert_awaited_once();node.reset_grasp_reference.assert_not_awaited()
            if changed:node.submit.call_async.assert_not_called()
            else:self.assertEqual(node.submit.call_async.call_args.args[0].command.positions,[.1])

    def test_x_held_before_a_fault_cannot_start_recovery_without_release(self):
        for entry in ('lock','operation_failure'):
            r=Rig(load='held',guided=True,dual=True);r.core.teleop_mode='bimanual_tool'
            r.core.ground_recovery_enabled=True;r.core.continuous_axial=True
            r.core.phase='following';r.grips=[1.,1.];r.stopped=False;r.buttons[0]=1
            self.assertEqual(r.step(),[])  # X has no grasp-switch effect in this real axial mode.
            if entry=='lock':
                r.core.lock('BIMANUAL_MEASURED_GRASP_ERROR',r.now);r.core.drain();r.complete()
            else:
                r.core.emit('follow');r.core.drain();r.complete(False,'BIMANUAL_MEASURED_GRASP_ERROR')
            r.stopped=True;r.grips=[0.,0.]
            self.assertTrue(all(not r.step() for _ in range(60)))
            r.buttons[0]=0;r.step();r.buttons[0]=1
            effects=[]
            for _ in range(55):effects+=r.step()
            self.assertEqual([e.operation for e in effects],['ground_recovery'])

    def test_x_with_a_b_or_y_cannot_start_a_lift(self):
        for side,mask in ((1,1),(1,2),(0,2)):
            r=Rig(load='held',guided=True,dual=True);r.core.teleop_mode='bimanual_tool'
            r.core.ground_recovery_enabled=True
            r.core.lock('BIMANUAL_MEASURED_GRASP_ERROR',r.now);r.core.drain();r.complete();r.step()
            r.buttons[0]=1;r.step()
            for _ in range(49):self.assertEqual(r.step(),[])
            r.buttons[side]|=mask
            effects=r.step()
            self.assertFalse(any(e.operation in ('ground_recovery','prepare') for e in effects))
            self.assertNotEqual(r.core.phase,'ground_recovery')

    def test_a_b_y_stop_a_lift_and_held_x_cannot_restart_it(self):
        for side,mask in ((1,1),(1,2),(0,2)):
            r=Rig(load='held',guided=True,dual=True);r.core.teleop_mode='bimanual_tool'
            r.core.ground_recovery_enabled=True
            r.core.lock('BIMANUAL_MEASURED_GRASP_ERROR',r.now);r.core.drain();r.complete();r.step()
            r.buttons[0]=1
            for _ in range(55):r.step()
            self.assertEqual(r.core.phase,'ground_recovery')
            r.buttons[side]|=mask
            self.assertEqual([e.operation for e in r.step()],['stop'])
            r.complete();r.buttons[side]&=~mask
            self.assertTrue(all(not r.step() for _ in range(60)))

    def test_ground_lift_monitor_uses_live_reference_and_fails_on_late_or_wrong_token(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock
        node=self.production('monitor_ground_recovery','accept_ground_observation')
        clock=[0.];scope=node.monitor_ground_recovery.__func__.__globals__
        scope['time'].monotonic=lambda:clock[0]
        reference=Mock(return_value=((.2,),1.))
        scope['sampled_nominal_reference']=reference
        node.tool_cfg=dict(feedback_max_age=.5,ground_recovery=dict(relative_position_margin_m=.01,relative_rotation_margin_rad=.05))
        initial=dict(height_m=-.01,relative_position_error_m=.07,relative_rotation_error_rad=.04)
        plan=dict(hold={'positions':{'joint':.1}},initial=initial,next_monitor=0.)
        node.ground_recovery_plan=plan
        worker=S(busy=False,submitted=0.)
        def submit(*args,**kwargs):
            worker.busy=True;worker.submitted=clock[0]
            return True
        worker.submit=Mock(side_effect=submit)
        node.validator=worker;node.validation_result=None
        node.names=('joint',);node.now=lambda:1.;intent=S(token=7);lease=S(epoch=2);state=object()
        node.monitor_ground_recovery(intent,state,{'joint':.1},lease)
        self.assertEqual(node.validator.submit.call_args.kwargs['reference'],[.1])
        reference.assert_not_called()
        self.assertFalse(node.monitor_ground_recovery(intent))
        node.validation_result=('result',7,initial)
        worker.busy=False
        self.assertTrue(node.monitor_ground_recovery(intent))
        self.assertEqual(worker.submit.call_count,1)
        plan['stream_started']=0.;clock[0]=.2
        node.monitor_ground_recovery(intent,state,{'joint':.2},lease)
        self.assertEqual(node.validator.submit.call_args.kwargs['reference'],(.2,))
        node.validation_result=('result',6,initial)
        with self.assertRaises(RuntimeError):node.monitor_ground_recovery(intent,state,{'joint':.2},lease)
        node.validation_result=None;clock[0]=.71
        with self.assertRaisesRegex(RuntimeError,'MONITOR_TIMEOUT'):
            node.monitor_ground_recovery(intent,state,{'joint':.2},lease)

    def test_ground_lift_active_y_or_input_loss_always_requests_stop(self):
        for stop in ('Y','input_loss'):
            r=Rig(load='held',guided=True,dual=True);r.core.ground_recovery_enabled=True
            r.core.phase='ground_recovery';r.core.emit('ground_recovery');r.core.drain()
            if stop=='Y':r.buttons[0]=2;effects=r.step()
            else:
                r.core.tick(r.now+r.core.max_age+.01);effects=r.core.drain()
            self.assertEqual([e.operation for e in effects],['stop'])
            self.assertEqual(r.core.phase,'locked')

    def test_ground_lift_late_completed_monitor_result_is_rejected(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock
        node=self.production('monitor_ground_recovery','accept_ground_observation')
        node.monitor_ground_recovery.__func__.__globals__['time'].monotonic=lambda:.6
        initial=dict(height_m=-.01,relative_position_error_m=.07,relative_rotation_error_rad=.04)
        node.tool_cfg=dict(feedback_max_age=.5,ground_recovery=dict(relative_position_margin_m=.01,relative_rotation_margin_rad=.05))
        node.ground_recovery_plan=dict(initial=initial,next_monitor=1.)
        node.validator=S(busy=False,submitted=0.,submit=Mock(return_value=True))
        node.validation_result=('result',7,initial)
        node.names=('joint',);node.now=lambda:1.
        with self.assertRaisesRegex(RuntimeError,'MONITOR_TIMEOUT'):
            node.monitor_ground_recovery(S(token=7),object(),{'joint':.1},S(epoch=2))
        node.validator.submit.assert_not_called()

    async def test_ground_lift_reports_measured_gain_without_an_extra_half_step_gate(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock,AsyncMock
        node=self.production('ground_recovery','accept_ground_observation')
        initial=dict(height_m=-.01,relative_position_error_m=.07,relative_rotation_error_rad=.04)
        plan=dict(positions=[.15],duration_s=5.,initial=initial,measured_start=[.08])
        status=dict(initial,height_m=-.005)
        node.tool_cfg={'ground_recovery':dict(relative_position_margin_m=.01,relative_rotation_margin_rad=.05)}
        node.recover=AsyncMock();node.prepare_hold_reference=AsyncMock(return_value={'positions':{'joint':.1}})
        node.ground_request=AsyncMock(side_effect=[plan,status]);node.monitor_ground_recovery=Mock(return_value=True)
        node.names=('joint',);node.reading=lambda:(None,{'joint':.08});node.move=AsyncMock();node.event=Mock()
        await node.ground_recovery(S(token=1))
        self.assertAlmostEqual(status['lifted_m'],.005)
        self.assertEqual(node.event.call_args.args[0],'GROUND_RECOVERY_STEP_COMPLETE')
        self.assertAlmostEqual(json.loads(node.event.call_args.args[1])['lifted_m'],.005)
        self.assertIsNone(node.ground_recovery_plan)
        node.reset_grasp_reference.assert_not_awaited()

    async def test_rpc_timing_preserves_result_exception_and_timeout(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock,AsyncMock
        for outcome in ('completed','exception','timeout'):
            with self.subTest(outcome=outcome):
                node=self.production('wait');node.event=Mock();node.pause=AsyncMock()
                clock=node.wait.__func__.__globals__['time']
                clock.monotonic=Mock(side_effect=[0.,1.1,1.2] if outcome=='timeout' else [0.,.02])
                future=S(done=lambda:outcome!='timeout',result=Mock(return_value='reply'))
                if outcome=='exception':future.result.side_effect=RuntimeError('ORIGINAL_ERROR')
                if outcome=='completed':self.assertEqual(await node.wait(future,1.),'reply')
                else:
                    with self.assertRaisesRegex(RuntimeError,'PICO_RPC_TIMEOUT' if outcome=='timeout' else 'ORIGINAL_ERROR'):
                        await node.wait(future,1.)
                state,code=node.event.call_args.args
                self.assertEqual(state,'PICO_RPC_TIMING')
                data=json.loads(code)
                self.assertEqual(data['outcome'],outcome);self.assertEqual(data['timeout_s'],1.)
                self.assertGreater(data['elapsed_s'],0.)

    def test_execution_contract_is_supplied_by_assembly(self):
        import ast
        from types import SimpleNamespace as S
        path=Path(find_spec('snowbot_teleop.node').origin)
        node=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='PicoNode')
        init=next(n for n in node.body if getattr(n,'name','')=='__init__')
        def assigned(item,name):
            return isinstance(item,ast.Assign) and isinstance(item.targets[0],ast.Attribute) and item.targets[0].attr==name
        begin=next(i for i,item in enumerate(init.body) if assigned(item,'position_contract'))
        end=next(i for i,item in enumerate(init.body) if assigned(item,'tool_cfg'))
        code=compile(ast.Module(body=init.body[begin:end],type_ignores=[]),str(path),'exec')
        for selected,schema,mode,backend,execution in (
                (True,2,'real','snowbot_vendor','impedance'),
                (True,2,'simulation','kinematic','position'),
                (False,1,'simulation','kinematic','position')):
            owner=S(cfg={'schema_version':schema,'mapping':PICO_MAPPING},runtime_mode=mode,execution_mode=execution,
                declare_parameter=lambda *a,**kw:S(value=selected),get_parameter=lambda *a:S(value=backend))
            exec(code,{'self':owner,'PICO_MAPPING':PICO_MAPPING,'ParameterDescriptor':lambda **kw:kw})
            self.assertIs(owner.position_contract,selected)
            self.assertEqual(owner.backend,backend)
        owner.cfg['schema_version']=2
        with self.assertRaisesRegex(ValueError,'PICO_CONFIG_SCHEMA'):
            exec(code,{'self':owner,'PICO_MAPPING':PICO_MAPPING,'ParameterDescriptor':lambda **kw:kw})

    def test_control_chain_sets_async_transport_before_ros_init_and_honors_site_override(self):
        import ast,os
        from unittest.mock import Mock,patch
        from tests.support.paths import BINDU_ROOT
        root=BINDU_ROOT/'src/integration/bindu_runtime/bindu_runtime'
        for filename,node,runner in (('pico_node.py','PicoNode','spin'),
                ('vr_input_node.py','VRInputNode','spin'),
                ('bimanual_tool_node.py','BimanualToolNode','spin'),
                ('execution_node.py','ExecutionNode','run_execution')):
            path=(Path(find_spec('snowbot_teleop.node').origin) if filename=="pico_node.py" else root/filename)
            main=next(n for n in ast.parse(path.read_text()).body if getattr(n,'name','')=='main')
            for configured in (None,'SYNCHRONOUS'):
                with self.subTest(entry=filename,override=configured),patch.dict(os.environ,
                        {} if configured is None else {'RMW_FASTRTPS_PUBLICATION_MODE':configured},clear=True):
                    seen=[];start=Mock(side_effect=lambda *a,**kw:seen.append(os.environ.get('RMW_FASTRTPS_PUBLICATION_MODE')))
                    ns=dict(os=os,**{runner:start,node:object})
                    exec(compile(ast.Module(body=[main],type_ignores=[]),str(path),'exec'),ns);ns['main']()
                    self.assertEqual(seen,[configured or 'ASYNCHRONOUS'])
                    start.assert_called_once_with(object,**({'threaded':False} if runner=='spin' else {}))

    def test_stale_feedback_keeps_original_deadline_and_stop_failure_priority(self):
        from types import SimpleNamespace as S
        node=self.production('reading');node.lease=node.tool_request=None;node.now=lambda:1.
        node.tool_cfg={'feedback_max_age':.2};node.profile=S(digest='p',feedback_tolerances={},limits={'joint':(-1,1)})
        node.backend='snowbot_vendor';node.simulation=False
        node.state=S(instance_id='executor',stop_failures=[],code='OK',profile_hash='p',
            device_backend='snowbot_vendor',simulated=False,feedback_stamp=S(sec=0,nanosec=900000000),
            joints=S(name=['joint'],position=[0.]))
        self.assertEqual(node.reading()[1],{'joint':0.})
        node.state.feedback_stamp.nanosec=580000000
        with self.assertRaisesRegex(RuntimeError,'^PICO_FEEDBACK_STALE$'):node.reading()
        node.state.stop_failures=['vendor_hold']
        with self.assertRaisesRegex(RuntimeError,'PICO_STOP_UNCONFIRMED'):node.reading()

    async def test_capture_not_ready_returns_to_standby_only_after_confirmed_stop(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock,AsyncMock
        lease_refusal='BIMANUAL_LEASE_NOT_ACQUIRED:'+json.dumps(dict(
            reason='FEEDBACK_NOT_STATIONARY',instance_id='executor',profile_hash='p',epoch=3))
        for accepted,stop_ok,code in ((0,True,'SNOWBOT_HELD_REFERENCE_NOT_STATIONARY'),
                (0,True,lease_refusal),(1,True,lease_refusal),(0,False,lease_refusal),
                (0,True,lease_refusal.replace('FEEDBACK_NOT_STATIONARY','DEVICE_FAULT')),
                (0,True,'BIMANUAL_LEASE_NOT_ACQUIRED:FEEDBACK_NOT_STATIONARY'),
                (1,True,'SNOWBOT_HELD_REFERENCE_NOT_STATIONARY'),
                (0,False,'SNOWBOT_HELD_REFERENCE_NOT_STATIONARY'),
                (0,True,'SNOWBOT_PDO_STALE')):
            with self.subTest(accepted=accepted,stop_ok=stop_ok,code=code):
                r=Rig(guided=True);r.core.phase='engaging';intent=r.core.emit('follow');r.core.drain()
                node=self.production('perform');node.control=r.core;node.position_contract=False
                node.cancel_tool=AsyncMock(side_effect=[None,None if stop_ok else RuntimeError('STOP_UNCONFIRMED')])
                node.current=Mock();node.select_tool_mode=AsyncMock();node.follow=AsyncMock(side_effect=RuntimeError(code))
                node.tool_result=S(done=lambda:True,result=lambda:S(result=S(code=code,accepted_commands=accepted)))
                node.now=lambda:r.now;node.feedback=lambda:PicoFeedback(r.now,stop_ok)
                node.event=Mock();node.simulation=False
                await node.perform(intent)
                retry=stop_ok and temporary_capture_refusal(code,accepted)
                self.assertEqual(r.core.phase,'standby' if retry else 'locked')
                self.assertEqual(r.core.squeeze_released,[False,False])
                self.assertIsNone(r.core.pending)
                if retry:
                    self.assertFalse(r.core.fault_active)
                    r.grips=[1.,1.];self.assertEqual(r.step(),[])

    def test_pending_capture_refusal_is_not_locked_by_timer_before_cleanup(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock
        lease_refusal='BIMANUAL_LEASE_NOT_ACQUIRED:'+json.dumps(dict(
            reason='FEEDBACK_NOT_STATIONARY',instance_id='executor',profile_hash='p',epoch=3))
        for code in ('SNOWBOT_HELD_REFERENCE_NOT_STATIONARY',lease_refusal,'SNOWBOT_PDO_STALE',
                     'BIMANUAL_LEASE_NOT_ACQUIRED:bad-json'):
            node=self.production('tick');node.last_tick_at=None;node.waiters=set();node.validator=S(poll=lambda:None)
            node.state=node.position_ops=node.base=None;node.last_context_at=node.last_status=1.
            node.now=lambda:1.;node.feedback=lambda:PicoFeedback(1.,True)
            node.control=S(phase='engaging',teleop_mode='bimanual_tool',context_at=1.,pending=S(operation='follow'),
                lock=Mock(),update_feedback=Mock(),tick=Mock(),drain=lambda:[])
            node.tool_result=S(done=lambda:True,result=lambda:S(result=S(code=code,accepted_commands=0)))
            node.pending_intent=node.record_future=node.running=None
            node.tick()
            self.assertEqual(node.control.lock.called,not temporary_capture_refusal(code,0))

    def test_continuous_axial_ignores_x_without_changing_gripper_opening(self):
        r=Rig(load='held',guided=True,dual=True,direct=True)
        r.core.continuous_axial=True;r.core.teleop_mode='bimanual_tool'
        r.core.phase='following';r.grips=[1.,1.];r.step()
        self.assertEqual(r.click(0,0),[])
        self.assertEqual(r.core.phase,'following')
        self.assertFalse(r.core.virtual)
        self.assertEqual(r.core.snapshot()['grasp_requested'],'right_axial')
        self.assertEqual(r.modes,('closed','closed'))

    def test_independent_grippers_and_mode_switch_ignore_load_metadata(self):
        r=Rig(load='held',guided=True,dual=True,direct=True)
        r.core.independent_grippers=True;r.core.teleop_mode='bimanual_tool'
        r.core.phase='following';r.grips=[1.,1.];r.step()
        r.triggers=[1.,1.]
        self.assertEqual(r.step(),[]);self.assertEqual(r.core.phase,'following')
        r.triggers=[0.,0.]
        self.assertEqual(r.step(),[]);self.assertEqual(r.core.phase,'following')
        r.core.phase='standby';r.grips=[0.,0.];r.step()
        result=r.click(1,1)
        self.assertEqual(result[-1].operation,'select_mode')
        self.assertEqual(result[-1].value,'independent_arms')
        r.complete();self.assertEqual((r.core.load,r.core.load_source),('empty','teleop_mode'))
        r.modes=('unknown','virtual');r.core.emit('confirm_tool')
        r.complete();self.assertFalse(r.core.virtual)


    async def test_b_recovery_rechecks_exact_stop_before_clearing_latch(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock,AsyncMock
        from bindu_runtime.stop_evidence import stop_feedback_matches
        node=self.production('recover')
        node.recover.__func__.__globals__['stop_feedback_matches']=stop_feedback_matches
        node.current=Mock();node.lease=node.tool_request=node.tool_goal=None
        node.profile=S(digest='profile',feedback_tolerances={});node.simulation=False;node.now=lambda:10.
        node.tool_cfg={'feedback_max_age':.2};node.names=('joint',)
        state=S(instance_id='executor',lease_epoch=2,lease_active=False,profile_hash='profile',
                simulated=False,state='CANCELED',stop_failures=[],code='CANCELED',
                feedback_stamp=9.99,reference=S(name=[]),joints=S(name=['joint'],velocity=[0.]))
        node.reading=Mock(return_value=(state,{'joint':0.}))
        barrier=dict(instance_id='executor',epoch=2,lease_active=False,requested=9.8,failed=True)
        node.stop_request=barrier;node.position_ops=S(recover=AsyncMock())
        node.handoff_complete=False
        node.control=S(update_feedback=Mock(),rest=lambda _:True);node.feedback=Mock()
        await node.recover(S(token=1))
        self.assertIsNone(node.stop_request);node.position_ops.recover.assert_awaited_once()
        self.assertTrue(node.handoff_complete)
        for change in ({'lease_epoch':3},{'lease_active':True},{'stop_failures':['vendor_hold']}):
            state.lease_epoch=2;state.lease_active=False;state.stop_failures=[]
            for key,value in change.items():setattr(state,key,value)
            node.stop_request=barrier;node.position_ops.recover.reset_mock()
            with self.assertRaisesRegex(RuntimeError,'STOP_NOT_CONFIRMED'):await node.recover(S(token=2))
            self.assertIs(node.stop_request,barrier);node.position_ops.recover.assert_not_awaited()
        state.lease_epoch=2;state.lease_active=False;state.stop_failures=[]
        context=dict(instance_id='executor',epoch=1)
        node.tool_request=context;node.tool_goal=S(accepted=True)
        node.tool_result=S(done=lambda:False)
        with self.assertRaisesRegex(RuntimeError,'OWNERSHIP_UNRESOLVED:TOOL_RESULT'):
            await node.recover(S(token=3))
        self.assertIs(node.tool_request,context)
        node.tool_result=S(done=lambda:True,result=lambda:S(result=S(code='BIMANUAL_WORKER_TIMEOUT:STOP_UNCONFIRMED')))
        node.tool_recover_client.call_async.return_value=S(ok=False,code='BIMANUAL_RECOVERY_STOP_NOT_CONFIRMED:DRIVE_FAULT')
        with self.assertRaisesRegex(RuntimeError,'DRIVE_FAULT'):await node.recover(S(token=4))
        self.assertIs(node.tool_request,context)
        node.tool_recover_client.call_async.return_value=S(ok=True)
        await node.recover(S(token=5))
        self.assertIsNone(node.tool_request);self.assertIsNone(node.tool_goal);self.assertIsNone(node.tool_result)
        self.assertEqual(node.tool_phase,'inactive')
        # A timed-out tool acquisition may later be explicitly denied at the
        # baseline epoch. Only the tool recovery service can attest that case.
        state.lease_epoch=0;node.tool_request=context;node.tool_goal=S(accepted=True)
        node.tool_result=S(done=lambda:True,result=lambda:S(result=S(code='LEASE_TIMEOUT:STOP_UNCONFIRMED')))
        node.stop_request=barrier
        await node.recover(S(token=6))
        self.assertIsNone(node.tool_request)
        self.assertEqual(node.tool_recover_client.call_async.call_args.args[0].epoch,0)

    async def test_b_restarts_failed_validator_only_and_discards_its_old_results(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock,AsyncMock
        node=self.production('recover')
        node.current=Mock();node.lease=node.tool_request=node.tool_goal=None
        node.profile=S(digest='profile',feedback_tolerances={});node.simulation=False;node.now=lambda:10.
        node.tool_cfg={'feedback_max_age':.5};node.names=('joint',)
        state=S(instance_id='executor',lease_epoch=2,lease_active=False,state='CANCELED',code='CANCELED',
                stop_failures=[],reference=S(name=[]),joints=S(name=['joint'],velocity=[0.]))
        node.reading=Mock(return_value=(state,{'joint':0.}));node.stop_request=None;node.position_ops=None
        node.control=S(update_feedback=Mock(),rest=lambda _:True);node.feedback=Mock()
        old=S(failure='BIMANUAL_WORKER_TIMEOUT',grasp_checkpoint=([.1],[[1]],None),close=Mock())
        node.validator=old;node.validation_cfg={'original':True};node.validation_result=('result',3,'old')
        node.ground_recovery_plan='old'
        factory=Mock(return_value=S(ready=True,failure=''))
        node.recover.__func__.__globals__['BimanualWorker']=factory
        await node.recover(S(token=1))
        old.close.assert_called_once()
        factory.assert_called_once_with(node.validation_cfg,node.profile,preload_path_checks=True,grasp_checkpoint=old.grasp_checkpoint)
        self.assertIsNone(node.validation_result);self.assertIsNone(node.ground_recovery_plan)
        await node.recover(S(token=2))
        self.assertEqual(factory.call_count,1)

    async def test_gripper_recovery_settles_a_late_acquisition_without_dropping_live_authority(self):
        import ast
        from types import SimpleNamespace as S
        from unittest.mock import Mock
        path=Path(find_spec('snowbot_teleop.position').origin)
        cls=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='PositionOperations')
        method=next(n for n in cls.body if getattr(n,'name','')=='recover')
        scope={'SIDES':('left','right')}
        exec(compile(ast.Module(body=[method],type_ignores=[]),str(path),'exec'),scope)
        future=Mock();future.done.return_value=False
        states=[S(lease_active=False,state='CANCELED',epoch=11,measured_velocity=0.) for _ in range(2)]
        owner=S(node=S(profile=S(digest='profile')),acquire_future=future,
                reading=Mock(side_effect=lambda *a,**kw:states[0 if a[0]=='left' else 1]),
                lease=None,lease_instance='executor',config={'velocity_tolerance':.01},stop_unconfirmed=True)
        with self.assertRaisesRegex(RuntimeError,'ACQUIRE_UNRESOLVED'):await scope['recover'](owner,object())
        future.done.return_value=True
        future.result.return_value=S(ok=True,epoch=11,profile_hash='profile')
        with self.assertRaisesRegex(RuntimeError,'RECOVERY_EPOCH'):await scope['recover'](owner,object())
        self.assertIs(owner.acquire_future,future);self.assertTrue(owner.stop_unconfirmed)
        future.result.return_value.epoch=10
        await scope['recover'](owner,object())
        self.assertIsNone(owner.acquire_future);self.assertFalse(owner.stop_unconfirmed)

    async def test_feedback_fault_recovery_needs_completed_stop_and_fresh_device_hold(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock,AsyncMock
        from bindu_runtime.stop_evidence import stop_feedback_matches
        node=self.production('recover','stopped','begin_stop_wait')
        node.recover.__func__.__globals__['stop_feedback_matches']=stop_feedback_matches
        node.current=Mock();node.lease=node.tool_request=node.tool_goal=None
        node.profile=S(digest='profile',feedback_tolerances={});node.simulation=False;node.now=lambda:10.
        node.tool_cfg={'feedback_max_age':.2};node.names=('joint',)
        state=S(instance_id='executor',lease_epoch=10,lease_active=False,profile_hash='profile',
                simulated=False,state='FAILED',stop_failures=[],code='DEVICE_FEEDBACK_ERROR',
                feedback_stamp=9.99,reference=S(name=[]),joints=S(name=['joint'],velocity=[0.]))
        node.state=state;node.reading=Mock(return_value=(state,{'joint':0.}))
        node.position_ops=None;node.control=S(update_feedback=Mock(),rest=lambda _:True)
        node.feedback=Mock();node.stop_request=None;node.stop_diagnostic={'old':True}
        node.prepare_hold_reference=AsyncMock(return_value={'session':2,'sequence':268})
        with self.assertRaisesRegex(RuntimeError,'STOP_NOT_CONFIRMED'):
            await node.recover(S(token=1))
        request=node.begin_stop_wait('executor',10,False);request['requested']=9.8
        self.assertIsNone(node.stop_diagnostic)
        await node.stopped(request)
        self.assertEqual(node.confirmed_stop,request);self.assertIsNone(node.stop_request)
        await node.recover(S(token=1))
        self.assertEqual(node.prepare_hold_reference.await_count,2)
        for change in ({'lease_epoch':11},{'lease_active':True},{'stop_failures':['vendor_hold']},
                       {'state':'STOPPING'},{'code':'EMERGENCY_STOP'},{'feedback_stamp':9.7}):
            with self.subTest(change=change):
                saved={k:getattr(state,k) for k in change}
                for k,v in change.items():setattr(state,k,v)
                with self.assertRaisesRegex(RuntimeError,'STOP_NOT_CONFIRMED'):
                    await node.recover(S(token=2))
                for k,v in saved.items():setattr(state,k,v)
        node.prepare_hold_reference.side_effect=RuntimeError('SNOWBOT_DRIVE_FAULT')
        with self.assertRaisesRegex(RuntimeError,'DRIVE_FAULT'):await node.recover(S(token=3))
        node.prepare_hold_reference.side_effect=None
        async def owner_changed(*args,**kwargs):
            changed=S(**vars(state));changed.lease_epoch+=1
            node.reading.return_value=(changed,{'joint':0.})
            return {'session':2,'sequence':268}
        node.prepare_hold_reference.side_effect=owner_changed
        with self.assertRaisesRegex(RuntimeError,'STATE_CHANGED'):await node.recover(S(token=4))

    def test_execution_fault_and_stop_protocol_reach_pico_without_normal_cancel_fault(self):
        from types import SimpleNamespace as S
        node=self.production('on_motion_event');node.run_id='run';node.runtime_mode='real';node.instance_id='test'
        r=Rig();node.control=r.core;node.lease=('lease','instance');node.tool_request=None
        node.now=lambda:r.now
        def event(state,code,task='pico_test',component='execution',run='run'):
            return S(run_id=run,runtime_mode='real',task_id=task,component=component,state=state,
                     code=code,stamp=S(sec=1,nanosec=0))
        node.on_motion_event(event('STOPPING','CANCELED'))
        node.on_motion_event(event('FAILED','DEVICE_FEEDBACK_ERROR',task='another'))
        node.on_motion_event(event('FAILED','DEVICE_FEEDBACK_ERROR',run='other'))
        self.assertIsNone(r.core.first_fault)
        node.on_motion_event(event('DEVICE_COMMAND_DETAIL','SNOWBOT_PDO_STALE',task=''))
        node.on_motion_event(event('STOP_CHECK',json.dumps(dict(sequence=1932,
            observed=dict(command_sequence=1930,session_reason=7)))))
        self.assertEqual(r.core.first_fault['code'],'SNOWBOT_PDO_STALE')
        self.assertEqual(r.core.recovery_required,'device_inspection')
        self.assertEqual(node.stop_diagnostic['observed']['command_sequence'],1930)

    def test_prepare_fault_is_owned_by_command_id_and_not_hidden_by_lease_failure(self):
        from types import SimpleNamespace as S
        for command,owned in (('test_8_random_93',True),('another_8_random_93',False)):
            r=Rig();node=self.production('on_motion_event');node.control=r.core
            node.run_id='run';node.runtime_mode='real';node.instance_id='test'
            node.lease=('lease','executor');node.tool_request=None;node.now=lambda:r.now
            event=S(run_id='run',runtime_mode='real',component='execution',task_id='pico_prepare',
                command_id=command,state='STOPPING',code='COMMAND_TIMEOUT',stamp=S(sec=1,nanosec=0))
            node.on_motion_event(event)
            if owned:
                r.core.record_fault('RuntimeError:INVALID_LEASE',r.now+.1)
                self.assertEqual(r.core.first_fault['code'],'COMMAND_TIMEOUT')
            else:self.assertIsNone(r.core.first_fault)

    @staticmethod
    def production(*names):
        # Execute production methods with ROS/service/process boundaries inert.
        import ast
        from snowbot_hardware.parameters import parameters_for
        from types import SimpleNamespace as S
        path=Path(find_spec('snowbot_teleop.node').origin)
        node=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='PicoNode')
        methods=[n for n in node.body if getattr(n,'name','') in names]
        module=ast.Module(body=[ast.ClassDef(name='Actual',bases=[],keywords=[],body=methods,decorator_list=[])],type_ignores=[])
        namespace=dict(json=json,math=__import__('math'),time=S(monotonic=lambda:0.),parameters_for=parameters_for,
            temporary_capture_refusal=temporary_capture_refusal,
            seconds=lambda stamp:stamp.sec+stamp.nanosec*1e-9,
            joints_at_rest=lambda joints,names,tol:all(abs(v)<=tol for n,v in zip(joints.name,joints.velocity) if n in names),
            Trigger=S(Request=lambda:S()),ResetToolReference=S(Request=lambda **kw:S(**kw)),
            Superseded=type('Superseded',(RuntimeError,),{}))
        helpers=[n for n in ast.parse(path.read_text()).body
                 if getattr(n,'name','') in ('prepare_reference','prepare_start_tolerance')]
        exec(compile(ast.Module(body=helpers,type_ignores=[]),str(path),'exec'),namespace)
        exec(compile(ast.fix_missing_locations(module),str(path),'exec'),namespace)
        actual=namespace['Actual']()
        from unittest.mock import AsyncMock,Mock
        if 'reset_grasp_reference' not in names:actual.reset_grasp_reference=AsyncMock()
        if 'recover' in names:
            actual.validator=S(failure='')
            actual.tool_recover_client=S(call_async=Mock(return_value=S(ok=True)))
            actual.wait=AsyncMock(side_effect=lambda reply,timeout:reply)
        actual.position_contract=False  # Legacy-only cases opt in explicitly below.
        return actual

    async def test_preparation_reference_reset_keeps_original_scope_while_waiting_for_state(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock,AsyncMock
        owner=self.production('reset_grasp_reference')
        state=S(instance_id='execution',lease_epoch=4)
        replies=iter((S(ok=False,code='BIMANUAL_REFERENCE_STATE_PENDING'),S(ok=True,code='GRASP_REFERENCE_RESET_QUEUED')))
        owner.state,owner.current=state,Mock()
        owner.wait=AsyncMock(side_effect=lambda reply,timeout:reply)
        owner.grasp_reference_reset_client=S(call_async=Mock(side_effect=lambda request:next(replies)))
        async def pause():state.lease_epoch=5
        owner.pause=pause
        await owner.reset_grasp_reference(object())
        requests=[call.args[0] for call in owner.grasp_reference_reset_client.call_async.call_args_list]
        self.assertEqual([(r.execution_instance_id,r.epoch) for r in requests],[('execution',4)]*2)
        owner.wait.assert_awaited()

    async def test_superseded_preparation_does_not_accept_a_late_reference_reset(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock,AsyncMock
        owner=self.production('reset_grasp_reference')
        owner.state=S(instance_id='execution',lease_epoch=4)
        owner.current=Mock(side_effect=(None,None,RuntimeError('SUPERSEDED')))
        owner.wait=AsyncMock(return_value=S(ok=True,code='GRASP_REFERENCE_RESET_QUEUED'))
        owner.grasp_reference_reset_client=S(call_async=Mock(return_value=object()))
        owner.pause=AsyncMock()
        with self.assertRaisesRegex(RuntimeError,'SUPERSEDED'):
            await owner.reset_grasp_reference(object())
        owner.grasp_reference_reset_client.call_async.assert_called_once()
        owner.pause.assert_not_awaited()

    def test_display_keeps_fresh_coordinated_pair_during_one_hand_delivery(self):
        from types import SimpleNamespace as S
        rig=Rig();node=self.production('input_status');node.control=rig.core
        pair=rig.core.frames
        def raw(f):
            return S(side=f.side,source_id=f.source_id,seq=f.seq,valid=f.valid,
                     stamp=S(sec=int(f.stamp),nanosec=round((f.stamp-int(f.stamp))*1e9)))
        node.frames={f.side:raw(f) for f in pair}
        state=node.input_status(rig.now)
        self.assertTrue(state['raw_input_valid']);self.assertFalse(state['input_pair_pending'])
        node.frames['left']=raw(replace(pair[0],seq=pair[0].seq+1,stamp=rig.now+.01))
        state=node.input_status(rig.now+.01)
        self.assertTrue(state['raw_input_valid']);self.assertTrue(state['input_pair_pending'])
        self.assertEqual(node.control.frames,pair)  # Display never ingests/executes a partial pair.
        rig.step(.02)
        node.frames={f.side:raw(f) for f in rig.core.frames}
        self.assertFalse(node.input_status(rig.now)['input_pair_pending'])

    def test_display_never_hides_invalid_stale_future_or_changed_source_input(self):
        from types import SimpleNamespace as S
        for fault in ('invalid','stale','future','source','reversed','missing','pair_stale','no_pair'):
            with self.subTest(fault=fault):
                rig=Rig();node=self.production('input_status');node.control=rig.core
                node.frames={f.side:S(side=f.side,source_id=f.source_id,seq=f.seq,valid=True,
                    stamp=S(sec=1,nanosec=round((f.stamp-1)*1e9))) for f in rig.core.frames}
                left=node.frames['left'];now=rig.now
                if fault=='invalid':left.valid=False
                elif fault=='stale':now+=1.01
                elif fault=='future':left.stamp=S(sec=2,nanosec=0)
                elif fault=='source':left.source_id='reconnected'
                elif fault=='reversed':left.seq-=1
                elif fault=='missing':node.frames.pop('right')
                elif fault=='no_pair':node.control.frames=None
                else:
                    now+=1.01
                    for f in node.frames.values():
                        f.stamp=S(sec=int(now),nanosec=round((now-int(now))*1e9));f.seq+=1
                state=node.input_status(now)
                self.assertFalse(state['raw_input_valid']);self.assertFalse(state['input_pair_pending'])

    def test_unheld_policy_is_explicit_and_defaults_to_physical_grasp_checks(self):
        import ast
        from types import SimpleNamespace as S
        path=Path(find_spec('snowbot_teleop.node').origin)
        node=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='PicoNode')
        init=next(n for n in node.body if getattr(n,'name','')=='__init__')
        begin=next(i for i,n in enumerate(init.body) if isinstance(n,ast.Assign) and isinstance(n.targets[0],ast.Name) and n.targets[0].id=='policy')
        end=next(i for i,n in enumerate(init.body) if isinstance(n,ast.Assign) and isinstance(n.targets[0],ast.Subscript) and getattr(n.targets[0].slice,'value','')=='urdf')
        code=compile(ast.Module(body=init.body[begin:end],type_ignores=[]),str(path),'exec')
        for config,unheld in (({},False),({'measured_grasp_policy':'unheld_test'},True)):
            owner=S(tool_cfg=dict(right_axial_grasp={'configured':True},
                collision_scope='configured_tool_geometry',**config),position_contract=True,
                cfg={'tool_grasp_confirmation':'calibrated'})
            exec(code,{'self':owner})
            self.assertIs(owner.unheld_test,unheld)
        for source in ('measured_rigid','measured_axial'):
            owner=S(tool_cfg=dict(right_axial_grasp={'configured':True},
                collision_scope='configured_tool_geometry',
                grasp_reference_source=source),position_contract=True,cfg={'tool_grasp_confirmation':'operator'})
            exec(code,{'self':owner})
            self.assertTrue(owner.operator_grasp)
        owner.cfg={}
        with self.assertRaisesRegex(ValueError,'TOOL_GRASP_CONFIRMATION_CONFIG'):
            exec(code,{'self':owner})
        for policy in (None,False,'disabled','unheld'):
            with self.subTest(policy=policy),self.assertRaisesRegex(ValueError,'MEASURED_GRASP_POLICY'):
                exec(code,{'self':S(tool_cfg={'measured_grasp_policy':policy})})

    async def test_explicit_unheld_confirmation_uses_no_physical_grasp_or_calibration(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock,AsyncMock
        for unheld in (False,True):
            node=self.production('confirm_tool');node.unheld_test=unheld;node.operator_grasp=False
            node.control=S(load='unknown',virtual=True);node.operator_grasp_positions={'left':.1}
            node.feedback=lambda:PicoFeedback(0.,True,('unknown','unknown'))
            node.position_ops=S(config={'calibrated':False},reading=Mock(side_effect=AssertionError('physical grasp queried')))
            node.ready_client=S(service_is_ready=lambda:True,call_async=lambda _:S(success=True))
            node.current=Mock();node.wait=AsyncMock(side_effect=lambda value:value);node.event=Mock();node.tool_cfg={}
            intent=S(operation='select_mode')
            if not unheld:
                with self.assertRaisesRegex(RuntimeError,'UNCONFIRMED_UNCALIBRATED'):await node.confirm_tool(intent)
                continue
            await node.confirm_tool(intent)
            node.position_ops.reading.assert_not_called()
            self.assertFalse(node.control.virtual);self.assertEqual(node.operator_grasp_positions,{})
            evidence=json.loads(node.event.call_args.args[1])
            self.assertFalse(evidence['physical_grasp_confirmed']);self.assertFalse(evidence['grasp_sensor_confirmed'])
            node.control.load='held'
            with self.assertRaisesRegex(RuntimeError,'UNHELD_TEST_WITH_HELD_LOAD'):await node.confirm_tool(intent)
            node.control.load='unknown';node.feedback=lambda:PicoFeedback(0.,False)
            with self.assertRaisesRegex(RuntimeError,'REQUIRES_STOPPED'):await node.confirm_tool(intent)
            node.feedback=lambda:PicoFeedback(0.,True,healthy=False)
            with self.assertRaisesRegex(RuntimeError,'FEEDBACK_UNAVAILABLE'):await node.confirm_tool(intent)

    async def test_preflight_refusal_stays_read_only_for_supported_gripper_modes(self):
        from types import SimpleNamespace as S
        from unittest.mock import AsyncMock,Mock
        for position_contract in (False,True):
            for operation in ('confirm_tool','select_mode'):
                with self.subTest(position_contract=position_contract,operation=operation):
                    node=self.production('perform');node.position_contract=position_contract
                    node.automatic_close_grippers=position_contract;node.independent_grippers=False;node.unheld_test=False
                    node.cancel_tool=AsyncMock();node.current=Mock();node.event=Mock();node.now=lambda:1.
                    node.simulation=True;node.feedback=Mock(return_value=PicoFeedback(1.,True))
                    node.control=S(complete=Mock(),update_feedback=Mock(),teleop_mode='independent_arms',source='pico',seq=1)
                    node.confirm_tool=AsyncMock(side_effect=RuntimeError('BIMANUAL_GRASP_POSE'))
                    intent=S(token=1,operation=operation,value='bimanual_tool')
                    await node.perform(intent)
                    result=node.control.complete.call_args
                    self.assertFalse(result.args[1])
                    self.assertEqual(result.args[2],'RuntimeError:BIMANUAL_GRASP_POSE')
                    self.assertTrue(result.kwargs['rejected'])

    async def test_read_only_refusal_rechecks_feedback_after_stop_barrier(self):
        from unittest.mock import AsyncMock,Mock
        for feedback,standby in ((PicoFeedback(1.,True),True),(PicoFeedback(1.,False),False),
                (PicoFeedback(.5,True),False),(PicoFeedback(1.,True,healthy=False),False)):
            with self.subTest(feedback=feedback):
                r=Rig('unknown',guided=True,dual=True);r.click(1,1);intent=r.core.pending
                node=self.production('perform');node.position_contract=True;node.automatic_close_grippers=True
                node.independent_grippers=False;node.unheld_test=False;node.simulation=True;node.control=r.core
                node.cancel_tool=AsyncMock();node.current=Mock();node.event=Mock();node.now=lambda:r.now
                node.feedback=lambda:replace(feedback,stamp=r.now if feedback.stamp==1. else r.now-.5)
                node.confirm_tool=AsyncMock(side_effect=RuntimeError('BIMANUAL_GRASP_POSE'))
                # The stop coroutine completed after the timer cached its in-progress feedback.
                r.core.feedback=replace(r.core.feedback,stopped=False)
                await node.perform(intent)
                self.assertEqual(r.core.phase,'standby' if standby else 'locked')
                self.assertEqual(r.core.teleop_mode,'independent_arms')
                if feedback.healthy and feedback.stamp==1.:self.assertIsNone(r.core.pending)
                else:self.assertEqual(r.core.pending.operation,'stop')

    async def test_prepare_closes_after_arm_arrival_and_failure_cannot_report_success(self):
        from types import SimpleNamespace as S
        from unittest.mock import AsyncMock,Mock
        for failed in (False,True):
            node=self.production('perform');node.automatic_close_grippers=True;node.unheld_test=False
            node.position_contract=True;node.prepare_recovery=None;node.prepare_position_tolerance=.01
            node.simulation=False;node.simulation_demo=False;node.cfg={'prepare_duration':1.}
            node.control=S(load='unknown',complete=Mock());node.now=lambda:1.;node.current=Mock()
            node.cancel_tool=AsyncMock();node.validate_prepare=AsyncMock(return_value={'joint':.3})
            node.reading=lambda:(None,{'joint':0.});order=[]
            async def move(*a,**kw):order.append('arms_arrived')
            async def close(*a,**kw):
                self.assertEqual(order,['arms_arrived']);order.append('close')
                if failed:raise RuntimeError('PICO_GRIPPER_OPERATION_FAILED:left:POSITION_TIMEOUT')
            node.move=move;node.position_ops=S(close_grippers=close);node.event=Mock()
            intent=S(token=1,operation='prepare')
            await node.perform(intent)
            self.assertEqual(order,['arms_arrived','close'])
            self.assertIs(node.control.complete.call_args.args[1],not failed)
            self.assertFalse(node.control.complete.call_args.kwargs['rejected'])
            self.assertEqual(node.control.load,'unknown')

    async def test_mode_readiness_does_not_declare_physical_grasp_or_change_intent(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock,AsyncMock
        for success in (False,True):
            r=Rig('unknown',guided=True,dual=True);r.click(1,1)
            intent=r.core.pending;generation=r.core.generation
            node=self.production('confirm_tool','current');node.unheld_test=False;node.operator_grasp=True;node.control=r.core
            node.position_contract=True
            node.feedback=lambda:PicoFeedback(r.now,True,('unknown','unknown'))
            node.position_ops=S(reading=lambda _:S(measured_position=.02))
            node.ready_client=S(service_is_ready=lambda:True,call_async=lambda _:S(success=success,message='BIMANUAL_GRASP_POSE'))
            node.wait=AsyncMock(side_effect=lambda value:value);node.event=Mock();node.tool_cfg={};node.now=lambda:r.now
            if not success:
                with self.assertRaisesRegex(RuntimeError,'GRASP_POSE'):await node.confirm_tool(intent)
                self.assertEqual(r.core.load,'unknown');self.assertEqual(r.core.load_source,'unknown')
                self.assertEqual(r.core.generation,generation);continue
            await node.confirm_tool(intent)
            self.assertEqual((r.core.load,r.core.load_source),('unknown','unknown'))
            self.assertEqual(r.core.generation,generation);self.assertIs(r.core.pending,intent)
            self.assertEqual(r.core.pending.token,intent.token);node.current(intent)
            self.assertFalse(any(c.args[0]=='PICO_LOAD_OPERATOR_CONFIRMED' for c in node.event.call_args_list))
            await node.confirm_tool(intent)
            self.assertEqual(r.core.generation,generation);node.current(intent)
            self.assertTrue(r.core.complete(intent.token,True,now=r.now))
            self.assertEqual(r.core.load,'held');self.assertEqual(r.core.load_source,'teleop_mode')
            r.step();r.click(1,1)
            self.assertEqual(r.core.pending.value,'independent_arms')

    async def test_calibrated_position_preflight_preserves_confirmed_virtual_grasp(self):
        from types import SimpleNamespace as S
        from unittest.mock import AsyncMock,Mock
        for calibrated in (False,True):
            with self.subTest(calibrated=calibrated):
                node=self.production('confirm_tool');node.position_contract=True;node.operator_grasp=False
                node.unheld_test=False;node.control=S(load='unknown',virtual=True)
                node.feedback=lambda:PicoFeedback(1.,True,('closed','virtual'))
                node.current=Mock();node.wait=AsyncMock(side_effect=lambda value:value);node.event=Mock()
                node.position_ops=S(config={'calibrated':calibrated},reading=Mock())
                node.ready_client=S(service_is_ready=lambda:True,call_async=Mock(return_value=S(success=True)))
                node.tool_cfg={}
                if not calibrated:
                    with self.assertRaisesRegex(RuntimeError,'UNCONFIRMED_UNCALIBRATED'):
                        await node.confirm_tool(S(operation='follow'))
                    node.ready_client.call_async.assert_not_called()
                else:
                    await node.confirm_tool(S(operation='follow'))
                    self.assertTrue(node.control.virtual)
                    self.assertEqual(node.control.load,'unknown')
                    node.position_ops.reading.assert_not_called()

    async def test_a_return_is_empty_hand_in_every_load_and_grasp_policy(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock
        for load in ('empty','held','unknown'):
            for unheld in (False,True):
                with self.subTest(load=load,unheld=unheld):
                    node=self.production('validate_prepare');node.unheld_test=unheld;node.position_contract=True
                    node.control=S(load=load);node.prepare_positions={'joint':.2};node.names=('joint',)
                    node.tool_cfg={'resource_groups':['arm']};node.cfg={'prepare_collision_recovery':{'configured':True}}
                    node.current=Mock();node.event=Mock()
                    node.profile=S(groups={'arm':['joint']},limits={'joint':(-1.,1.)},feedback_tolerances={'joint_position':.002})
                    node.reading=lambda:(S(joints=S(name=['joint'],velocity=[0.])),{'joint':0.})
                    def submit(*args,**kwargs):node.validation_result=('result',1,{});return True
                    node.validator=S(ready=True,busy=False,startup_timeout=1.,submit=Mock(side_effect=submit))
                    intent=S(operation='prepare',token=1)
                    await node.validate_prepare(intent)
                    node.validator.submit.assert_called_once_with('prepare_recovery_path',1,{'joint':0.},[.2])
                    event=json.loads(node.event.call_args.args[1])
                    self.assertFalse(event['tool_present']);self.assertFalse(event['coupling_checked'])
                    self.assertFalse(event['load_state_checked'])

    async def test_gripper_path_selects_rod_from_mode_only(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock
        for mode in ('independent_arms','bimanual_tool'):
            for load in ('empty','held','unknown'):
                node=self.production('validate_prepare');node.position_contract=True;node.unheld_test=True
                node.control=S(load=load,teleop_mode=mode);node.prepare_positions=None;node.names=('joint',)
                node.tool_cfg={'resource_groups':['arm']};node.cfg={};node.current=Mock();node.event=Mock()
                node.profile=S(groups={'arm':['joint']},limits={'joint':(-1.,1.)},feedback_tolerances={'joint_position':.002})
                node.reading=lambda:(S(joints=S(name=['joint'],velocity=[0.])),{'joint':0.})
                def submit(*args,**kwargs):node.validation_result=('result',1,{});return True
                node.validator=S(ready=True,busy=False,startup_timeout=1.,submit=Mock(side_effect=submit))
                target={'finger':.1}
                await node.validate_prepare(S(token=1),target)
                expected='gripper_path' if mode=='bimanual_tool' else 'gripper_body_path'
                node.validator.submit.assert_called_once_with(expected,1,{'joint':0.},target)
                evidence=json.loads(node.event.call_args.args[1])
                self.assertEqual(evidence['tool_present'],mode=='bimanual_tool')
                self.assertFalse(evidence['load_state_checked'])

    async def test_operator_confirmation_does_not_make_gripper_position_a_grasp_sensor(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock
        for unheld in (False,True):
            node=self.production('tick');node.unheld_test=unheld;node.operator_grasp=True;node.base=None
            node.position_contract=True
            node.last_tick_at=None;node.waiters=set();node.validator=S(poll=lambda:None)
            node.state=None;node.tool_result=None;node.pending_intent=None;node.record_future=None;node.running=None
            node.now=lambda:0.;node.last_status=0.;node.last_context_at=0.
            node.control=S(teleop_mode='bimanual_tool',phase='following',context_at=0.,
                lock=Mock(),update_feedback=Mock(),tick=Mock(),drain=lambda:[])
            node.feedback=lambda:PicoFeedback(0.,True)
            node.operator_grasp_positions={'left':0.}
            node.position_ops=S(config={'position_tolerance':.003},reading=Mock(return_value=S(state='IDLE',measured_position=.01)))
            node.tick()
            node.control.lock.assert_not_called()
            node.position_ops.reading.assert_not_called()

    async def test_unheld_rejects_axial_switch_before_any_gripper_command(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock,AsyncMock
        for unheld,continuous in ((True,False),(False,True)):
            node=self.production('perform');node.unheld_test=unheld;node.operator_grasp=False
            node.position_contract=True;node.position_ops=S(axial_available=True,grasp_target=Mock())
            node.cancel_tool=AsyncMock();node.current=Mock();node.now=lambda:0.;node.event=Mock();node.simulation=False
            node.control=S(complete=Mock(),continuous_axial=continuous)
            await node.perform(S(operation='switch_grasp',token=1,value=True))
            node.position_ops.grasp_target.assert_not_called()
            args=node.control.complete.call_args.args
            self.assertFalse(args[1]);self.assertIn('PICO_AXIAL_GRASP_UNAVAILABLE',args[2])

    async def test_prepared_residual_inside_its_band_does_not_acquire_or_move_again(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock,AsyncMock
        for error in (.0284,.03501):
            with self.subTest(error=error):
                node=self.production('perform');node.position_contract=True;node.prepare_position_tolerance=.035
                node.control=S(complete=Mock());node.current=Mock();node.cancel_tool=AsyncMock()
                node.validate_prepare=AsyncMock(return_value={'joint':.2});node.reading=lambda:(None,{'joint':.2-error})
                node.prepare_recovery=None;node.cfg={'prepare_duration':12.};node.simulation_demo=False
                node.move=AsyncMock();node.now=lambda:0.;node.event=Mock();node.simulation=False
                await node.perform(S(operation='prepare',token=1))
                self.assertEqual(node.move.await_count,0 if error<.035 else 1)
                self.assertTrue(node.control.complete.call_args.args[1])
                if error<.035:
                    event=next(c for c in node.event.call_args_list if c.args[0]=='PICO_PREPARE_ALREADY_REACHED')
                    self.assertEqual(json.loads(event.args[1])['position_tolerance_rad'],.035)
                node.validate_prepare.assert_awaited_once()



    def test_primitive_library_initialization_uses_available_replay_api(self):
        import ast
        from types import SimpleNamespace as S
        from snowbot_teleop.replay import NAMES
        library = package_share('snowbot_bringup')/'config/robots/snowbot/primitive-library/manifest.json'
        manifest = json.loads(library.read_text())
        tree = ast.parse(Path(find_spec('snowbot_teleop.node').origin).read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'PicoNode')
        init = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '__init__')
        block = next(n for n in init.body if isinstance(n, ast.If) and
                     "self.cfg.get('primitive_library')" in ast.unparse(n.test))
        node = S(position_contract=True, cfg={'primitive_library': str(library)})
        exec(compile(ast.Module(body=[block], type_ignores=[]), '<primitive-init>', 'exec'), {'self': node})
        self.assertEqual(node.prepare_positions, dict(zip(NAMES, manifest['nodes'][manifest['home_node']])))
        self.assertFalse(hasattr(node, 'primitive_grasp_guard'))

    async def test_primitive_a_uses_dynamic_prepare_when_start_is_not_a_certified_node(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock, AsyncMock, patch
        from snowbot_teleop.replay import NAMES
        node=self.production('prepare_primitive')
        node.current=Mock();node.event=Mock();node.primitive_library='library';node.profile=object()
        node.names=NAMES;node.unheld_test=False;node.control=S(load='held',teleop_mode='bimanual_tool')
        node.reading=lambda:(S(joints=S(name=list(NAMES),velocity=[0.]*16)),dict.fromkeys(NAMES,0.))
        node.feedback=lambda:S(modes=('closed','closed'))
        node.move=AsyncMock()
        plan=S(points=((0.,)*16,(.1,)*16),names=NAMES,route=('FlipReverse','ScoopPrefixReverse'),
               duration=2.,validate_profile=Mock())
        with patch('snowbot_teleop.replay.plan_from_current',return_value=plan) as planner:
            self.assertTrue(await node.prepare_primitive(S(token=1)))
            self.assertEqual(planner.call_args.args[1],'Home')
            node.move.assert_awaited_once()
            self.assertIs(node.move.call_args.kwargs['replay'],plan)
        node.move.reset_mock()
        for mode in ('bimanual_tool','independent_arms'):
            node.control.teleop_mode=mode
            for load in ('held','unknown','empty'):
                node.control.load=load
                with patch('snowbot_teleop.replay.plan_from_current',
                           side_effect=RuntimeError('REPLAY_UNKNOWN_START')):
                    self.assertFalse(await node.prepare_primitive(S(token=1)))
                node.move.assert_not_awaited()
        with patch('snowbot_teleop.replay.plan_from_current',
                   side_effect=RuntimeError('REPLAY_MOTION_LIMITS')):
            with self.assertRaisesRegex(RuntimeError,'REPLAY_MOTION_LIMITS'):
                await node.prepare_primitive(S(token=1))


    def test_real_impedance_enable_flag_retains_simulation_and_mode_guards(self):
        import ast
        from types import SimpleNamespace as S
        path=Path(find_spec('snowbot_bringup.runtime.run_vr').origin)
        entry=next(n for n in ast.parse(path.read_text()).body if getattr(n,'name','')=='parse_args')
        block=next(n for n in entry.body if isinstance(n,ast.If) and isinstance(n.test,ast.Attribute)
                   and n.test.attr=='enable_commands')
        code=compile(ast.Module(body=[block],type_ignores=[]),str(path),'exec')
        def error(message):raise ValueError(message)
        for mode in ('position','impedance'):
            d=dict(runtime_mode='real',device_backend='snowbot_vendor',execution_mode=mode)
            exec(code,dict(args=S(enable_commands=True,physics=False,declare_load=None),d=d,parser=S(error=error)))
            self.assertEqual(d['command_enabled'],'true')
        for change in ({'runtime_mode':'simulation'},{'device_backend':'kinematic'},{'execution_mode':'unknown'}):
            d=dict(runtime_mode='real',device_backend='snowbot_vendor',execution_mode='impedance')
            d.update(change)
            with self.assertRaises(ValueError):
                exec(code,dict(args=S(enable_commands=True,physics=False,declare_load=None),d=d,parser=S(error=error)))


    async def test_impedance_a_selects_confirmed_nominal_node_under_compliance(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock,AsyncMock,patch
        from snowbot_teleop.replay import NAMES
        node=self.production('prepare_primitive');node.execution_mode='impedance'
        node.current=Mock();node.event=Mock();node.primitive_library='library';node.profile=object();node.names=NAMES
        node.control=S(load='held',teleop_mode='bimanual_tool');node.unheld_test=False
        node.tool_cfg=dict(reference_kinematics=True,max_reference_error=.15,
             impedance_reference_limits={'joint_reference_error_limits':dict.fromkeys(NAMES[2:],.25)})
        actual=dict.fromkeys(NAMES,.0);actual.update(dict.fromkeys(NAMES[2:],.2))
        node.reading=lambda:(S(joints=S(name=list(NAMES),velocity=[0.]*16)),actual)
        held={'positions':dict.fromkeys(NAMES,0.)};node.prepare_hold_reference=AsyncMock(return_value=held)
        node.primitive_grasp_guard=S(check=Mock());node.move=AsyncMock()
        plan=S(names=NAMES,points=((0.,)*16,(.1,)*16),duration=2.,route=('route',),validate_profile=Mock())
        with patch('snowbot_teleop.replay.plan_from_current',return_value=plan) as planner:
            self.assertTrue(await node.prepare_primitive(S(token=1)))
            self.assertEqual(planner.call_args.args[2],held['positions'])
        node.move.assert_awaited_once()
        node.move.reset_mock()
        node.control.load='unknown'
        with patch('snowbot_teleop.replay.plan_from_current',
                   side_effect=RuntimeError('REPLAY_UNKNOWN_START')):
            self.assertFalse(await node.prepare_primitive(S(token=1)))
        node.move.assert_not_awaited()

class LimitRecoveryRuntimeTests(unittest.IsolatedAsyncioTestCase):
    async def test_short_exit_is_revalidated_before_full_prepare(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock,AsyncMock
        for exit_fails in (False,True):
            node=ScenarioRuntimeTests.production('perform');node.position_contract=True
            node.prepare_limit_reference=None;node.prepare_recovery=None;node.prepare_position_tolerance=.035
            node.cfg={'prepare_duration':12.};node.simulation_demo=False;node.simulation=False
            node.control=S(complete=Mock());node.current=Mock();node.cancel_tool=AsyncMock()
            node.now=lambda:0.;node.event=Mock();node.reading=lambda:(None,{'joint':.55})
            calls=[]
            async def validate(intent):
                calls.append('validate')
                node.prepare_limit_reference={'positions':{'joint':.58}} if len(calls)==1 else None
                return {'joint':.546} if len(calls)==1 else {'joint':.43}
            async def move(intent,target,duration,**kw):
                calls.append(('move',duration,kw.get('limit_exit',False)))
                if exit_fails:raise RuntimeError('PREPARE_LIMIT_RECOVERY_NOT_INSIDE')
            node.validate_prepare=validate;node.move=move
            await node.perform(S(operation='prepare',token=1))
            expected=['validate',('move',3.,True)]
            if not exit_fails:expected+=['validate',('move',12.,False)]
            self.assertEqual(calls,expected)
            self.assertIs(node.control.complete.call_args.args[1],not exit_fails)

    def held_node(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock,AsyncMock
        node=ScenarioRuntimeTests.production('prepare_hold_reference')
        node.current=Mock();node.stop_request=None;node.now=lambda:10.
        node.names=('joint',);node.profile=S(digest='profile',feedback_tolerances={},limits={'joint':(-1.,1.)})
        node.tool_cfg={'feedback_max_age':.1}
        state=S(instance_id='executor',lease_active=False,lease_epoch=2,revision=3,
                state='CANCELED',stop_failures=[],joints=S(name=['joint'],velocity=[0.]))
        node.reading=Mock(return_value=(state,{'joint':-1.01}))
        value=dict(instance_id='executor',profile_hash='profile',lease_id='',epoch=2,
                   revision=3,stamp=9.99,positions={'joint':-.995},boot_id=1,session=2,sequence=3)
        node.prepare_hold_client=S(call_async=Mock(return_value=S()))
        node.wait=AsyncMock(return_value=S(success=True,message=json.dumps(value)))
        return node,state,value

    async def test_hold_snapshot_is_read_only_and_bound_to_current_state_and_pause(self):
        from types import SimpleNamespace as S
        node,state,value=self.held_node();intent=S(token=1)
        self.assertEqual(await node.prepare_hold_reference(intent),value)
        for key,bad in [('instance_id','other'),('epoch',1),('revision',4),('stamp',9.),
                        ('boot_id',0),('positions',{'joint':-1.001})]:
            changed=dict(value,**{key:bad});node.wait.return_value=S(success=True,message=json.dumps(changed))
            with self.subTest(key=key),self.assertRaisesRegex(RuntimeError,'HOLD_CHANGED'):
                await node.prepare_hold_reference(intent)
        node.wait.return_value=S(success=True,message=json.dumps(value));state.stop_failures=['vendor_hold']
        with self.assertRaisesRegex(RuntimeError,'HOLD_CHANGED'):await node.prepare_hold_reference(intent)
        state.stop_failures=[];state.state='FAILED';state.code='DEVICE_FEEDBACK_ERROR'
        with self.assertRaisesRegex(RuntimeError,'HOLD_CHANGED'):await node.prepare_hold_reference(intent)
        self.assertEqual(await node.prepare_hold_reference(intent,allow_feedback_fault=True),value)
        state.code='EMERGENCY_STOP'
        with self.assertRaisesRegex(RuntimeError,'HOLD_CHANGED'):
            await node.prepare_hold_reference(intent,allow_feedback_fault=True)
        state.state='CANCELED';state.lease_active=True;value.update(lease_id='lease',epoch=3);state.lease_epoch=3
        node.wait.return_value=S(success=True,message=json.dumps(value));lease=S(lease_id='lease',epoch=3)
        self.assertEqual(await node.prepare_hold_reference(intent,lease=lease,previous=dict(value)),value)
        with self.assertRaisesRegex(RuntimeError,'HOLD_CHANGED'):
            await node.prepare_hold_reference(intent,lease=lease,previous=dict(value,sequence=1))

    async def test_a_validates_raw_measured_prefix_then_the_legal_held_reference_path(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock,AsyncMock
        node=ScenarioRuntimeTests.production('validate_prepare')
        node.current=Mock();node.event=Mock();node.position_contract=True;node.unheld_test=False
        node.prepare_positions={'joint':.2};node.names=('joint',);node.cfg={}
        node.prepare_limit_max=.017453292519943295;node.prepare_position_tolerance=.035
        node.control=S(load='held');node.tool_cfg={'resource_groups':['arm']}
        node.profile=S(groups={'arm':['joint']},limits={'joint':(-1.,1.)},feedback_tolerances={'joint_position':.002})
        measured={'joint':-1.01};state=S(joints=S(name=['joint'],velocity=[0.]))
        node.reading=Mock(return_value=(state,measured))
        node.prepare_hold_reference=AsyncMock(return_value={'positions':{'joint':-.995}})
        def submit(operation,token,feedback,target,**extra):
            node.validation_result=('result',token,{});return True
        node.validator=S(ready=True,busy=False,startup_timeout=1.,submit=Mock(side_effect=submit))
        self.assertEqual(await node.validate_prepare(S(token=1)),{'joint':-.965})
        node.validator.submit.assert_called_once_with('prepare_limit_recovery_path',1,measured,[-.965],reference={'joint':-.995})
        self.assertEqual(measured,{'joint':-1.01})
        self.assertEqual(node.prepare_limit_start,measured)
        # Recorded R1 residual is inside hard limits: use the original A path.
        measured['joint']=.18645057082176208
        node.profile.limits['joint']=(-3.1417453292519943,3.1417453292519943)
        node.prepare_hold_reference.reset_mock();node.validator.submit.reset_mock()
        await node.validate_prepare(S(token=2))
        node.prepare_hold_reference.assert_not_awaited()
        node.validator.submit.assert_called_once_with('prepare_path',2,measured,[.2])

    async def movement(self,*,outward=False,hold_error=False,renew_delay=None,renew_ok=True,revoked=False,renew_epoch=2,
                       start_delta=0.,start_tolerance=None,submit_delay=None,observe_submit=True,submit_error=None,cancel_at=None):
        from types import SimpleNamespace as S
        from unittest.mock import Mock,AsyncMock
        from bindu_contracts.profile import Profile
        from bindu_execution.interpolation import Curve,State
        import uuid
        node=ScenarioRuntimeTests.production('move');clock=[0.]
        node.move.__func__.__globals__.update(time=S(monotonic=lambda:clock[0]),Curve=Curve,State=State,
            Lease=S(Request=lambda **kw:S(**kw)),SubmitMotion=S(Request=lambda **kw:S(**kw)),
            MotionCommand=lambda **kw:S(**kw),JointTrajectoryPoint=lambda **kw:S(**kw),
            Duration=lambda **kw:S(**kw),stamp=lambda t:t,uuid=uuid)
        lo=-.5974532925199433;actual=-.6073604822158813;held=-.596094062733137
        node.profile=Profile('fixture',{'arm':['joint']},{'joint':[lo,2.61]},.5,('joint_position',),'profile',
            feedback_tolerances={'joint_position':.002})
        state=S(instance_id='executor',lease_active=True,lease_epoch=2,revision=1,
                command_id='',state='CANCELED',joints=S(name=['joint'],velocity=[0.]))
        node.state=state;node.current=Mock();node.feedback=lambda:S(stopped=True)
        def current(intent):
            if cancel_at is not None and clock[0]>=cancel_at:raise RuntimeError('PICO_OPERATION_SUPERSEDED')
        node.current.side_effect=current;node.test_clock=clock;node.submit_futures=[]
        node.prepare_position_tolerance=.035;node.prepare_limit_max=.017453292519943295
        node.prepare_limit_reference={'positions':{'joint':held}};node.prepare_limit_start={'joint':actual}
        node.prepare_hold_reference=AsyncMock(return_value=node.prepare_limit_reference)
        if hold_error:node.prepare_hold_reference.side_effect=RuntimeError('PREPARE_LIMIT_RECOVERY_HOLD_CHANGED')
        node.position_contract=True;node.names=('joint',);node.groups=('arm',);node.cfg={};node.simulation=False
        if start_tolerance is not None:node.cfg['prepare_start_tolerance_rad']=start_tolerance
        node.instance_id='pico';node.runtime_mode='real'
        lease=S(ok=True,lease_id='lease',epoch=2,profile_hash='profile')
        node.renew_times=[]
        def renew(request):
            if request.operation=='acquire':return lease
            issued=clock[0]
            node.renew_times.append(issued)
            return S(done=lambda:clock[0]>=issued+(renew_delay or 0.),
                result=lambda:S(**dict(vars(lease),epoch=renew_epoch)) if renew_ok else S(ok=False,code='INVALID_LEASE'))
        node.leases=S(call_async=Mock(side_effect=renew))
        async def wait(value,*args,**kw):
            if hasattr(value,'done'):
                while not value.done():clock[0]=round(clock[0]+.04,6)
                return value.result()
            return value
        node.wait=wait;node.now=lambda:clock[0];node.release_lease=AsyncMock();node.event=Mock()
        def reading():
            q=actual+start_delta if not clock[0] else actual-.003 if outward else .2
            state.feedback_stamp=S(sec=int(clock[0]),nanosec=0)
            if revoked and clock[0]>=.12:
                state.command_id=node.submit.call_async.call_args.args[0].command.command_id
                state.state='STOPPING';state.code='COMMAND_TIMEOUT';state.lease_active=False;state.lease_epoch=3
            return state,{'joint':q}
        node.reading=reading
        async def pause():
            clock[0]=(round(clock[0]+.04,6) if renew_delay is not None or submit_delay is not None else
                      .01 if outward or not clock[0] else 41.)
        def submit(request):
            issued=clock[0];revision=state.revision+1
            if observe_submit:
                self.assertEqual(request.command.expected_revision,state.revision)
                state.command_id=request.command.command_id;state.revision=revision
            future=S(done=lambda:clock[0]>=issued+(submit_delay or 0.),
                result=lambda:S(accepted=not submit_error,code=submit_error,revision=revision))
            node.submit_futures.append(future)
            return future
        node.pause=pause;node.submit=S(call_async=Mock(side_effect=submit))
        self.last_movement=node
        try:await node.move(S(token=1,operation='prepare'),{'joint':.2},12.)
        finally:node.release_lease.assert_awaited_once()
        return node,held,lo

    async def test_slow_renewal_does_not_block_fresh_targets_and_failures_still_stop(self):
        node,_,_=await self.movement(renew_delay=.24)
        stamps=[c.args[0].command.stamp for c in node.submit.call_async.call_args_list]
        self.assertGreaterEqual(len([t for t in stamps if t<.24]),5)
        self.assertLess(max(b-a for a,b in zip(stamps,stamps[1:])),.1)
        for kwargs,reason in (({'renew_ok':False},'INVALID_LEASE'),({'revoked':True},'COMMAND_TIMEOUT'),
                              ({'renew_epoch':3},'PICO_EXECUTION_AUTHORITY_CHANGED'),
                              ({'renew_delay':10.},'PICO_LEASE_RENEW_TIMEOUT')):
            with self.subTest(kwargs=kwargs),self.assertRaisesRegex(RuntimeError,reason):
                await self.movement(**dict({'renew_delay':.24},**kwargs))

    async def test_slow_submit_reply_uses_exact_executor_receipt_without_starving_targets(self):
        node,_,_=await self.movement(renew_delay=.12,submit_delay=.24)
        stamps=[c.args[0].command.stamp for c in node.submit.call_async.call_args_list if c.args[0].command.stamp<1.]
        self.assertGreaterEqual(len(stamps),20)
        self.assertLess(max(b-a for a,b in zip(stamps,stamps[1:])),.25-.12)
        renewals=[c for c in node.leases.call_async.call_args_list if c.args[0].operation=='renew']
        self.assertGreaterEqual(len(renewals),4)
        self.assertLess(max(b-a for a,b in zip(node.renew_times,node.renew_times[1:])),.241)
        for kwargs,code in (({'observe_submit':False,'submit_delay':10.},'PICO_SUBMIT_UNCONFIRMED'),
                            ({'submit_delay':.16,'submit_error':'STALE_REVISION'},'STALE_REVISION')):
            with self.subTest(code=code),self.assertRaisesRegex(RuntimeError,code):
                await self.movement(**kwargs)
        self.last_movement.release_lease.assert_awaited_once()

    async def test_cancellation_during_slow_submit_reply_releases_without_waiting_or_reviving(self):
        with self.assertRaisesRegex(RuntimeError,'PICO_OPERATION_SUPERSEDED'):
            await self.movement(submit_delay=.24,cancel_at=.12)
        node=self.last_movement;count=node.submit.call_async.call_count
        self.assertLess(node.now(),.16)
        node.release_lease.assert_awaited_once()
        node.test_clock[0]+=1.
        for future in node.submit_futures:
            self.assertTrue(future.done());future.result()
        self.assertEqual(node.submit.call_async.call_count,count)
        node.release_lease.assert_awaited_once()

    async def test_recovery_starts_at_confirmed_reference_and_returns_inside_hard_bounds(self):
        node,held,lo=await self.movement()
        command=node.submit.call_async.call_args.args[0].command
        self.assertAlmostEqual(command.positions[0],held)
        self.assertGreaterEqual(command.positions[0],lo)
        node.prepare_hold_reference.assert_awaited_once()

    async def test_recovery_start_snapshot_uses_same_bounded_tolerance(self):
        await self.movement(start_delta=.0002,start_tolerance=.001)
        for delta,tolerance in ((.0002,None),(.00101,.001)):
            with self.subTest(delta=delta,tolerance=tolerance),self.assertRaisesRegex(RuntimeError,'PREPARE_STATE_CHANGED'):
                await self.movement(start_delta=delta,start_tolerance=tolerance)

    async def test_changed_hold_or_further_outward_motion_aborts_and_releases(self):
        for options,reason in [({'hold_error':True},'HOLD_CHANGED'),({'outward':True},'MOVED_OUTWARD')]:
            with self.subTest(options=options),self.assertRaisesRegex(RuntimeError,reason):
                await self.movement(**options)


class PrepareReturnPathTests(unittest.IsolatedAsyncioTestCase):
    @staticmethod
    def move_module():
        # Execute the production method with inert ROS messages and services.
        import ast,types,math
        from types import SimpleNamespace as S
        from bindu_execution.interpolation import Curve,State
        from snowbot_hardware.parameters import parameters_for
        from bindu_runtime.stop_evidence import sampled_nominal_reference
        module=types.ModuleType('offline_prepare_move')
        module.__dict__.update(json=json,math=math,time=__import__('time'),uuid=__import__('uuid'),Curve=Curve,State=State,
            parameters_for=parameters_for,sampled_nominal_reference=sampled_nominal_reference,Lease=S(Request=lambda **kw:S(**kw)),
            SubmitMotion=S(Request=lambda **kw:S(**kw)),MotionCommand=S,JointTrajectoryPoint=S,Duration=S,
            stamp=lambda value:S(sec=int(value),nanosec=round((value-int(value))*1e9)),
            seconds=lambda value:value.sec+value.nanosec/1e9,
            joints_at_rest=lambda joints,names,tol:all(abs(v)<=tol for n,v in zip(joints.name,joints.velocity) if n in names))
        path=Path(find_spec('snowbot_teleop.node').origin);tree=ast.parse(path.read_text())
        node=next(x for x in tree.body if isinstance(x,ast.ClassDef) and x.name=='PicoNode')
        helpers=[x for x in tree.body if getattr(x,'name','') in ('prepare_reference','prepare_start_tolerance')]
        method=next(x for x in node.body if getattr(x,'name','')=='move')
        exec(compile(ast.Module(body=helpers+[method],type_ignores=[]),str(path),'exec'),module.__dict__)
        module.PicoNode=S(move=module.move)
        return module

    async def arrival(self,operation='prepare',error=.0284,velocity=0.,fresh=True,stop_error=None,parameters=None,tick_period=1.,position_contract=True,primitive=False,cancel_at=None,execution_mode='position'):
        from types import SimpleNamespace as S
        from unittest.mock import Mock,AsyncMock,patch
        module=self.move_module()
        from bindu_contracts.profile import Profile
        from snowbot_teleop.control import PicoIntent
        from tests.support.paths import BINDU_ROOT as ROOT
        profile=Profile.load(package_share("snowbot_bringup")/'config/robots/snowbot/snowbot_real_position.json')
        clock=[0.];joint='arm_l_joint_1';target=.2
        lease=S(ok=True,lease_id='test',epoch=1,profile_hash=profile.digest)
        state=S(instance_id='executor',profile_hash=profile.digest,
            lease_active=True,lease_epoch=1,revision=0,command_id='',state='HOLDING',
            joints=S(name=[joint],velocity=[velocity]),reference=S(name=[joint],position=[target]))
        def reading():
            state.feedback_stamp=module.stamp(clock[0] if fresh else 0.)
            state.reference.header=S(stamp=module.stamp(max(0.,clock[0]-.001)))
            return state,{joint:target-error}
        async def pause():clock[0]+=tick_period
        async def wait(value):return value
        renewal=S(done=lambda:True,result=lambda:lease)
        node=S(current=Mock(),feedback=lambda:S(stopped=True),prepare_position_tolerance=.035,
            reset_grasp_reference=AsyncMock(),
            leases=S(call_async=lambda r:lease if r.operation=='acquire' else renewal),wait=wait,instance_id='test',groups=('left_arm',),
            state=state,reading=reading,names=(joint,),position_contract=position_contract,profile=profile,
            event=Mock(),simulation=False,now=lambda:clock[0],runtime_mode='real',pause=pause,
            runtime_parameters=parameters,submit=S(call_async=Mock(return_value=S(done=lambda:True,result=lambda:S(accepted=True,revision=0)))),
            release_lease=AsyncMock(side_effect=stop_error))
        node.execution_mode=execution_mode
        node.tool_cfg=dict(reference_kinematics=True,max_reference_error=.15,feedback_max_age=.2,
            impedance_reference_limits={'joint_reference_error_limits':{joint:.25}})
        node.primitive_nominal_hold={'positions':{joint:target-error}}
        node.prepare_hold_reference=AsyncMock(return_value=node.primitive_nominal_hold)
        replay=None
        if primitive:
            from snowbot_teleop.replay import Replay
            replay=Replay((0.,2.),((target-error,),(target,)),(joint,),'tool','kin',1.)
            def current(intent):
                if cancel_at is not None and clock[0]>=cancel_at:raise RuntimeError('PREPARE_RELEASED')
            node.current=current
        with patch.object(module,'time',S(monotonic=lambda:clock[0])):
            try:await module.PicoNode.move(node,PicoIntent(1,operation),{joint:target},2.,replay=replay)
            finally:node.release_lease.assert_awaited_once()
        return node


    async def test_held_a_stream_uses_certified_samples_and_cancellation_stops_it(self):
        node=await self.arrival(error=.001,primitive=True,tick_period=.005)
        commands=[call.args[0].command for call in node.submit.call_async.call_args_list]
        stamps=[command.stamp.sec+command.stamp.nanosec/1e9 for command in commands]
        self.assertTrue(all(abs((b-a)-.02)<1e-8 for a,b in zip(stamps,stamps[1:])))
        self.assertTrue(all(c.valid_for==.25 for c in commands))
        self.assertAlmostEqual(commands[0].positions[0],.199)
        self.assertAlmostEqual(commands[-1].positions[0],.2)
        self.assertFalse(hasattr(node,'primitive_grasp_guard'))
        with self.assertRaisesRegex(RuntimeError,'REPLAY_STREAM_LATE'):
            await self.arrival(error=.001,primitive=True,tick_period=.1)
        with self.assertRaisesRegex(RuntimeError,'PREPARE_RELEASED'):
            await self.arrival(error=.001,primitive=True,tick_period=.005,cancel_at=.2)

    async def test_impedance_a_accepts_nominal_arrival_with_measured_rest(self):
        node=await self.arrival(error=.04,primitive=True,tick_period=.005,execution_mode='impedance')
        arrivals=[json.loads(c.args[1]) for c in node.event.call_args_list if c.args[0]=='PICO_ACTUATOR_ARRIVED']
        self.assertEqual(arrivals[0]['arrival_basis'],'sampled_nominal')
        self.assertFalse(arrivals[0]['measured_at_goal'])
        self.assertTrue(arrivals[0]['measured_at_rest'])
        node.prepare_hold_reference.assert_awaited_once()

    async def test_configured_prepare_period_and_ttl_reach_real_motion_messages(self):
        from snowbot_hardware.parameters import load_parameters
        parameters=load_parameters()
        parameters['pico'].update(prepare_command_period_s=.02,prepare_command_ttl_s=.3)
        node=await self.arrival(parameters=parameters,tick_period=.005)
        commands=[call.args[0].command for call in node.submit.call_async.call_args_list]
        self.assertGreater(len(commands),70)
        self.assertTrue(all(command.valid_for==.3 for command in commands))
        stamps=[command.stamp.sec+command.stamp.nanosec/1e9 for command in commands]
        gaps=[b-a for a,b in zip(stamps,stamps[1:])]
        self.assertGreaterEqual(min(gaps),.019999)
        self.assertLessEqual(max(gaps),.025001)
        parameters['pico']['reference_lookahead_s']=1.2
        node=await self.arrival(parameters=parameters,position_contract=False)
        point=node.submit.call_async.call_args_list[0].args[0].command.points[0]
        self.assertEqual((point.time_from_start.sec,point.time_from_start.nanosec),(1,200000000))

    async def test_prepare_arrival_tolerance_is_separate_from_other_moves_and_stop(self):
        node=await self.arrival()
        arrived=[json.loads(c.args[1]) for c in node.event.call_args_list if c.args[0]=='PICO_ACTUATOR_ARRIVED']
        self.assertEqual(len(arrived),1);self.assertEqual(arrived[0]['position_tolerance_rad'],.035)
        await self.arrival(error=.0284, primitive=True, tick_period=.005)
        for options in ({'error':.03501},{'operation':'load'},{'velocity':.00501},{'fresh':False}):
            with self.subTest(options=options),self.assertRaisesRegex(RuntimeError,'PICO_ACTUATOR_FEEDBACK_TIMEOUT'):
                await self.arrival(**options)
        await self.arrival(operation='load',error=.0019)
        with self.assertRaisesRegex(RuntimeError,'STOP_UNCONFIRMED'):
            await self.arrival(stop_error=RuntimeError('STOP_UNCONFIRMED'))

    def node(self, load='unknown', mode='bimanual_tool', *, error=None, changed=False):
        from types import SimpleNamespace as S
        from unittest.mock import Mock
        node=S(control=S(load=load,teleop_mode=mode),position_contract=True,unheld_test=False,
            prepare_positions={'joint':.2},names=('joint',),cfg={},
            tool_cfg={'resource_groups':['arm']},profile=S(groups={'arm':('joint',)},
                limits={'joint':(-1.,1.)},feedback_tolerances={'joint_position':.002}),
            current=Mock(),event=Mock(),validation_result=None)
        state=S(joints=S(name=['joint'],velocity=[0.]))
        readings=[(state,{'joint':0.}),(state,{'joint':.01 if changed else 0.})]
        node.reading=Mock(side_effect=readings)
        def submit(operation, token, feedback, target, **kwargs):
            node.validation_result=('error',token,error) if error else ('result',token,{})
            return True
        node.validator=S(ready=True,busy=False,startup_timeout=1.,submit=Mock(side_effect=submit))
        return node

    async def test_return_does_not_preserve_captured_grasp_in_either_mode_or_any_load_state(self):
        from snowbot_teleop.node import PicoNode
        from snowbot_teleop.control import PicoIntent
        for load in ('empty','held','unknown'):
            for mode in ('independent_arms','bimanual_tool'):
                for unheld in (False,True):
                    with self.subTest(load=load,mode=mode,unheld=unheld):
                        node=self.node(load,mode);node.unheld_test=unheld
                        self.assertEqual(await PicoNode.validate_prepare(node,PicoIntent(1,'prepare')),{'joint':.2})
                        node.validator.submit.assert_called_once_with('prepare_path',1,{'joint':0.},[.2])
                        self.assertEqual((node.control.load,node.control.teleop_mode),(load,mode))
                        event=json.loads(node.event.call_args.args[1])
                        self.assertFalse(event['load_state_checked']);self.assertFalse(event['coupling_checked'])
                        self.assertFalse(event['tool_present'])

    async def test_return_still_propagates_robot_collision_limits_and_changed_feedback(self):
        from snowbot_teleop.node import PicoNode
        from snowbot_teleop.control import PicoIntent
        for error in ('SELF_COLLISION: arm / body t=0.5','PREPARE_JOINT_LIMIT'):
            with self.subTest(error=error),self.assertRaisesRegex(RuntimeError,error):
                await PicoNode.validate_prepare(self.node(error=error),PicoIntent(1,'prepare'))
        with self.assertRaisesRegex(RuntimeError,'PREPARE_STATE_CHANGED'):
            await PicoNode.validate_prepare(self.node(changed=True),PicoIntent(1,'prepare'))

    async def test_configured_prepare_start_tolerance_accepts_small_change_but_rejects_excess(self):
        from snowbot_teleop.node import PicoNode
        from snowbot_teleop.control import PicoIntent
        for gripper in (False,True):
            for delta in (.00019174768,.001,.001001):
                with self.subTest(delta=delta,gripper=gripper):
                    node=self.node();node.cfg['prepare_start_tolerance_rad']=.001
                    state=next(node.reading.side_effect)[0]
                    node.reading.side_effect=[(state,{'joint':0.}),(state,{'joint':delta})]
                    target={'joint':.2} if gripper else None
                    if delta<=.001:
                        self.assertEqual(await PicoNode.validate_prepare(node,PicoIntent(1,'prepare'),target),{'joint':.2})
                    else:
                        with self.assertRaisesRegex(RuntimeError,'PREPARE_STATE_CHANGED'):
                            await PicoNode.validate_prepare(node,PicoIntent(1,'prepare'),target)
                        detail=json.loads(node.event.call_args.args[1])
                        self.assertEqual((detail['joint'],detail['delta_rad'],detail['tolerance_rad']),
                                         ('joint',delta,.001))

    async def test_prepare_start_relaxation_keeps_default_and_stationary_guards(self):
        from snowbot_teleop.node import PicoNode
        from snowbot_teleop.control import PicoIntent
        from types import SimpleNamespace as S
        for configured,gripper,velocity in ((False,False,0.),(False,True,0.),(True,False,.00501),(True,True,.00501)):
            with self.subTest(configured=configured,gripper=gripper,velocity=velocity):
                node=self.node()
                if configured:node.cfg['prepare_start_tolerance_rad']=.001
                node.reading.side_effect=[(S(joints=S(name=['joint'],velocity=[0.])),{'joint':0.}),
                    (S(joints=S(name=['joint'],velocity=[velocity])),{'joint':.0002})]
                code='PREPARE_REQUIRES_STATIONARY_START' if velocity else 'PREPARE_STATE_CHANGED'
                with self.assertRaisesRegex(RuntimeError,code):
                    await PicoNode.validate_prepare(node,PicoIntent(1,'prepare'),{'joint':.2} if gripper else None)

    async def test_prepare_start_tolerance_rejects_invalid_configuration(self):
        from snowbot_teleop.node import PicoNode
        from snowbot_teleop.control import PicoIntent
        for tolerance in (0.,-.001,float('nan'),float('inf'),.002001):
            with self.subTest(tolerance=tolerance):
                node=self.node();node.cfg['prepare_start_tolerance_rad']=tolerance
                with self.assertRaisesRegex(RuntimeError,'PICO_PREPARE_START_TOLERANCE_INVALID'):
                    await PicoNode.validate_prepare(node,PicoIntent(1,'prepare'))
                node.validator.submit.assert_not_called()

    async def test_boundary_reference_is_explicit_and_does_not_replace_measured_validation_start(self):
        from snowbot_teleop.node import PicoNode
        from snowbot_teleop.control import PicoIntent
        from types import SimpleNamespace as S
        node=self.node('empty');measured={'joint':-1.00007}
        state=S(joints=S(name=['joint'],velocity=[0.]))
        node.reading.side_effect=[(state,measured),(state,measured)]
        await PicoNode.validate_prepare(node,PicoIntent(1,'prepare'))
        node.validator.submit.assert_called_once_with('prepare_path',1,measured,[.2],reference={'joint':-1.})
        self.assertEqual(measured,{'joint':-1.00007})

    def test_boundary_reference_preserves_strict_curve_limits_and_rejects_large_overrun(self):
        from snowbot_teleop.node import prepare_reference
        from bindu_execution.interpolation import Curve,State
        from bindu_contracts.profile import Profile
        from tests.support.paths import BINDU_ROOT as ROOT
        profile=Profile.load(package_share("snowbot_bringup")/'config/robots/snowbot/snowbot_real_position.json')
        for name in ('arm_r_joint_3','arm_l_joint_3'):
            lo,hi=profile.limits[name]
            for actual,target in ((lo-.00007,lo+.1),(hi+.00007,hi-.1)):
                measured={name:actual};legal=prepare_reference(profile,[name],measured)
                curve=Curve.between(0.,2.,State.rest(legal.values()),State.rest([target]))
                self.assertTrue(curve.valid(profile,[name]))
                for t in np.linspace(0,2,101):self.assertTrue(lo<=curve.sample(t).q[0]<=hi)
                self.assertEqual(measured[name],actual)
            for bad in (lo-.002001,hi+.002001,float('nan')):
                with self.assertRaisesRegex(RuntimeError,'PREPARE_START_JOINT_LIMIT'):
                    prepare_reference(profile,[name],{name:bad})


class OperationCancellationTests(unittest.IsolatedAsyncioTestCase):
    async def test_gripper_handoff_refreshes_stopped_feedback_before_held_a_completion(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock,AsyncMock
        from snowbot_teleop.node import PicoNode
        r=Rig(guided=True,direct=True);r.triggers=[1.,0.];r.step()
        r.triggers=[0.,0.];r.step();r.stopped=False;r.buttons[1]=1;r.step()
        intent=r.core.pending
        node=S(control=r.core,position_contract=True,cancel_tool=AsyncMock(),current=Mock(),
            position_ops=S(move_grippers=AsyncMock()),now=lambda:r.now,
            feedback=Mock(return_value=PicoFeedback(r.now,True,('closed','closed'))),
            event=Mock(),simulation=True)
        await PicoNode.perform(node,intent)
        node.feedback.assert_called_once()
        self.assertEqual(r.core.reason,'PREPARE_HOLD_AFTER_GRIPPER_STOP')
        self.assertEqual(r.core.a_since,r.now)

    async def run_superseded(self, cleanup_error=None):
        from types import SimpleNamespace as S
        from unittest.mock import Mock,AsyncMock
        from snowbot_teleop.node import PicoNode,Superseded
        r=Rig(guided=True,direct=True);r.triggers=[1.,0.];old=r.step()[-1]
        r.triggers=[0.,0.];replacement=r.step()[-1]
        node=S(control=r.core,cancel_tool=AsyncMock(side_effect=[None,cleanup_error]),
            current=Mock(side_effect=Superseded('PICO_OPERATION_SUPERSEDED')),
            now=lambda:r.now,event=Mock(),simulation=True)
        await PicoNode.perform(node,old)
        self.assertEqual(node.cancel_tool.await_count,2)
        self.assertEqual(r.core.pending,replacement)
        return node

    async def test_normal_replacement_is_cancellation_and_keeps_new_intent(self):
        node=await self.run_superseded()
        self.assertEqual(node.event.call_args.args[0],'PICO_OPERATION_CANCELED')
        value=json.loads(node.event.call_args.args[1])
        self.assertEqual(value['outcome'],'superseded')
        self.assertNotIn('success',value)

    async def test_failed_stop_during_replacement_remains_a_real_error(self):
        node=await self.run_superseded(RuntimeError('PICO_STOP_UNCONFIRMED'))
        self.assertEqual(node.event.call_args.args[0],'PICO_OPERATION_RESULT')
        value=json.loads(node.event.call_args.args[1])
        self.assertFalse(value['success']);self.assertIn('PICO_STOP_UNCONFIRMED',value['reason'])


class DualModeTests(unittest.TestCase):
    def test_default_normal_accepts_one_hand_and_releases_independently(self):
        r=Rig('unknown',guided=True,dual=True)
        self.assertEqual(r.core.teleop_mode,'independent_arms')
        r.grips=[1.,0.];r.step();self.assertEqual(r.core.pending.operation,'follow')
        r.complete();r.stopped=False;r.grips=[1.,1.];r.step()
        r.click(0,0);self.assertIsNone(r.core.pending)
        r.grips=[0.,1.];self.assertEqual(r.step(),[])
        self.assertEqual(r.core.phase,'following')
        r.grips=[0.,0.];r.step();self.assertEqual(r.core.pending.operation,'stop')

    def test_b_switches_only_at_released_rest_and_does_not_follow(self):
        r=Rig(guided=True,dual=True);r.click(1,1)
        self.assertEqual((r.core.pending.operation,r.core.pending.value),('select_mode','bimanual_tool'))
        self.assertEqual(r.core.teleop_mode,'independent_arms')
        r.complete();self.assertEqual(r.core.teleop_mode,'bimanual_tool')
        self.assertEqual(r.core.phase,'standby');self.assertIsNone(r.core.pending)
        r.step();r.click(1,1);r.complete()
        self.assertEqual(r.core.teleop_mode,'independent_arms')

    def test_rejected_tool_mode_keeps_normal_mode_and_specific_reason(self):
        r=Rig(guided=True,dual=True);r.click(1,1)
        r.core.complete(r.core.pending.token,False,'RuntimeError:BIMANUAL_GRASP_POSE',r.now,rejected=True)
        self.assertEqual(r.core.teleop_mode,'independent_arms')
        self.assertEqual(r.core.phase,'standby');self.assertIsNone(r.core.pending)

    def test_old_load_cannot_block_independent_takeover_or_mode_selection(self):
        for load in ('held','unknown','empty'):
            r=Rig(load,guided=True,dual=True);r.grips=[1.,0.];r.step()
            self.assertEqual(r.core.pending.operation,'follow')
            r.complete();r.grips=[0.,0.];r.step();r.complete();r.step()
            r.click(1,1);r.complete()
            self.assertTrue(r.core.snapshot()['tool_collision_enabled'])
            self.assertFalse(r.core.snapshot()['load_state_checked'])
            r.step();r.click(1,1);r.complete()
            self.assertEqual((r.core.load,r.core.load_source),('empty','teleop_mode'))
            self.assertFalse(r.core.snapshot()['tool_collision_enabled'])

    def test_position_mode_stick_click_does_not_enter_loading_workflow(self):
        r=Rig('held',guided=True,dual=True)
        self.assertEqual(r.click(0,2),[])
        self.assertEqual(r.core.phase,'standby')

    def test_mode_key_while_following_only_stops_then_requires_new_press(self):
        r=Rig(guided=True,dual=True);r.grips=[1.,0.];r.step();r.complete()
        r.click(1,1);self.assertEqual(r.core.pending.operation,'stop');r.complete()
        self.assertEqual(r.core.teleop_mode,'independent_arms')
        r.grips=[0.,0.];r.step();self.assertIsNone(r.core.pending)
        r.click(1,1);self.assertEqual(r.core.pending.operation,'select_mode')

    def test_b_and_release_keep_mode_request_visible_without_queueing_switch(self):
        for coupled in (False,True):
            with self.subTest(coupled=coupled):
                r=Rig(guided=True,dual=True)
                if coupled:r.click(1,1);r.complete();r.step()
                mode=r.core.teleop_mode
                r.grips=[1.,1.];r.step();r.complete();r.stopped=False
                r.grips=[0.,0.]
                intents=r.click(1,1)
                self.assertEqual([i.operation for i in intents],['stop'])
                self.assertEqual(r.core.reason,'MODE_SWITCH_REQUIRES_STOP')
                guide=r.core.snapshot(r.now)['operator_guide']
                self.assertEqual(guide['title'],'模式尚未切换')
                self.assertIn('重新按 B',guide['detail'])
                self.assertEqual(r.core.teleop_mode,mode)
                self.assertEqual(r.click(1,1),[])
                r.complete();r.stopped=True;r.step()
                self.assertIsNone(r.core.pending)
                r.click(1,1)
                self.assertEqual(r.core.pending.operation,'select_mode')
                self.assertNotEqual(r.core.pending.value,mode)



class StateTests(unittest.TestCase):
    def test_b_explains_unmet_conditions_and_success_means_standby_only(self):
        from snowbot_teleop.control import loading_guide
        r=Rig();r.core.lock('BIMANUAL_WORKER_TIMEOUT',r.now);r.core.drain();r.complete()
        r.grips=[1.,0.];r.step();r.click(1,1)
        self.assertEqual(r.core.reason,'RECOVERY_RELEASE_CONTROLS')
        self.assertIn('握把和扳机',loading_guide(r.core.snapshot(),r.modes)['detail'])
        r.grips=[0.,0.];r.step();r.click(1,1)
        self.assertEqual(r.core.pending.operation,'recover')
        r.complete(False,'DRIVE_FAULT')
        self.assertIn('DRIVE_FAULT',loading_guide(r.core.snapshot(),r.modes)['detail'])
        self.assertEqual(r.core.first_fault['code'],'BIMANUAL_WORKER_TIMEOUT')
        r.click(1,1);r.complete()
        self.assertEqual(loading_guide(r.core.snapshot(),r.modes)['title'],'已恢复待命')
        self.assertIsNone(r.core.pending);self.assertEqual(r.step(),[])

    def test_b_rechecks_latched_stop_and_restores_standby_only_after_confirmation(self):
        r=Rig();r.core.lock('PICO_STOP_UNCONFIRMED',r.now);r.core.drain();r.complete(False,'PICO_STOP_UNCONFIRMED')
        intent=r.click(1,1)[-1]
        self.assertEqual(intent.operation,'recover')
        r.core.complete(intent.token,False,'GRIPPER_RECOVERY_REQUIRES_REST',r.now)
        self.assertEqual(r.core.phase,'locked')
        intent=r.click(1,1)[-1]
        r.complete(True)
        self.assertEqual((r.core.phase,r.core.reason),('standby','RECOVERED_TO_STANDBY'))
        self.assertIsNone(r.core.stop_failure)

    def test_device_fault_b_rechecks_then_returns_only_to_standby(self):
        r=Rig('unknown',guided=True,dual=True)
        mode=r.core.teleop_mode
        r.core.lock('SNOWBOT_PDO_STALE',r.now);r.core.drain();r.complete()
        self.assertEqual(r.core.recovery_required,'device_inspection')
        first=r.core.first_fault.copy()
        r.click(1,1)
        self.assertEqual(r.core.pending.operation,'recover')
        self.assertEqual(r.core.phase,'locked')
        r.complete(False,'PICO_RECOVERY_STOP_NOT_CONFIRMED')
        self.assertEqual(r.core.phase,'locked')
        self.assertEqual(r.core.recovery_required,'device_inspection')
        self.assertEqual(r.core.first_fault,first)
        r.click(1,1)
        self.assertEqual(r.core.pending.operation,'recover')
        r.complete(True)
        self.assertEqual((r.core.phase,r.core.reason),('standby','RECOVERED_TO_STANDBY'))
        self.assertFalse(r.core.fault_active);self.assertFalse(r.core.recovery_required)
        self.assertEqual(r.core.teleop_mode,mode)
        self.assertIsNone(r.core.pending);self.assertEqual(r.step(),[])

    def test_stop_failure_and_late_request_never_replace_first_fault(self):
        r=Rig();r.core.phase='following'
        r.core.lock('PICO_FEEDBACK_UNAVAILABLE',r.now)
        stop=r.core.pending
        r.core.record_fault('SNOWBOT_PDO_STALE',r.now-.01)
        r.core.complete(stop.token,False,'PICO_STOP_UNCONFIRMED:TOOL_STOP_UNCONFIRMED',r.now+.01)
        r.core.record_fault('PICO_STOP_UNCONFIRMED:REQUEST_NOT_CURRENT',r.now+.02,kind='request')
        r.core.lock('PICO_INPUT_TIMEOUT',r.now+.03)
        view=r.core.snapshot()
        self.assertEqual(view['first_fault']['code'],'SNOWBOT_PDO_STALE')
        self.assertEqual(view['reason'],'PICO_FEEDBACK_UNAVAILABLE')
        self.assertEqual(view['stop_failure']['code'],'PICO_STOP_UNCONFIRMED:TOOL_STOP_UNCONFIRMED')
        self.assertEqual(view['request_failure']['code'],'PICO_STOP_UNCONFIRMED:REQUEST_NOT_CURRENT')
        self.assertEqual(view['recovery_required'],'device_inspection')
        self.assertIsNone(r.core.pending)
        r.click(1,1)
        self.assertEqual(r.core.phase,'locked')

    def test_confirmed_stream_stop_recovers_only_on_fresh_released_b_edge(self):
        r=Rig();r.core.phase='following'
        r.core.lock('PICO_INPUT_TIMEOUT',r.now);r.complete()
        self.assertFalse(r.core.recovery_required)
        r.grips=[1.,0.];r.step();r.click(1,1)
        self.assertEqual(r.core.phase,'locked')
        r.grips=[0.,0.];r.step();r.click(1,1)
        self.assertEqual(r.core.pending.operation,'recover')
        self.assertEqual(r.core.phase,'locked')
        r.complete()
        self.assertEqual(r.core.phase,'standby');self.assertFalse(r.core.fault_active)
        self.assertEqual(r.core.first_fault['code'],'PICO_INPUT_TIMEOUT')
        self.assertIsNone(r.core.pending)
        r.core.record_fault('DEVICE_FEEDBACK_ERROR',r.now-.2)
        self.assertFalse(r.core.fault_active)  # Delayed prior-episode telemetry.
        r.core.lock('PICO_PAIR_INVALID',r.now+.01)
        self.assertEqual(r.core.first_fault['code'],'PICO_PAIR_INVALID')
        self.assertIsNone(r.core.stop_failure)

    def test_failed_stop_cannot_recover_from_rest_only(self):
        r=Rig();r.core.lock('PICO_INPUT_TIMEOUT',r.now)
        r.complete(False,'PICO_STOP_UNCONFIRMED:DEADLINE_EXPIRED')
        r.click(1,1)
        self.assertEqual(r.core.phase,'locked')

    def test_timeout_keeps_triggering_old_frame_evidence_after_fresh_input_returns(self):
        rig=Rig(guided=True,dual=True);rig.grips=[1.,1.];rig.step();rig.complete()
        core=rig.core;old_stamp=core.stamp;old_seq=core.seq
        rig.now=old_stamp+1.01
        core.update_feedback(PicoFeedback(rig.now,False),rig.now)
        core.tick(rig.now)
        evidence=core.last_input_timeout
        self.assertEqual((core.phase,core.reason),('locked','PICO_INPUT_TIMEOUT'))
        self.assertEqual(evidence['phase'],'following')
        self.assertEqual(evidence['input_seq'],old_seq)
        self.assertAlmostEqual(evidence['input_age_s'],1.01)
        core.tick(rig.now+.01)
        rig.step(.01)
        self.assertIs(core.last_input_timeout,evidence)
        self.assertTrue(core.snapshot(rig.now)['input_fresh'])
        self.assertEqual(core.snapshot(rig.now)['last_input_timeout'],evidence)


    def test_pair_skew_compares_each_hand_with_its_own_previous_time(self):
        rig=Rig();core=rig.core;t=rig.now+.02
        pair=tuple(replace(f,seq=rig.seq+1,stamp=t+i*.0007) for i,f in enumerate(core.frames))
        core.ingest(*pair,t+.005)
        newer=tuple(replace(f,seq=f.seq+1,stamp=t+dt) for f,dt in zip(pair,(.00056,.001)))
        core.ingest(*newer,t+.006)
        self.assertEqual(core.seq,newer[0].seq)
        self.assertEqual(core.phase,'standby')

    def test_real_time_reversal_of_either_hand_still_locks_and_stops(self):
        for side in (0,1):
            with self.subTest(side=side):
                rig=Rig();core=rig.core;previous=core.frames
                pair=tuple(replace(f,seq=f.seq+1,stamp=f.stamp+(-.0001 if i==side else .001))
                           for i,f in enumerate(previous))
                core.ingest(*pair,rig.now+.005)
                self.assertEqual((core.phase,core.reason),('locked','PICO_TIME_REVERSED'))
                self.assertEqual(core.pending.operation,'stop')
                self.assertEqual(core.frames,previous)

    def test_stop_fault_keeps_its_cause_instead_of_claiming_feedback_loss(self):
        rig=Rig();core=rig.core
        code='PICO_STOP_UNCONFIRMED:STOP_UNCONFIRMED:vendor_hold'
        core.update_feedback(PicoFeedback(rig.now,False,healthy=False,fault=code),rig.now)
        self.assertEqual((core.phase,core.reason),('locked',code))
        self.assertFalse(core.rest(rig.now))

    def test_startup_input_before_feedback_does_not_latch_timeout_or_use_stale_pair(self):
        core=PicoControl(initial_load='unknown')
        rig=Rig()
        core.ingest(*rig.core.frames,rig.now)
        later=rig.now+1.01
        core.tick(later)
        self.assertEqual(core.phase,'checking')
        core.update_feedback(PicoFeedback(later,True),later)
        self.assertEqual(core.phase,'checking')
        rig.step(1.01)
        core.ingest(*rig.core.frames,rig.now)
        core.update_feedback(PicoFeedback(rig.now,True),rig.now)
        self.assertEqual(core.phase,'standby')
        core.tick(rig.now+1.01)
        self.assertEqual((core.phase,core.reason),('locked','PICO_INPUT_TIMEOUT'))

    def test_initial_feedback_discovery_waits_without_latching_or_moving(self):
        rig=Rig();core=PicoControl(initial_load='empty')
        core.update_feedback(PicoFeedback(0.,False,healthy=False),rig.now)
        core.ingest(*rig.core.frames,rig.now)
        core.update_feedback(PicoFeedback(0.,False,healthy=False),rig.now)
        core.tick(rig.now)
        self.assertEqual(core.phase,'checking');self.assertEqual(core.drain(),[])
        core.update_feedback(PicoFeedback(rig.now,True,('closed','closed')),rig.now)
        self.assertEqual(core.phase,'standby')

    def test_boot_has_no_effects_and_squeeze_is_not_gripper_open(self):
        rig=Rig();self.assertEqual(rig.core.phase,'standby')
        rig.grips=[1.,1.];self.assertEqual(rig.step(),[])
        self.assertEqual(rig.core.desired,['closed','closed'])

    def test_trigger_controls_only_its_gripper_and_release_closes(self):
        rig=Rig();rig.loading()
        rig.triggers[0]=1.;effects=rig.step()
        self.assertEqual(effects[-1].value,('open','closed'))
        rig.modes=('open','closed');rig.complete()
        rig.triggers[0]=0.;effects=rig.step()
        self.assertEqual(effects[-1].value,('closed','closed'))
        rig.modes=('closed','closed');rig.complete()
        rig.triggers[1]=1.;self.assertEqual(rig.step()[-1].value,('closed','open'))

    def test_trigger_hysteresis_does_not_close_at_half_press(self):
        rig=Rig();rig.loading();rig.triggers[0]=1.;rig.step();rig.modes=('open','closed');rig.complete()
        rig.triggers[0]=.5;self.assertEqual(rig.step(),[])
        self.assertEqual(rig.core.desired,['open','closed'])
        rig.triggers[0]=0.;self.assertEqual(rig.step()[-1].value,('closed','closed'))

    def test_prepare_held_during_loading_must_be_released_before_reuse(self):
        rig=Rig();rig.loading();rig.buttons[1]=1;rig.step()
        rig.core.phase='standby'
        for _ in range(70):self.assertEqual(rig.step(),[])

    def test_pose_origin_jump_locks_active_follow(self):
        rig=Rig('held');rig.grips=[1.,1.];rig.step();rig.core.phase='following';left,right=rig.core.frames
        pose=np.eye(4);pose[0,3]=1.
        rig.core.ingest(replace(left,pose=tuple(pose.ravel()),seq=left.seq+1,stamp=rig.now+.01),
                        replace(right,seq=right.seq+1,stamp=rig.now+.01),rig.now+.01)
        self.assertEqual(rig.core.phase,'locked')
        self.assertEqual(rig.core.reason,'PICO_POSE_ORIGIN_JUMP')
        self.assertEqual(rig.core.last_pose_jump['translation_limit_m'],.35)
        self.assertAlmostEqual(rig.core.last_pose_jump['translation_m'],1.)

    def test_moderate_displacement_and_wrist_rotation_survive_a_frame_gap(self):
        rig=Rig('held');rig.grips=[1.,1.];rig.step();rig.core.phase='following';left,right=rig.core.frames
        angle=.8;pose=np.eye(4);pose[:2,:2]=[[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]]
        pose[0,3]=.25
        rig.core.ingest(replace(left,pose=tuple(pose.ravel()),seq=left.seq+1,stamp=rig.now+.14),
                        replace(right,seq=right.seq+1,stamp=rig.now+.14),rig.now+.14)
        self.assertEqual(rig.core.phase,'following')
        self.assertIsNone(rig.core.last_pose_jump)

    def test_release_takes_priority_over_a_large_paused_hand_displacement(self):
        rig=Rig('held');rig.grips=[1.,1.];rig.step();rig.core.phase='following';left,right=rig.core.frames
        pose=np.eye(4);pose[0,3]=1.
        rig.core.ingest(replace(left,pose=tuple(pose.ravel()),grip=0.,seq=left.seq+1,stamp=rig.now+.01),
                        replace(right,seq=right.seq+1,stamp=rig.now+.01),rig.now+.01)
        self.assertNotEqual(rig.core.reason,'PICO_POSE_ORIGIN_JUMP')
        self.assertEqual(rig.core.pending.operation,'stop')

    def test_invalid_pose_jump_limits_are_rejected(self):
        for options in ({'pose_jump_translation_m':0.},{'pose_jump_rotation_rad':float('nan')},
                        {'pose_jump_rotation_rad':4.}):
            with self.assertRaisesRegex(ValueError,'POSE_JUMP_CONFIG'):PicoControl(**options)

    def test_invalid_pose_never_reaches_actuator_requests(self):
        rig=Rig();left,right=rig.core.frames
        rig.core.ingest(replace(left,pose=(float('nan'),)*16,seq=10),replace(right,seq=10),rig.now)
        self.assertEqual(rig.core.phase,'locked')
        self.assertEqual(rig.core.drain()[-1].operation,'stop')

    def test_two_triggers_and_cross_release_remain_independent(self):
        rig=Rig();rig.loading();rig.triggers=[1.,1.]
        self.assertEqual(rig.step()[-1].value,('open','open'))
        rig.modes=('open','open');rig.complete()
        rig.triggers[0]=0.
        self.assertEqual(rig.step()[-1].value,('closed','open'))

    def test_open_outside_loading_only_stops_and_is_not_deferred(self):
        rig=Rig();rig.triggers[1]=1.
        self.assertEqual(rig.step()[-1].operation,'stop')
        self.assertEqual(rig.core.desired,['closed','closed'])
        rig.complete();rig.context()
        rig.click(0,2);rig.complete();rig.click(1,1)
        self.assertEqual(rig.core.phase,'loading')
        self.assertEqual(rig.step(),[])

    def test_support_and_b_are_required_before_loading(self):
        rig=Rig('held');rig.click(0,2);rig.complete();rig.click(1,1)
        self.assertEqual(rig.core.phase,'load_check')
        rig.context(supported=True);rig.click(1,1)
        self.assertEqual(rig.core.phase,'loading')

    def test_prepare_requires_a_continuously_held_empty_and_released_squeeze(self):
        rig=Rig();rig.buttons[1]=1;rig.step()
        effects=[]
        for _ in range(60):effects+=rig.step()
        self.assertEqual([x.operation for x in effects],['prepare'])
        self.assertTrue(rig.core.last_prepare_check['accepted'])
        self.assertTrue(all(rig.core.last_prepare_check['checks'].values()))
        rig.buttons[1]=0
        self.assertEqual(rig.step()[-1].operation,'stop')

    def test_prepare_rejected_while_held_does_not_run_when_condition_improves(self):
        rig=Rig('held');rig.buttons[1]=1;rig.step()
        for _ in range(60):self.assertEqual(rig.step(),[])
        rig.core.load='empty'
        for _ in range(60):self.assertEqual(rig.step(),[])

    def test_prepare_failure_snapshot_retains_attempt_conditions(self):
        rig=Rig('unknown');rig.grips[0]=.8;rig.stopped=False;rig.buttons[1]=1
        for _ in range(60):self.assertEqual(rig.step(),[])
        check=rig.core.last_prepare_check
        self.assertFalse(check['accepted'])
        self.assertEqual({k for k,v in check['checks'].items() if not v},
                         {'load_empty','left_squeeze_released','motion_stopped'})
        self.assertEqual(check['load_source'],'unknown')
        rig.stopped=True;rig.grips[0]=0.;rig.step();rig.context('empty')
        self.assertEqual(rig.core.snapshot()['load_source'],'operator_declaration')
        self.assertTrue(all(rig.core.prepare_checks(rig.now).values()))
        self.assertIs(rig.core.last_prepare_check,check)
        self.assertEqual(check['load_state'],'unknown')
        from bindu_teleoperation.vr.hud import hud_spec
        hud=hud_spec(dict(mode='standby',reason=rig.core.reason,last_prepare_check=check))
        self.assertIn('空手状态未确认',hud['detail'])
        self.assertIn('左侧握未松开',hud['detail'])

    def test_startup_held_a_and_triggers_are_not_actions(self):
        rig=Rig();rig.core=PicoControl(initial_load='empty');rig.buttons[1]=1;rig.triggers=[1.,1.]
        for _ in range(80):self.assertEqual(rig.step(),[])

    def test_squeeze_release_stops_without_closing_virtual_grip(self):
        rig=Rig('held');rig.core.virtual=True;rig.core.phase='following';rig.grips=[1.,1.];rig.step()
        rig.grips[0]=0.;effects=rig.step()
        self.assertEqual(effects[-1].operation,'stop');self.assertTrue(rig.core.virtual)

    def test_trigger_during_preparation_or_x_switch_only_stops(self):
        for phase in ('preparing','following','switching','engaging'):
            rig=Rig('held');rig.core.phase=phase;rig.buttons[1]=1 if phase=='preparing' else 0
            rig.grips=[0.,0.] if phase=='preparing' else [1.,1.]
            rig.triggers[1]=1.;rig.buttons[0]=1
            effects=rig.step()
            self.assertEqual(effects[-1].operation,'stop')
            self.assertEqual(rig.core.desired,['closed','closed'])

    def test_y_has_priority_over_b_and_a(self):
        rig=Rig();rig.buttons=[2,3];effects=rig.step()
        self.assertEqual(rig.core.phase,'locked')
        self.assertEqual([x.operation for x in effects],['stop'])

    def test_timeout_and_late_completion_never_rearm(self):
        rig=Rig();rig.loading();rig.triggers[0]=1.;token=rig.step()[-1].token
        rig.now+=.3;rig.core.tick(rig.now)
        self.assertEqual(rig.core.phase,'locked')
        self.assertFalse(rig.core.complete(token,True,now=rig.now))
        self.assertEqual(rig.core.phase,'locked')

    def test_input_loss_during_open_does_not_synthesize_close(self):
        rig=Rig();rig.loading();rig.triggers[0]=1.;rig.step();rig.modes=('open','closed');rig.complete()
        rig.now+=.3;rig.core.tick(rig.now)
        self.assertEqual(rig.core.drain()[-1].operation,'stop')
        self.assertEqual(rig.core.desired,['open','closed'])

    def remote_parameters(self):
        from snowbot_hardware.parameters import load_parameters
        config = package_share('snowbot_bringup') / 'config/robots/snowbot/runtime_parameters_remote_vr.json'
        return load_parameters(config)

    def test_remote_pair_window_and_stream_timeout(self):
        import ast
        from types import SimpleNamespace as S
        from snowbot_hardware.parameters import apply_tool_timing
        params = self.remote_parameters()
        tool = apply_tool_timing({}, params)
        self.assertEqual((tool['input_max_age'], tool['command_max_age'], tool['feedback_max_age']),
                         (1., 1.5, .5))
        path=Path(find_spec('snowbot_teleop.node').origin)
        tree=ast.parse(path.read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='PicoNode')
        init=next(n for n in cls.body if getattr(n,'name','')=='__init__')
        control=next(n for n in init.body if isinstance(n,ast.Assign) and
                     isinstance(n.targets[0],ast.Attribute) and n.targets[0].attr=='control')
        node=S(tool_cfg=tool,runtime_parameters=params,cfg={},simulation_demo=False,position_contract=True)
        exec(compile(ast.Module(body=[control],type_ignores=[]),str(path),'exec'),
             dict(self=node,PicoControl=PicoControl,initial_load='empty'))
        self.assertEqual((node.control.max_age,node.control.feedback_max_age),(1.,.5))
        for settings, skew, expected in ((None, .04, 'standby'),
                                        (params['pico'], .04, 'standby'),
                                        (params['pico'], .051, 'locked')):
            with self.subTest(settings=settings, skew=skew):
                rig = Rig(settings=settings)
                t = rig.now + .1
                left, right = rig.core.frames
                rig.core.ingest(replace(left, stamp=t-skew, seq=3),
                                replace(right, stamp=t, seq=3), t)
                self.assertEqual(rig.core.phase, expected)
        rig = Rig(settings=params['pico'])
        core = rig.core
        core.phase = 'following'
        for gap in (.99, 1.01):
            now = rig.now + gap
            core.update_feedback(PicoFeedback(now, True), now)
            core.tick(now)
            if gap == .99:
                self.assertEqual(core.phase, 'following')
        self.assertEqual(core.reason, 'PICO_INPUT_TIMEOUT')
        self.assertEqual(core.last_input_timeout['input_max_age_s'], 1.)
        rig = Rig(settings=params['pico'])
        rig.core.tick(rig.now + .3)
        self.assertEqual(rig.core.reason, 'PICO_FEEDBACK_STALE')

    def test_remote_duplicate_and_expired_pairs_do_not_fault_or_refresh_input(self):
        rig = Rig(settings=self.remote_parameters()['pico'])
        core = rig.core
        core.phase = 'following'
        frames = core.frames
        for queued in ([replace(f, valid=False) for f in frames],
                       [replace(f, seq=f.seq+1, stamp=f.stamp-2.) for f in frames]):
            core.ingest(*queued, rig.now)
            self.assertIs(core.frames, frames)
            self.assertEqual(core.phase, 'following')
            self.assertEqual(core.drain(), [])
        # A queue of old events cannot hide a genuine loss of current input.
        now = rig.now + 1.01
        core.update_feedback(PicoFeedback(now, True), now)
        core.tick(now)
        self.assertEqual(core.reason, 'PICO_INPUT_TIMEOUT')
        self.assertEqual([i.operation for i in core.drain()], ['stop'])

    def test_remote_reconnect_in_standby_and_in_motion(self):
        settings = self.remote_parameters()['pico']
        rig = Rig(settings=settings)
        old = rig.core.frames
        rig.source = 'reconnected'
        acquiring = [replace(f, source_id=rig.source, valid=False) for f in old]
        rig.core.ingest(*acquiring, rig.now)
        self.assertEqual((rig.core.phase, rig.core.source), ('standby', 'headset-1'))
        rig.buttons[1] = 1  # A is already held on reconnect.
        self.assertEqual(rig.step(), [])
        self.assertEqual((rig.core.phase, rig.core.source), ('standby', 'reconnected'))
        rig.core.ingest(*(replace(f, valid=False) for f in old), rig.now)
        self.assertEqual(rig.core.phase, 'standby')
        self.assertEqual(rig.step(), [])
        rig.core.phase = 'following'
        rig.source = 'another-connection'
        self.assertEqual([i.operation for i in rig.step()], ['stop'])
        self.assertEqual(rig.core.reason, 'PICO_CONNECTION_CHANGED')

    def test_reconnect_locks_and_retired_source_cannot_take_over(self):
        rig=Rig();rig.core.phase='following'
        old=rig.core.frames;rig.source='new';rig.counters=[ButtonCounters(),ButtonCounters()]
        rig.step();self.assertEqual(rig.core.phase,'locked')
        rig.core.ingest(*old,rig.now)
        self.assertEqual(rig.core.source,'new')

    def test_coalesced_squeeze_release_stops_follow_or_preparation(self):
        for phase in ('following','preparing'):
            rig=Rig('held');rig.core.phase=phase
            left,right=rig.core.frames
            rig.core.ingest(replace(left,seq=left.seq+1,stamp=rig.now+.01,clutch_seq=left.clutch_seq+1),
                            replace(right,seq=right.seq+1,stamp=rig.now+.01),rig.now+.01)
            self.assertEqual(rig.core.drain()[-1].operation,'stop')
            self.assertNotEqual(rig.core.phase,phase)

    def test_source_change_invalidates_support_and_load_declarations(self):
        rig=Rig('held');rig.context('held',True)
        rig.source='new';rig.counters=[ButtonCounters(),ButtonCounters()];rig.step()
        self.assertFalse(rig.core.supported);self.assertEqual(rig.core.load,'unknown')

    def test_loading_completion_needs_fresh_load_declaration(self):
        rig=Rig();rig.loading();rig.click(1,1)
        self.assertEqual(rig.core.phase,'loading')
        self.assertEqual(rig.core.reason,'FRESH_LOAD_DECLARATION_REQUIRED')

    def test_context_is_fenced_and_never_recovers_a_fault(self):
        rig=Rig();rig.core.lock('fault',rig.now);rig.complete()
        with self.assertRaises(ValueError):rig.core.context(rig.source,0,'held',True,rig.now)
        rig.context('held');self.assertEqual(rig.core.phase,'locked')



if __name__ == "__main__":
    unittest.main()
