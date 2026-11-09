"""Head stick and native-port adapter checks without ROS or devices."""
from dataclasses import replace
import math
import sys
import unittest
from types import SimpleNamespace as S, ModuleType
from unittest.mock import Mock, patch
from bindu_contracts.profile import Profile
from bindu_contracts.resources import package_share
from bindu_contracts.teleoperation import VRFrame
from snowbot_teleop.head import HeadTeleop, TeleopHead, NAMES

CONFIG = package_share('snowbot_bringup')/'config/robots/snowbot'


class HeadTests(unittest.TestCase):
    def setUp(self):
        self.profile = Profile.load(CONFIG/'snowbot_real_position.json')
        self.core = HeadTeleop(self.profile)
        self.now, self.seq, self.source = 10., 0, 'vr'
        self.q = dict(zip(NAMES, (.2, -.1)))
        self.v = dict.fromkeys(NAMES, 0.)
        self.step()
        self.step()
        self.assertTrue(self.core.armed)

    def pair(self, stick=(0., 0.), left=(0., 0.)):
        self.seq += 1
        return tuple(VRFrame(self.source, self.seq, self.now, side, (),
            mapping='pico_snowbot_v1', stick=axes)
            for side, axes in (('left', left), ('right', stick)))

    def step(self, stick=(0., 0.), *, left=(0., 0.), allowed=True, dt=.02, track=True):
        if track and self.core.target is not None:
            self.q = dict(zip(NAMES, self.core.target))
        self.now += dt
        self.core.ingest(*self.pair(stick, left), self.now, self.now)
        return self.core.tick(self.now, self.now, self.q, self.v, self.now, allowed=allowed)

    def test_measured_capture_two_axes_and_centre_keeps_pose(self):
        self.assertEqual(self.core.target, (.2, -.1))
        target = self.step((-1., -1.), left=(1., 1.))
        self.assertAlmostEqual(target[0], .21)
        self.assertAlmostEqual(target[1], -.09)
        self.assertEqual(self.step(left=(-1., -1.)), target)
        reverse = self.step((1., 1.))
        self.assertAlmostEqual(reverse[0], .2)
        self.assertAlmostEqual(reverse[1], -.1)

    def test_deadzone_limits_and_no_catchup_after_gap(self):
        self.assertEqual(self.step((.049, -.049)), (.2, -.1))
        for _ in range(200):
            self.step((-1., -1.))
        self.assertEqual(self.core.target, (.791, .791))
        self.assertIsNone(self.step((-1., -1.), dt=.11))
        self.assertEqual(self.core.reason, 'HEAD_TICK_GAP')
        self.assertIsNone(self.step((-1., -1.)))
        self.step()
        self.assertTrue(self.core.armed)

    def test_pause_requires_new_neutral_and_stationary_feedback(self):
        self.step((-1., 0.))
        self.assertIsNone(self.step(allowed=False))
        stopped_at = self.core.stop_at
        self.assertFalse(self.core.stopped(self.q, self.v, stopped_at, self.now))
        self.assertIsNone(self.step((-1., 0.)))
        self.v[NAMES[1]] = .02
        self.assertIsNone(self.step())
        self.v[NAMES[1]] = 0.
        self.assertIsNotNone(self.step())

    def test_existing_measured_tolerance_keeps_targets_inside_limits(self):
        self.step(allowed=False)
        self.q[NAMES[0]]=.7925
        target=self.step(track=False)
        self.assertEqual(target[0],.791)
        self.assertEqual(self.q[NAMES[0]],.7925)
        self.q[NAMES[0]]=.794
        self.assertIsNone(self.step(track=False))

    def test_coordinator_waits_for_head_stop_feedback(self):
        from tests.unit.test_pico_control import ScenarioRuntimeTests
        from snowbot_teleop.control import PicoFeedback
        n=ScenarioRuntimeTests.production('feedback')
        n.feedback.__func__.__globals__['PicoFeedback']=PicoFeedback
        stamp=S(sec=10,nanosec=0)
        s=S(lease_active=False,state='IDLE',reference=S(name=[]),
            joints=S(name=[],velocity=[]),feedback_stamp=stamp)
        n.reading=lambda:(s,{})
        n.position_ops=S(modes=lambda **kw:('closed','closed'),stopped=lambda:True)
        n.control=S(phase='standby');n.profile=self.profile;n.names=[]
        n.handoff_complete=False;n.stop_request=None;n.base=None;n.now=lambda:10.
        n.head=S(writable=True,stopped=Mock(return_value=False))
        self.assertFalse(n.feedback().stopped)
        n.head.stopped.return_value=True
        self.assertTrue(n.feedback().stopped)

    def test_invalid_axes_replay_and_source_change_require_neutral(self):
        for axes in ((), (math.nan, 0.), (True, 0.), (1.1, 0.)):
            left, right = self.pair()
            self.core.ingest(left, replace(right, stick=axes), self.now, self.now)
            self.assertFalse(self.core.armed)
        self.step()
        self.source = 'new'
        self.assertIsNone(self.step((-1., 0.)))
        self.step()
        pair = self.pair()
        self.now += .02
        self.core.ingest(*pair, self.now, self.now)
        self.assertFalse(self.core.armed)
        self.assertEqual(self.core.reason, 'HEAD_INPUT_STALE')

    def test_input_feedback_and_following_error_stop(self):
        for kind in ('source', 'receipt', 'feedback', 'following'):
            with self.subTest(kind=kind):
                self.step()
                self.now += .02
                self.core.ingest(*self.pair((-1., 0.)), self.now, self.now)
                stamp = self.now
                if kind == 'source':self.core.stamp -= .26
                if kind == 'receipt':self.core.received -= .26
                if kind == 'feedback':stamp -= .21
                if kind == 'following':self.q[NAMES[0]] -= .16
                self.assertIsNone(self.core.tick(self.now, self.now, self.q, self.v, stamp, allowed=True))
                self.assertFalse(self.core.armed)
                if kind == 'following':self.q[NAMES[0]] += .16


class AdapterTests(unittest.TestCase):
    def test_coordinator_filters_pairs_before_head_input(self):
        from tests.support.pico import Rig
        from tests.unit.test_pico_control import ScenarioRuntimeTests
        from snowbot_teleop.control import PicoFeedback
        rig=Rig();n=ScenarioRuntimeTests.production('on_frame')
        n.on_frame.__func__.__globals__.update(VRFrame=VRFrame,VRInput=lambda **kw:S(**kw),PICO_MAPPING='pico_snowbot_v1')
        self.now=rig.now+.02
        n.now=lambda:self.now;n.control=rig.core;n.frames={}
        n.matches_mode=lambda msg:True;n.feedback=lambda:PicoFeedback(self.now,True)
        n.last_pair_at=n.last_pair_monotonic=n.last_prepare_check=None
        n.runtime_parameters={'diagnostics':{'input_gap_warning_s':1.}}
        n.base=None;n.head=S(core=Mock());n.event=Mock();n.projected=Mock()
        n.release_until=0.;n.instance_id='coordinator';n.independent_grippers=False
        n.position_contract=True;n.handoff_complete=True
        pair=tuple(replace(f,seq=f.seq+1,stamp=self.now) for f in rig.core.frames)
        def deliver(frames):
            for f in frames:
                sec,ns=divmod(round(f.stamp*1e9),10**9)
                n.on_frame(S(**dict(vars(f),stamp=S(sec=sec,nanosec=ns),runtime_mode='real')))
        deliver(pair)
        self.assertEqual(n.head.core.ingest.call_count,1)
        deliver(pair)  # The coordinator owns duplicate/retired-source rejection.
        self.assertEqual(n.head.core.ingest.call_count,1)
        deliver(tuple(replace(f,seq=f.seq+1,mapping='invalid') for f in pair))
        self.assertEqual(n.head.core.ingest.call_count,1)
        self.assertEqual(n.control.phase,'locked')
        deliver(tuple(replace(f,source_id='new',seq=1) for f in pair))
        self.assertEqual(n.head.core.ingest.call_count,2)
        deliver(tuple(replace(f,seq=f.seq+2) for f in pair))
        self.assertEqual(n.head.core.ingest.call_count,2)

    def node(self, *, simulation=False, enabled=True):
        profile = Profile.load(CONFIG/'snowbot_real_position.json')
        joints = S(name=list(NAMES), position=[.2, -.1], velocity=[0., 0.])
        return S(profile=profile, simulation=simulation, now=lambda: self.now,
            get_parameter=lambda name: S(value=enabled),
            tool_cfg={'max_reference_error':.15,'feedback_max_age':.2},
            state=S(joints=joints, feedback_stamp=S(sec=10, nanosec=0), instance_id='execution'),
            create_publisher=Mock(return_value=Mock()),
            get_name=lambda:'pico_control', get_namespace=lambda:'/bindu_snowbot',
            get_publishers_info_by_topic=lambda topic:[] if topic=='/head/motor_cmd' else [S(node_name='pico_control',node_namespace='/bindu_snowbot')],
            get_subscriptions_info_by_topic=lambda topic:[S(node_name='servo_node',node_namespace='/',
                topic_type='std_msgs/msg/Float64MultiArray')])

    def setUp(self):
        self.now = 10.
        self.clock = patch('snowbot_teleop.head.time.monotonic', lambda:self.now)
        self.clock.start();self.addCleanup(self.clock.stop)
        msg = ModuleType('std_msgs.msg');msg.Float64MultiArray=lambda **kw:S(**kw)
        self.modules=patch.dict(sys.modules, {'std_msgs.msg':msg})
        self.modules.start();self.addCleanup(self.modules.stop)

    def test_disabled_and_simulation_never_create_writer(self):
        for simulation, enabled in ((False,False),(True,True)):
            n=self.node(simulation=simulation,enabled=enabled)
            bridge=TeleopHead(n)
            n.create_publisher.assert_not_called()
            self.assertFalse(bridge.writable)

    def test_native_packet_deadline_stop_pacing_and_executor_change(self):
        n=self.node();bridge=TeleopHead(n)
        control=S(source='vr',seq=0,phase='standby',pending=None)
        def step(stick=(0.,0.), dt=.021):
            self.now+=dt;control.seq+=1
            ns=round(self.now*1e9);sec,nanosec=divmod(ns,10**9)
            n.state.feedback_stamp=S(sec=sec,nanosec=nanosec)
            pair=[VRFrame('vr',control.seq,self.now,side,(),mapping='pico_snowbot_v1',stick=axes)
                for side,axes in (('left',(0.,0.)),('right',stick))]
            bridge.core.ingest(*pair,self.now,self.now)
            bridge.tick(control,True)
        step();step();step((-1.,0.))
        publish=n.create_publisher.return_value.publish
        data=publish.call_args.args[0].data
        self.assertEqual(data[2:4],[bridge.core.stamp,.25])
        self.assertEqual(data[-1],0.)
        control.pending=object();self.now+=.001
        bridge.tick(control,True)
        self.assertEqual(publish.call_args.args[0].data[-1],1.)
        self.assertFalse(bridge.core.armed)
        control.pending=None
        step();step((-1.,0.))
        # Invalid input also bypasses target pacing and requests a native stop.
        step((),dt=.001)
        self.assertEqual(publish.call_args.args[0].data[-1],1.)
        n.state.instance_id='replacement'
        step()
        self.assertEqual(bridge.fault,'HEAD_EXECUTOR_CHANGED_RESTART_REQUIRED')
        self.assertFalse(bridge.core.armed)

    def test_conflicting_writer_revokes_head_only(self):
        n=self.node();bridge=TeleopHead(n)
        self.assertTrue(bridge.endpoints())
        n.get_publishers_info_by_topic=lambda topic:[S(),S()]
        self.now+=.11
        self.assertFalse(bridge.endpoints())


if __name__ == '__main__':
    unittest.main()
