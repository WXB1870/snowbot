"""Diagnostic capture must preserve payloads, drain on exit and expose loss."""
from importlib.util import find_spec
from tests.support.paths import BINDU_ROOT as ROOT
import importlib.util
from io import StringIO
import json
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace as S
import unittest
from unittest.mock import patch

spec=importlib.util.spec_from_file_location('snowbot_diagnostics',Path(find_spec('snowbot_bringup.visualization.snowbot_diagnostics').origin))
diagnostics=importlib.util.module_from_spec(spec)
spec.loader.exec_module(diagnostics)


class TimingStatisticsTests(unittest.TestCase):
    def test_five_second_distribution_uses_configured_limits_and_keeps_short_spikes(self):
        parameters=diagnostics.load_parameters()
        parameters['teleop']['feedback_max_age_s']=.4
        stats=diagnostics.TimingStatistics(parameters, now=0.)
        for i in range(1,101):
            stats.observe('teleop-state',dict(feedback_age_s=i*.01),wall=10.)
        self.assertIsNone(stats.flush(4.999))
        metric=stats.flush(5.)['metrics']['feedback_age_s']
        self.assertEqual(metric['count'],100)
        self.assertEqual(metric['p50_s'],.5);self.assertAlmostEqual(metric['p95_s'],.95)
        self.assertEqual(metric['max_s'],1.)
        self.assertEqual(metric['over_limit_count'],60)
        self.assertEqual(metric['limit_range_s'],[.4,.4])
        self.assertIsNone(stats.flush(10.))

    def test_rpc_network_and_command_clocks_are_separate_and_partial_window_flushes(self):
        stats=diagnostics.TimingStatistics(diagnostics.load_parameters(), now=0.)
        rpc=S(state='PICO_RPC_TIMING',code=json.dumps(dict(elapsed_s=5.1,timeout_s=5.,outcome='timeout')))
        self.assertTrue(stats.observe('runtime-events',rpc,wall=100.))
        stats.observe('vr-health',dict(socket=dict(rtt_s=.5)),wall=100.)
        stats.observe('vr-health',dict(browser_http_probe=dict(elapsed_ms=510.,outcome='completed')),wall=100.)
        stats.observe('vr-health',dict(browser_http_probe=dict(elapsed_ms=4000.,outcome='timeout')),wall=100.)
        stats.observe('accepted-commands',S(stamp=S(sec=99,nanosec=0),valid_for=.8),wall=100.)
        for state, data in (
                ('BIMANUAL_STARTED',dict(max_sample_gap=1.)),
                ('BIMANUAL_QP_RESULT',dict(solve_us=200000.,result_age_s=.6,sample_gap_s=1.1)),
                ('BIMANUAL_SUBMIT_TIMING',dict(submit_round_trip_s=.03, rpc_call_monotonic=2.01,
                    reply_observed_monotonic=2.03, sampled_monotonic=2.,input_pair=dict(
                    left_callback_monotonic=1.99,right_callback_monotonic=1.98),solve_span=dict(runqueue_wait_s=.004)))):
            self.assertFalse(stats.observe('runtime-events',S(state=state,code=json.dumps(data)),wall=100.))
        report=stats.flush(2.,final=True);metrics=report['metrics']
        self.assertEqual(metrics['pico_service_action_wait_s']['outcomes'],{'timeout':1})
        self.assertEqual(metrics['pico_service_action_wait_s']['over_limit_count'],1)
        self.assertEqual(metrics['observer_command_age_s']['over_limit_count'],1)
        self.assertEqual(metrics['tcp_peer_srtt_s']['p50_s'],.5)
        self.assertEqual(metrics['browser_http_round_trip_s']['count'],1)
        self.assertEqual(metrics['browser_http_round_trip_s']['max_s'],.51)
        self.assertEqual(metrics['browser_http_probe_wait_s']['outcomes'],dict(completed=1,timeout=1))
        for name in ('solve_s','worker_result_age_s','sample_gap_s'):
            self.assertEqual(metrics[name]['over_limit_count'],1)
        self.assertEqual(metrics['submit_round_trip_s']['max_s'],.03)
        for name,expected in [('submit_call_to_observed_s',.02),('pair_callback_to_sample_s',.01),('worker_runqueue_wait_s',.004)]:
            self.assertAlmostEqual(metrics[name]['max_s'],expected)
        self.assertIsNone(metrics['tcp_peer_srtt_s']['over_limit_count'])
        self.assertIn('not end-to-end',report['network_scope'])
        self.assertTrue(report['final']);self.assertIsNone(stats.flush(2.,final=True))

    def test_writer_counts_before_sampling_without_writing_each_rpc(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);writer=diagnostics.DiagnosticWriter(root/'diagnostics.jsonl')
            for i in range(100):writer.record('teleop-state',dict(feedback_age_s=i*.001))
            writer.record('runtime-events',S(state='PICO_RPC_TIMING',code=json.dumps(
                dict(elapsed_s=.02,timeout_s=5.,outcome='completed'))))
            writer.close()
            rows=[json.loads(line) for line in (root/'timing-summary.jsonl').read_text().splitlines()]
            self.assertEqual(len(rows),1)
            self.assertEqual(rows[0]['data']['metrics']['feedback_age_s']['count'],100)
            self.assertGreater(writer.status()['sampled_out'],0)
            self.assertFalse((root/'runtime-events.jsonl').exists())
            self.assertEqual(writer.status()['error'],'')


class DiagnosticTests(unittest.TestCase):
    def test_rotation_retains_order_and_shutdown_drains_all_streams(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);writer=diagnostics.DiagnosticWriter(root/'diagnostics.jsonl',chunk_bytes=180,sampling=False)
            for i in range(50):
                writer.record('vr-raw',dict(seq=i,pose=[1.,float('nan')]))
                writer.record('execution-state',dict(seq=i))
            writer.close()
            for stream in ('vr-raw','execution-state'):
                files=[root/(stream+'.jsonl'),*sorted(root.glob(stream+'-*.jsonl'))]
                rows=[json.loads(line) for path in files for line in path.read_text().splitlines()]
                self.assertEqual([r['data']['seq'] for r in rows],list(range(50)))
                self.assertTrue(all(r['received_monotonic_ns']>0 for r in rows))
            self.assertEqual(json.loads((root/'vr-raw.jsonl').read_text().splitlines()[0])['data']['pose'][1],'nan')
            self.assertEqual(writer.status()['written'],100)
            self.assertFalse(json.loads((root/'diagnostics-health.json').read_text())['active'])

    def test_full_queue_does_not_block_producer_and_reports_loss(self):
        gate=threading.Event();original=diagnostics.DiagnosticWriter.run
        def delayed(writer):
            gate.wait(2.);original(writer)
        with tempfile.TemporaryDirectory() as directory, patch.object(diagnostics.DiagnosticWriter,'run',delayed):
            writer=diagnostics.DiagnosticWriter(Path(directory)/'diagnostics.jsonl',capacity=1)
            try:
                writer.record('test',1);writer.record('test',2)
                self.assertEqual(writer.status()['dropped'],1)
            finally:gate.set();writer.close()
            self.assertEqual(writer.status()['written'],1)

    def test_write_failure_is_visible_and_never_reported_as_recording(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);(root/'bad.jsonl').mkdir()
            writer=diagnostics.DiagnosticWriter(root/'diagnostics.jsonl')
            writer.record('bad',{})
            writer.close()
            self.assertIn('IsADirectoryError',writer.status()['error'])
            self.assertFalse(writer.status()['active'])

    def test_ros_fixed_arrays_and_nan_are_serializable(self):
        from bindu_interfaces.msg import VRInput
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'diagnostics.jsonl';writer=diagnostics.DiagnosticWriter(path)
            writer.record('diagnostics',VRInput(seq=12,pose=[float('nan')]+[0.]*15),ros=True)
            writer.close()
            self.assertEqual(writer.status()['error'],'')
            data=json.loads(path.read_text())['data']
            self.assertEqual(data['seq'],12);self.assertEqual(data['pose'][0],'nan')

    def test_real_pdo_fixed_arrays_keep_integer_types_and_write_successfully(self):
        from snowbot_ctl_msgs.msg import PositionObservation
        from snowbot_bringup.visualization.snowbot_pdo import observation_payload, PdoMonitor
        msg=PositionObservation(boot_id=2**64-1,pdo_sample=1,pdo_age=0.,
            wkc=67,expected_wkc=67,valid_mask=(1<<20)-1,status=[0x40]*20)
        msg.header.stamp.sec=10
        payload=observation_payload(msg)
        self.assertTrue(all(type(value) is int for value in payload['status']))
        health=PdoMonitor().receive(payload,10.,1.)
        self.assertNotIn('PDO_STATUSWORD_INVALID',health['reasons'])
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'pdo-observation.jsonl'
            writer=diagnostics.DiagnosticWriter(path)
            writer.record('diagnostics',dict(observation=payload));writer.close()
            self.assertEqual(writer.status()['error'],'')
            self.assertEqual(writer.status()['written'],1)
            saved=json.loads(path.read_text())['data']['observation']
            self.assertEqual(saved['boot_id'],2**64-1)
            self.assertEqual(saved['status'],[0x40]*20)


class WrenchObserverTests(unittest.TestCase):
    @staticmethod
    def message(values=(1.,2.,3.,4.,5.,6.), sec=10, frame='left_sensor'):
        return S(header=S(stamp=S(sec=sec,nanosec=7),frame_id=frame),
                 wrench=S(force=S(**dict(zip('xyz',values[:3]))),
                          torque=S(**dict(zip('xyz',values[3:])))))

    def observer(self, writer=None):
        records=[];subscriptions=[]
        node=S(create_subscription=lambda *args:subscriptions.append(args) or object())
        modules={'geometry_msgs':S(),'geometry_msgs.msg':S(WrenchStamped=object),
                 'rclpy.qos':S(QoSProfile=lambda **kw:kw,
                     ReliabilityPolicy=S(BEST_EFFORT='best_effort'),
                     DurabilityPolicy=S(VOLATILE='volatile'))}
        with patch.dict('sys.modules',modules):
            observer=diagnostics.WrenchObserver(node,writer or S(record=lambda *args:records.append(args)),
                                                'run',period_s=.2)
        return observer,records,subscriptions

    def test_subscribes_to_both_existing_topics_without_control_ports(self):
        observer,records,subscriptions=self.observer()
        self.assertEqual([s[1] for s in subscriptions],list(observer.TOPICS.values()))
        for subscription in subscriptions:
            self.assertEqual(subscription[3],dict(depth=10,reliability='best_effort',durability='volatile'))
        # One message argument must suffice on Humble; side cannot be overwritten by message metadata.
        subscriptions[1][2](self.message(frame='right_sensor'))
        self.assertEqual(records[-1][0],'wrench-right')
        self.assertEqual(observer.received,dict(left=0,right=1))
        self.assertFalse(records[0][1]['pdo_correlated'])
        self.assertEqual(records[0][1]['default_max_logged_hz_per_side'],5.)

    def test_detaches_all_six_components_and_keeps_unknown_sensor_validity(self):
        for values in ((0.,)*6,(1.,2.,3.,4.,5.,6.),(float('nan'),2.,3.,4.,5.,6.)):
            observer,records,_=self.observer();msg=self.message(values)
            with patch.object(diagnostics.time,'time',return_value=10.1),patch.object(diagnostics.time,'monotonic',return_value=1.):
                observer.receive('left',msg)
            row=records[-1][1];msg.wrench.force.x=99.
            self.assertEqual(row['publication_stamp_ns'],10_000_000_007)
            self.assertEqual(row['frame_id'],'left_sensor')
            self.assertEqual(row['force'][1:],list(values[1:3]))
            self.assertEqual(row['torque'],list(values[3:]))
            self.assertNotEqual(row['force'][0],99.)
            self.assertEqual(row['finite'],all(x==x for x in values))
            for key in ('sensor_valid','sensor_data_no','sensor_age_s'):self.assertIsNone(row[key])
            for key in ('calibration_verified','units_verified','publisher_identity_verified','pdo_correlated'):
                self.assertFalse(row[key])

    def test_missing_sides_and_stale_publications_do_not_become_valid_force(self):
        observer,_,_=self.observer()
        empty=observer.sample(10.,1.)
        self.assertEqual(empty['sides']['left']['received'],0)
        self.assertIsNone(empty['sides']['left']['publication_stamp_ns'])
        self.assertFalse(empty['sides']['right']['recently_received'])
        with patch.object(diagnostics.time,'time',return_value=10.1),patch.object(diagnostics.time,'monotonic',return_value=1.):
            observer.receive('left',self.message())
        self.assertTrue(observer.sample(10.1,1.1)['sides']['left']['publication_fresh'])
        stale=observer.sample(10.4,1.4)
        self.assertFalse(stale['sides']['left']['publication_fresh'])
        self.assertFalse(stale['sides']['left']['recently_received'])
        for stamp in (0,8,12):
            with patch.object(diagnostics.time,'time',return_value=10.),patch.object(diagnostics.time,'monotonic',return_value=2.):
                observer.receive('right',self.message(sec=stamp))
            status=observer.sample(10.,2.)
            self.assertTrue(status['sides']['right']['recently_received'])
            self.assertFalse(status['sides']['right']['publication_fresh'])
            self.assertIsNone(status['sensor_valid'])
        self.assertFalse(observer.sample(10.,1.)['sides']['right']['recently_received'])

    def test_sampler_is_independent_per_side_and_keeps_invalidity_changes(self):
        sampler=diagnostics.DiagnosticSampler();kept=dict(left=0,right=0)
        value=dict(frame_id='sensor',finite=True,publication_fresh=True)
        for tick in range(1000):
            for side in kept:kept[side]+=sampler.keep('wrench-'+side,value,tick*.001)
        self.assertEqual(kept,dict(left=10,right=10))
        self.assertEqual(sampler.sampled_out,2000-sum(kept.values()))
        self.assertTrue(sampler.keep('wrench-left',dict(value,finite=False),.9991))
        self.assertTrue(sampler.keep('wrench-left',dict(value,publication_fresh=False),.9992))
        self.assertEqual(len(sampler.last),2)

    def test_async_writer_saves_both_sides_and_invalid_numbers_without_filling(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);writer=diagnostics.DiagnosticWriter(root/'diagnostics.jsonl')
            observer,_,_=self.observer(writer)
            observer.receive('left',self.message((float('nan'),1.,2.,3.,4.,5.)))
            observer.receive('right',self.message((0.,)*6,frame='right_sensor'))
            writer.close()
            left=json.loads((root/'wrench-left.jsonl').read_text())['data']
            right=json.loads((root/'wrench-right.jsonl').read_text())['data']
            self.assertEqual(left['force'][0],'nan');self.assertFalse(left['finite'])
            self.assertEqual(right['force'],[0.,0.,0.]);self.assertIsNone(right['sensor_valid'])
            self.assertTrue((root/'wrench-capture.jsonl').is_file())
            self.assertEqual(writer.status()['error'],'');self.assertEqual(writer.status()['dropped'],0)

    def test_live_observer_wires_force_capture_only_for_real_vendor_diagnostics(self):
        from contextlib import redirect_stdout
        from snowbot_bringup.visualization import snowbot_live
        for mode,backend in (('real','snowbot_vendor'),('simulation','kinematic')):
            with self.subTest(mode=mode),tempfile.TemporaryDirectory() as directory:
                root=Path(directory);model=root/'model.urdf';model.write_text('offline fixture')
                config=root/'tool.json';config.write_text('{}')
                robot=S(tool_config=config,urdf=model,display_urdf=model,profile=S(digest='profile',limits={'joint':(-1.,1.)}))
                subscriptions={};snapshots=[];alive=iter((True,False))
                node=S(create_subscription=lambda cls,topic,callback,qos:subscriptions.setdefault(topic,callback),destroy_node=lambda:None)
                def spin_once(node,**kwargs):
                    for side,topic in diagnostics.WrenchObserver.TOPICS.items():
                        if topic in subscriptions:subscriptions[topic](self.message(frame=side+'_sensor'))
                modules={'rclpy':S(init=lambda **kw:None,create_node=lambda name:node,ok=lambda:next(alive),
                                 spin_once=spin_once,try_shutdown=lambda:None,executors=S(ExternalShutdownException=RuntimeError)),
                    'rclpy.signals':S(SignalHandlerOptions=S(NO=0,ALL=1)),
                    'rclpy.qos':S(QoSProfile=lambda **kw:kw,ReliabilityPolicy=S(BEST_EFFORT='best_effort'),DurabilityPolicy=S(VOLATILE='volatile')),
                    'geometry_msgs':S(),'geometry_msgs.msg':S(WrenchStamped=object),
                    'std_msgs.msg':S(String=object),'rcl_interfaces.msg':S(Log=S(WARN=30)),
                    'bindu_interfaces.msg':S(**{k:object for k in ('ExecutionState','RuntimeEvent','VRInput','MotionCommand','GripperState','TeleopModeState')}),
                    'bindu_runtime.common':S(EVENT_QOS=object()),
                    'bindu_runtime.robot_setup':S(RobotSetup=S(load=lambda path:robot)),
                    'snowbot_bringup.visualization.snowbot_errors':S(EndEffectorErrors=lambda *a:S(sample=lambda *a:{}),pinocchio_fk=lambda *a:None)}
                args=S(robot_config=config,run_id='run',runtime_mode=mode,device_backend=backend,
                       state_file=None,trace_file=root/'diagnostics.jsonl',namespace='fixture',duration=0.)
                with patch.dict('sys.modules',modules),redirect_stdout(StringIO()):snowbot_live.observe(args,publish=snapshots.append)
                expected=mode=='real'
                self.assertEqual((root/'wrench-left.jsonl').exists(),expected)
                self.assertEqual((root/'wrench-right.jsonl').exists(),expected)
                self.assertEqual('wrench_capture' in snapshots[0],expected)
                if expected:self.assertEqual(snapshots[0]['wrench_capture']['sides']['left']['received'],1)


class SamplingTests(unittest.TestCase):
    def test_regular_input_is_capped_per_hand_before_ros_serialization(self):
        sampler=diagnostics.DiagnosticSampler()
        messages={side:S(side=side,source_id='vr',valid=True,button_edges=[0]*8,
                         grip=.8,trigger=0.) for side in ('left','right')}
        kept={side:0 for side in messages}
        for tick in range(1000):
            for side,msg in messages.items():
                msg.seq=tick
                kept[side]+=sampler.keep('vr-raw',msg,tick*.001)
        self.assertTrue(all(9 <= count <= 10 for count in kept.values()),kept)
        self.assertEqual(sampler.sampled_out,2000-sum(kept.values()))

    def test_edges_release_reconnect_invalid_and_clock_reversal_are_kept(self):
        sampler=diagnostics.DiagnosticSampler()
        msg=S(side='left',source_id='vr',valid=True,button_edges=[0]*8,grip=.8,trigger=0.)
        self.assertTrue(sampler.keep('vr-raw',msg,1.))
        self.assertFalse(sampler.keep('vr-raw',msg,1.001))
        for tick,change in enumerate((dict(button_edges=[1]+[0]*7),dict(grip=.2),
                dict(valid=False),dict(source_id='reconnected'),dict(trigger=.9))):
            msg.__dict__.update(change)
            self.assertTrue(sampler.keep('vr-raw',msg,1.002+tick*.001))
        self.assertTrue(sampler.keep('vr-raw',msg,.1))
        for i in range(1000):
            msg.source_id=str(i);sampler.keep('vr-raw',msg,2.+i*.001)
        self.assertEqual(len(sampler.last),1)

    def test_health_is_one_hz_with_immediate_fault_and_recovery(self):
        sampler=diagnostics.DiagnosticSampler()
        value=dict(valid=True,reasons=[],fault_mask=0)
        self.assertTrue(sampler.keep('pdo-health',value,0.))
        self.assertFalse(sampler.keep('pdo-health',value,.5))
        self.assertTrue(sampler.keep('pdo-health',dict(value,fault_mask=1),.501))
        self.assertTrue(sampler.keep('pdo-health',value,.502))
        self.assertFalse(sampler.keep('pdo-health',value,1.))
        self.assertTrue(sampler.keep('pdo-health',value,1.503))

    def test_feedback_command_ids_do_not_bypass_rate_and_stop_changes_are_kept(self):
        sampler=diagnostics.DiagnosticSampler()
        value=dict(instance_id='execution',state='RUNNING',code='OK',stop_failures=[])
        self.assertTrue(sampler.keep('execution-state',value,1.))
        self.assertFalse(sampler.keep('execution-state',dict(value,command_id='new',revision=20),1.001))
        self.assertTrue(sampler.keep('execution-state',dict(value,stop_failures=['STOP_UNCONFIRMED']),1.002))

    def test_idle_snapshots_thin_without_delaying_faults_or_active_motion(self):
        sampler=diagnostics.DiagnosticSampler()
        for stream,idle,fault,active in (
            ('execution-state',dict(state='CANCELED'),dict(code='FAULT'),dict(state='RUNNING')),
            ('teleop-state',dict(phase='locked',execution_state='CANCELED'),dict(stop_failure={'code':'FAULT'}),
             dict(phase='following',execution_state='RUNNING')),
            ('diagnostics',dict(execution='CANCELED',pico={'phase':'locked'}),dict(valid=False),
             dict(execution='RUNNING',pico={'phase':'following'}))):
            with self.subTest(stream=stream):
                self.assertTrue(sampler.keep(stream,idle,0.))
                self.assertFalse(sampler.keep(stream,idle,.2))
                self.assertTrue(sampler.keep(stream,idle,1.))
                self.assertTrue(sampler.keep(stream,dict(idle,**fault),1.01))
                self.assertTrue(sampler.keep(stream,idle,1.02))
                self.assertTrue(sampler.keep(stream,active,1.03))
                self.assertTrue(sampler.keep(stream,active,1.14))

    def test_continuous_qp_diagnostics_are_sampled_but_command_mapping_is_full(self):
        sampler=diagnostics.DiagnosticSampler()
        value=S(state='BIMANUAL_QP_RESULT',code='{"code":"OK","elapsed":0.01}')
        self.assertTrue(sampler.keep('runtime-events',value,1.))
        value.code='{"code":"OK","elapsed":0.02}'
        self.assertFalse(sampler.keep('runtime-events',value,1.001))
        value.code='{"code":"FAILED"}'
        self.assertTrue(sampler.keep('runtime-events',value,1.002))
        value.state='SNOWBOT_SEGMENT'
        self.assertTrue(all(sampler.keep('runtime-events',value,1.003) for _ in range(100)))
        self.assertTrue(all(sampler.keep('accepted-commands',{},1.003) for _ in range(100)))

    def test_submit_sampling_preserves_fault_neighbors_and_full_timing_statistics(self):
        for enabled in ('0','1'):
            with self.subTest(enabled=enabled), tempfile.TemporaryDirectory() as directory, patch.dict(
                    diagnostics.os.environ, {'BINDU_TIMING_TRACE':enabled}):
                root=Path(directory)
                writer=diagnostics.DiagnosticWriter(root/'diagnostics.jsonl')
                with patch.object(diagnostics.time,'monotonic',return_value=1.):
                    for seq in range(9):
                        if seq==5:
                            writer.record('runtime-events',dict(state='TARGET_REJECTED',task_id='t',code='expired'))
                        writer.record('runtime-events',dict(state='BIMANUAL_SUBMIT_TIMING',task_id='t',
                            command_id=str(seq),code=json.dumps(dict(input_seq=seq,submit_round_trip_s=.01*seq))))
                writer.close()
                events=[json.loads(line)['data'] for line in (root/'runtime-events.jsonl').read_text().splitlines()]
                ids=[int(e['command_id']) for e in events if e['state']=='BIMANUAL_SUBMIT_TIMING']
                self.assertEqual(ids,list(range(9)) if enabled=='1' else [0,5,6,7])
                self.assertEqual(sum(e['state']=='TARGET_REJECTED' for e in events),1)
                context=root/'submit-context.jsonl'
                if enabled=='0':
                    before=json.loads(context.read_text())['data']['before']
                    self.assertEqual([int(row['command_id']) for row in before],[2,3,4])
                else:
                    self.assertFalse(context.exists())
                counts=[json.loads(line)['data']['metrics']['submit_round_trip_s']['count']
                        for line in (root/'timing-summary.jsonl').read_text().splitlines()]
                self.assertEqual(sum(counts),9)

    def test_writer_exposes_intentional_sampling_separately_from_queue_loss(self):
        with tempfile.TemporaryDirectory() as directory:
            writer=diagnostics.DiagnosticWriter(Path(directory)/'diagnostics.jsonl')
            with patch.object(diagnostics.time,'monotonic',return_value=1.):
                for i in range(100):writer.record('vr-raw',dict(side='left',seq=i))
            writer.close()
            self.assertEqual(writer.status()['sampled_out'],99)
            self.assertEqual(writer.status()['written'],1)
            self.assertEqual(writer.status()['dropped'],0)
            health=json.loads((Path(directory)/'diagnostics-health.json').read_text())
            self.assertEqual(health['sampled_out'],99)


class RepeatedLogTests(unittest.TestCase):
    def setUp(self):
        self.events=[]
        self.repeat=diagnostics.RepeatedLogs(lambda stream,value,text:self.events.append((stream,value,text)))
        self.value=dict(name='servo_node',level=40,message='Motor 2 fault 0x001D',file='driver.cpp',line=20)

    def add(self,value=None,now=0.):
        self.repeat.add('rosout-log-events',self.value if value is None else value,now=now,wall=100.+now)

    def test_first_and_periodic_summary_count_all_observed_duplicates(self):
        for i in range(1001):self.add(now=i*.001)
        self.assertEqual(len(self.events),1)
        self.repeat.flush(5.)
        first,summary=[x[1] for x in self.events]
        self.assertEqual(first['kind'],'first')
        self.assertEqual(summary['suppressed_repeats'],1000)
        self.assertEqual(summary['observed_count'],1001)
        self.assertEqual((summary['first_observed_at'],summary['last_observed_at']),(100.,101.))
        self.assertEqual(self.repeat.coalesced,1000)
        self.repeat.flush(10.)
        self.assertEqual(len(self.events),2)

    def test_motor_code_level_source_and_callsite_are_never_merged(self):
        self.add()
        for change in (dict(message='Motor 3 fault 0x001D'),dict(message='Motor 2 fault 0x001E'),
                       dict(name='dual_arm_control_node'),dict(level=50),dict(line=21)):
            self.add(dict(self.value,**change),.001)
        self.assertEqual(len(self.events),6)

    def test_close_and_capacity_eviction_flush_counts_without_claiming_recovery(self):
        self.repeat.capacity=2
        self.add();self.add(now=.01)
        self.add(dict(self.value,message='other'),.02)
        self.add(dict(self.value,message='third'),.03)
        self.assertEqual(self.repeat.evicted,1)
        self.assertEqual(len(self.repeat.entries),2)
        self.assertEqual(self.events[2][1]['summary_reason'],'capacity_eviction')
        self.add(dict(self.value,message='third'),.04)
        self.repeat.flush(.05,final=True)
        self.assertEqual(self.events[-1][1]['summary_reason'],'close')
        self.assertTrue(all(x[1]['kind'] in ('first','repeat_summary') for x in self.events))
        self.assertEqual(len(self.repeat.entries),0)

    def test_console_time_and_ansi_are_ignored_but_error_numbers_are_not(self):
        a=diagnostics.console_log('\x1b[31m[node-1] [WARN] [10.1] [servo_node]: WKC 61/67\x1b[0m\n','control')
        b=diagnostics.console_log('[node-1] [WARN] [10.2] [servo_node]: WKC 61/67\n','control')
        c=diagnostics.console_log('[WARN] [10.3] [servo_node]: WKC 55/67\n','control')
        self.add(a);self.add(b,.01);self.add(c,.02)
        self.repeat.flush(final=True)
        self.assertEqual([event[1]['kind'] for event in self.events],['first','first','repeat_summary'])
        self.assertEqual(a['message'],'WKC 61/67')
        self.assertEqual(a['name'],'servo_node')
        self.assertIsNone(diagnostics.console_log('  File "[ERROR]", line 2\n','display'))

    def test_console_copy_preserves_tracebacks_and_coalesces_repeated_warning_lines(self):
        from snowbot_bringup.runtime.run_vr import copy_output
        stack='Traceback (most recent call last):\n  File "a.py", line 2\nValueError: invalid target\n'
        lines=''.join(f'[WARN] [{10+i*.001:.3f}] [servo_node]: pdo_stale\n' for i in range(1000))+stack
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            writer=diagnostics.DiagnosticWriter(root/'process-output.jsonl',health_filename='process-log-health.json')
            child=S(stdout=StringIO(lines))
            copy_output(child,root/'display.log',writer)
            self.assertTrue(child.stdout.closed)
            writer.close()
            saved=(root/'display.log').read_text()
            self.assertIn(stack,saved)
            self.assertEqual(saved.count('[WARN]'),1)
            self.assertIn('"suppressed_repeats": 999',saved)
            health=json.loads((root/'process-log-health.json').read_text())
            self.assertEqual(health['coalesced'],999)
            self.assertEqual(health['dropped'],0)

    def test_slow_disk_cannot_block_pipe_drain_and_loss_is_exposed(self):
        from snowbot_bringup.runtime.run_vr import copy_output
        gate=threading.Event();original=diagnostics.DiagnosticWriter.run
        def delayed(writer):gate.wait(2.);original(writer)
        with tempfile.TemporaryDirectory() as directory,patch.object(diagnostics.DiagnosticWriter,'run',delayed):
            root=Path(directory);writer=diagnostics.DiagnosticWriter(root/'process-output.jsonl',capacity=1)
            child=S(stdout=StringIO('ordinary output\n'*1000))
            reader=threading.Thread(target=copy_output,args=(child,root/'control.log',writer))
            try:
                reader.start();reader.join(timeout=1.)
                self.assertFalse(reader.is_alive())
                self.assertTrue(child.stdout.closed)
                self.assertEqual(writer.status()['dropped'],999)
            finally:gate.set();reader.join();writer.close()

    def test_rotated_text_keeps_its_log_extension_and_health_file_is_independent(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            writer=diagnostics.DiagnosticWriter(root/'process-output.jsonl',chunk_bytes=10,
                                               health_filename='process-log-health.json')
            for i in range(20):writer.write_text('control',str(i)+'\n')
            writer.close()
            files=[root/'control.log',*sorted(root.glob('control-*.log'))]
            self.assertEqual([int(row) for path in files for row in path.read_text().splitlines()],list(range(20)))
            self.assertTrue((root/'process-log-health.json').is_file())
            self.assertFalse((root/'diagnostics-health.json').exists())

    def test_two_pipe_readers_share_a_writer_without_losing_duplicate_counts(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);writer=diagnostics.DiagnosticWriter(root/'process-output.jsonl')
            def produce(stream):
                for _ in range(1000):writer.record_log(stream,self.value)
            workers=[threading.Thread(target=produce,args=(stream,)) for stream in ('control','display')]
            for worker in workers:worker.start()
            for worker in workers:worker.join()
            writer.close()
            for stream in ('control','display'):
                rows=[json.loads(line)['data'] for line in (root/(stream+'.jsonl')).read_text().splitlines()]
                self.assertEqual(rows[-1]['observed_count'],1000)
            self.assertEqual(writer.status()['coalesced'],1998)
            self.assertEqual(writer.status()['dropped'],0)

    def test_rosout_filters_non_vendor_and_info_without_creating_control_publishers(self):
        records=[]
        writer=S(record=lambda stream,value:records.append((stream,value)),
                 record_log=lambda stream,value:records.append((stream,value)))
        subscriptions=[]
        node=S(create_subscription=lambda *args:subscriptions.append(args))
        with patch.dict('sys.modules',{'rcl_interfaces':S(),'rcl_interfaces.msg':S(Log=S(WARN=b'\x1e')),
                                      'rclpy.qos':S(QoSProfile=lambda **kw:kw,
                                          ReliabilityPolicy=S(BEST_EFFORT='best_effort'),
                                          DurabilityPolicy=S(VOLATILE='volatile'))}):
            observer=diagnostics.RosLogObserver(node,writer,'run')
        self.assertEqual(subscriptions[0][1],'/rosout')
        self.assertEqual(json.loads(json.dumps(records[0][1]))['minimum_level'],30)
        self.assertEqual(subscriptions[0][3],dict(depth=100,reliability='best_effort',durability='volatile'))
        fields=dict(level=40,name='servo_node',msg='PDO stale',stamp=S(sec=10,nanosec=1),
                    file='driver.cpp',function='poll',line=7)
        observer.receive(S(**dict(fields,level=20)))
        observer.receive(S(**dict(fields,name='unrelated')))
        for name in observer.LOGGERS:observer.receive(S(**dict(fields,name=name)))
        self.assertEqual(observer.status()['received'],3)
        self.assertEqual(len(records),4)
        self.assertEqual(records[-1][1]['source_stamp_ns'],10000000001)
        self.assertTrue(records[-1][1]['source_stamp_before_capture'])
        self.assertFalse(observer.status()['plain_stdio_captured'])

    def test_only_explicit_prediction_info_is_retained(self):
        records=[]
        observer=diagnostics.RosLogObserver.__new__(diagnostics.RosLogObserver)
        observer.writer=S(record_log=lambda stream,value:records.append(value))
        observer.run_id='test';observer.started_ns=1;observer.received=0
        fields=dict(level=20,name='servo_node',msg='TARGET_PREDICTION {"sequence":7}',
                    stamp=S(sec=10,nanosec=0),file='servo.cpp',function='prepare',line=1)
        observer.receive(S(**fields))
        observer.receive(S(**dict(fields,msg='TARGET_PREDICTION_MODEL {}')))
        observer.receive(S(**dict(fields,msg='ordinary info')))
        observer.receive(S(**dict(fields,name='unrelated')))
        self.assertEqual(observer.received,2)
        self.assertEqual(records[0]['level'],20)

    def test_real_ros_log_qos_and_payload_are_supported_with_a_subscription_only_node(self):
        from rcl_interfaces.msg import Log
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);writer=diagnostics.DiagnosticWriter(root/'diagnostics.jsonl')
            observer=diagnostics.RosLogObserver(S(create_subscription=lambda *args:object()),writer,'test')
            for _ in range(3):observer.receive(Log(level=40,name='servo_node',msg='Error 0x001D'))
            writer.close()
            rows=[json.loads(line)['data'] for line in (root/'rosout-log-events.jsonl').read_text().splitlines()]
            self.assertEqual([row['kind'] for row in rows],['capture_started','first','repeat_summary'])
            self.assertEqual(rows[-1]['observed_count'],3)
            self.assertEqual(rows[-1]['suppressed_repeats'],2)
