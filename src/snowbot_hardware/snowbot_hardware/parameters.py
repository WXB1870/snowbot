"""Shared startup parameters; loading never contacts hardware or starts services."""
import copy
import hashlib
import json
import math
from pathlib import Path
from functools import lru_cache


_FIELDS = {'teleop': ('input_max_age_s', 'feedback_max_age_s', 'command_ttl_s'),
 'solver': ('timeout_s', 'worker_timeout_s', 'startup_timeout_s'),
 'pico': ('remote_input_timeout_s', 'remote_source_timeout_s', 'controller_fps',
          'prepare_button_hold_s',
          'pair_max_skew_s',
          'grip_press_threshold',
          'grip_release_threshold',
          'trigger_press_threshold',
          'trigger_release_threshold',
          'prepare_command_period_s',
          'prepare_command_ttl_s',
          'reference_lookahead_s',
          'rpc_timeout_s',
          'stop_wait_margin_s',
          'renewal_period_s',
          'stop_renewal_period_s',
          'renewal_timeout_s',
          'motion_completion_grace_s',
          'validation_worker_timeout_s'),
 'gripper': ('vr_input_max_age_s', 'max_tick_gap_s', 'lease_ttl_s', 'request_ttl_s'),
 'replay': ('period_s', 'command_ttl_s', 'renewal_period_s', 'start_lead_s', 'state_max_age_s'),
 'execution': ('target_stream_brake_lead_s', 'stop_confirmation_window_s',
               'held_reference_velocity_tolerance_rad_s',
               'startup_hold_ttl_s',
               'startup_lease_ttl_s',
               'startup_home_waist_max_velocity_rad_s',
               'prepare_start_velocity_tolerance_rad_s',
               'motion_arrival_velocity_tolerance_rad_s'),
 'diagnostics': ('state_period_s',
                 'health_period_s',
                 'wrench_period_s',
                 'input_gap_warning_s',
                 'publish_warning_s',
                 'pdo_before_s',
                 'pdo_after_s',
                 'pdo_capacity',
                 'pdo_sample_hz',
                 'queue_capacity',
                 'chunk_bytes'),
 'display': ('poll_ms', 'request_timeout_ms', 'stale_ms', 'reconnect_ms', 'report_ms')}
_INTEGER_FIELDS = frozenset(('controller_fps','pdo_capacity','queue_capacity','chunk_bytes','poll_ms','request_timeout_ms','stale_ms','reconnect_ms','report_ms'))


def default_path():
    from bindu_contracts.resources import package_share
    return package_share('snowbot_bringup')/'config/robots/snowbot/runtime_parameters.json'


def _parameter_object(values):
    """Discard JSON comment metadata before validating runtime settings."""
    values.pop('_comment', None)
    return values


@lru_cache(maxsize=1)
def _defaults():
    result = json.loads(default_path().read_text(), object_hook=_parameter_object)
    if (set(result) != {'schema_version', *_FIELDS} or
            type(result.get('schema_version')) is not int or result['schema_version'] != 1 or
            any(not isinstance(result[g],dict) or set(result[g]) != set(keys) for g,keys in _FIELDS.items())):
        raise ValueError('SNOWBOT_PARAMETERS_SCHEMA')
    return result


def load_parameters(path=None):
    result = copy.deepcopy(_defaults())
    raw = json.loads(Path(path).read_text(), object_hook=_parameter_object) if path else {}
    if (not isinstance(raw, dict) or type(raw.get('schema_version', 1)) is not int
            or raw.get('schema_version', 1) != 1):
        raise ValueError('SNOWBOT_PARAMETERS_SCHEMA')
    raw.pop('schema_version', None)
    # Existing site files may still select a connection or nest their overrides.
    raw.pop('vr_connection', None)
    legacy_overrides = raw.pop('remote_vr', {})
    for layer in (raw, legacy_overrides):
        if not isinstance(layer, dict) or set(layer)-set(_FIELDS):
            raise ValueError('SNOWBOT_PARAMETERS_SCHEMA')
        for group, values in layer.items():
            allowed = set(result[group])
            if not isinstance(values, dict) or set(values)-allowed:
                raise ValueError('SNOWBOT_PARAMETERS_GROUP:'+group)
            result[group].update(values)
    for group, values in result.items():
        if group == 'schema_version':
            continue
        for key, value in values.items():
            try: finite = type(value) in (int, float) and math.isfinite(value) and value > 0
            except OverflowError: finite = False
            if (not finite or
                    key in _INTEGER_FIELDS and type(value) is not int):
                raise ValueError('SNOWBOT_PARAMETER:'+group+'.'+key)
            if key not in _INTEGER_FIELDS:
                values[key] = float(value)
    t, s = result['teleop'], result['solver']
    if not t['input_max_age_s'] <= t['command_ttl_s'] <= .8:
        raise ValueError('SNOWBOT_TELEOP_FRESHNESS')
    if (not math.isfinite(s['timeout_s']*1e6) or round(s['timeout_s']*1e6) < 1 or
            s['timeout_s'] >= s['worker_timeout_s']):
        raise ValueError('SNOWBOT_SOLVER_TIMING')
    lead = result['execution']['target_stream_brake_lead_s']
    if lead >= .4 or not 0 < round(lead*1e9) < 400_000_000:
        raise ValueError('SNOWBOT_TARGET_BRAKE_LEAD')
    replay = result['replay']
    if (replay['period_s']+lead >= replay['command_ttl_s'] or
            replay['command_ttl_s'] > .4 or replay['start_lead_s'] >= replay['command_ttl_s'] or
            replay['start_lead_s'] > .15 or replay['renewal_period_s'] >= 2.):
        raise ValueError('SNOWBOT_REPLAY_TIMING')
    p, g, d = (result[name] for name in ('pico', 'gripper', 'diagnostics'))
    for name in ('grip', 'trigger'):
        if not 0 < p[name+'_release_threshold'] < p[name+'_press_threshold'] < 1:
            raise ValueError('SNOWBOT_INPUT_HYSTERESIS:'+name)
    if not p['prepare_command_period_s'] < p['prepare_command_ttl_s'] <= .4 or g['request_ttl_s'] > .4:
        raise ValueError('SNOWBOT_REQUEST_TIMING')
    if result['execution']['startup_hold_ttl_s'] > .4:
        raise ValueError('SNOWBOT_STARTUP_HOLD_TTL')
    if not math.isfinite(d['pdo_before_s']*d['pdo_sample_hz']) or not math.ceil(d['pdo_before_s']*d['pdo_sample_hz']) <= d['pdo_capacity'] <= 100000:
        raise ValueError('SNOWBOT_PDO_PREHISTORY_CAPACITY')
    return result


def validate_pico_timing(parameters, *, brake_lead_s=0.):
    """Check the selected PICO timing before launching its control chain."""
    p, g = parameters['pico'], parameters['gripper']
    if brake_lead_s and brake_lead_s >= parameters['teleop']['command_ttl_s']:
        raise ValueError('SNOWBOT_TARGET_BRAKE_LEAD_EXCEEDS_TTL')
    # ExecutionNode grants and Executor renews arm leases for two seconds.
    arm_lease_s = 2.
    for key, ttl in (('renewal_period_s', min(arm_lease_s, g['lease_ttl_s'])),
                     ('stop_renewal_period_s', arm_lease_s)):
        if p[key] >= ttl:
            raise ValueError(f'SNOWBOT_PICO_RENEWAL_TIMING:{key} must be < {ttl:g}s')
    if p['prepare_command_period_s'] + brake_lead_s >= p['prepare_command_ttl_s']:
        raise ValueError('SNOWBOT_PREPARE_TIMING: prepare_command_period_s + '
                         'target_stream_brake_lead_s must be < prepare_command_ttl_s')


def parameters_for(owner, group):
    """Use the selected startup snapshot, including lightweight offline fixtures."""
    selected = getattr(owner, 'runtime_parameters', None)
    return selected[group] if isinstance(selected, dict) else load_parameters()[group]


def apply_tool_timing(config, parameters):
    """Apply the selected site timing before constructing sessions and workers."""
    from bindu_contracts.teleoperation import tool_with_timing
    result = tool_with_timing(config, parameters['teleop'], parameters['solver'])
    result['input_max_age'] = parameters['pico']['remote_input_timeout_s']
    gap = result.get('integration_max_sample_gap', result.get('max_integration_interval', .1))
    if gap > result['input_max_age']:
        raise ValueError('BIMANUAL_SAMPLE_GAP_EXCEEDS_INPUT_TIMEOUT')
    return result


def select_mode_config(config, mode):
    """Expand a shared teleop/device file for one explicit runtime mode."""
    if mode not in ('real', 'simulation') or not isinstance(config, dict):
        raise ValueError('SNOWBOT_CONFIG_RUNTIME_MODE')
    result = copy.deepcopy(config)
    has_modes = 'modes' in result
    modes = result.pop('modes', None)
    if has_modes:
        if (not isinstance(modes, dict) or set(modes) != {'real', 'simulation'} or
                any(not isinstance(v, dict) or 'modes' in v for v in modes.values())):
            raise ValueError('SNOWBOT_CONFIG_MODES')
        def merge(base, changes):
            for key, value in changes.items():
                if isinstance(value, dict) and isinstance(base.get(key), dict):
                    merge(base[key], value)
                else:
                    base[key] = value
        merge(result, modes[mode])
    if result.get('simulation_only', False) is not (mode == 'simulation'):
        raise ValueError('SNOWBOT_CONFIG_MODE_MISMATCH')
    return result


def load_mode_config(path, mode):
    return select_mode_config(json.loads(Path(path).read_text()), mode)


def mode_config_digest(config, mode):
    """Bind device feedback to the resolved values and their runtime mode."""
    if mode not in ('real', 'simulation') or 'modes' in config:
        raise ValueError('SNOWBOT_CONFIG_RUNTIME_MODE')
    raw = json.dumps({'runtime_mode': mode, 'configuration': config},
                     sort_keys=True, separators=(',', ':'), allow_nan=False)
    return hashlib.sha256(raw.encode()).hexdigest()
