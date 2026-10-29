"""Shared startup parameters; loading never contacts hardware or starts services."""
import copy
import json
import math
from pathlib import Path
from functools import lru_cache


_FIELDS = {'pico': ('prepare_button_hold_s',
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
 'execution': ('stop_confirmation_window_s',
               'held_reference_velocity_tolerance_rad_s',
               'startup_hold_ttl_s',
               'startup_lease_ttl_s',
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
_INTEGER_FIELDS = frozenset(('pdo_capacity','queue_capacity','chunk_bytes','poll_ms','request_timeout_ms','stale_ms','reconnect_ms','report_ms'))


def default_path():
    from bindu_contracts.resources import package_share
    return package_share('snowbot_bringup')/'config/robots/snowbot/runtime_parameters.json'


@lru_cache(maxsize=1)
def _defaults():
    result = json.loads(default_path().read_text())
    if (set(result) != {'schema_version', *_FIELDS} or
            type(result.get('schema_version')) is not int or result['schema_version'] != 1 or
            any(not isinstance(result[g],dict) or set(result[g]) != set(keys) for g,keys in _FIELDS.items())):
        raise ValueError('SNOWBOT_PARAMETERS_SCHEMA')
    return result


def load_parameters(path=None):
    result = copy.deepcopy(_defaults())
    if path:
        raw = json.loads(Path(path).read_text())
        if (not isinstance(raw, dict) or set(raw)-set(result) or
                type(raw.get('schema_version', 1)) is not int or raw.get('schema_version', 1) != 1):
            raise ValueError('SNOWBOT_PARAMETERS_SCHEMA')
        for group, values in raw.items():
            if group == 'schema_version':
                continue
            if not isinstance(values, dict) or set(values)-set(result[group]):
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


def parameters_for(owner, group):
    """Use the selected startup snapshot, including lightweight offline fixtures."""
    selected = getattr(owner, 'runtime_parameters', None)
    return selected[group] if isinstance(selected, dict) else load_parameters()[group]
