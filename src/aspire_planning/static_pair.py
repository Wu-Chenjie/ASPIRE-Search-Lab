"""Prepare and verify paired runtime settings without starting a simulator."""
import copy
from aspire_interfaces.provenance import protocol_digest as digest

STATIC_SETTING_KEYS = frozenset({
    'static_event_min_interval_s', 'static_guidance_budget', 'static_weak_enabled',
    'static_weak_trajectory_s', 'static_weak_extra_cost', 'static_weak_extra_path_m',
    'static_visibility_config', 'static_visibility_model_id',
    'static_cancel_enabled',
    'static_moving_c2', 'static_c2_lead_s',
})
ARM_SWITCHES = frozenset({'bearing_bias', 'static_event_replan'})


def common_runtime_digest(runtime):
    excluded = ARM_SWITCHES | {'static_pair'}
    return digest({k: v for k, v in runtime.items() if k not in excluded})


def apply_static_settings(runtime, settings, *, guided, seconds=None, budget_clock=None):
    """Both arms share every setting except the two guidance switches.

    Values must be supplied explicitly from validation. This function does not
    decide that they are calibrated or authorize any experiment.
    """
    from tools.racer_ros2.configuration import static_event_parameters
    from aspire_interfaces.visibility import visibility_ros_arguments
    if not isinstance(runtime,dict) or not isinstance(settings,dict) or type(guided) is not bool:
        raise ValueError('Runtime/settings objects and boolean arm required')
    unknown = settings.keys() - STATIC_SETTING_KEYS
    if unknown:
        raise ValueError('Unknown static setting: ' + ','.join(sorted(map(str, unknown))))
    result=copy.deepcopy(runtime);result.pop('static_pair',None);result.update(copy.deepcopy(settings))
    if 'static_cancel_enabled' in result:
        result['static_cancel_enabled']=result['static_cancel_enabled'] is True
    result.update(static_event_audit=True,bearing_bias=guided,static_event_replan=guided)
    aliases = [result[key] for key in ('fleet_starts', 'search_starts', 'starts') if key in result]
    starts = aliases[0] if aliases else None
    if any(value != starts for value in aliases[1:]):
        raise ValueError('Conflicting static fleet start fields')
    min_time = result.get('racer_min_time', result.get('min_time', True))
    if 'min_time' in result and result['min_time'] != min_time:
        raise ValueError('Conflicting static timing fields')
    if not isinstance(starts,list):raise ValueError('Static paired runtime requires a fleet start list')
    # Keep the validated aliases aligned with the actual NativeLeaseAgent input.
    result['fleet_starts']=copy.deepcopy(starts)
    result['racer_min_time']=min_time
    # Validate treatment requirements even while generating baseline. Incomplete
    # settings must not pass unnoticed merely because that arm disables guidance.
    static_event_parameters(dict(result,starts=starts,min_time=min_time,bearing_bias=True,static_event_replan=True))
    visibility_ros_arguments(result)
    duration=result['static_guidance_budget']['duration_s']
    if seconds is not None and (type(seconds) not in (int,float) or seconds!=duration):
        raise ValueError('Harness time budget must equal the declared mission duration')
    if budget_clock is not None and budget_clock!='sim':
        raise ValueError('Paired static searches require the same simulation-time budget')
    result['static_pair']=dict(arm='guided' if guided else 'baseline',common_runtime_sha256=common_runtime_digest(result))
    return result


def prepare_pair(runtime, settings):
    baseline=apply_static_settings(runtime,settings,guided=False)
    guided=copy.deepcopy(baseline)
    guided.update(dict.fromkeys(ARM_SWITCHES,True))
    guided['static_pair']['arm']='guided'
    return dict(baseline=baseline,guided=guided)
