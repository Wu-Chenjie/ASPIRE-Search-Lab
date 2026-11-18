import copy
import json
import subprocess
import sys
from pathlib import Path
import pytest
sys.path[:0]=[str(Path(__file__).resolve().parents[1]),str(Path(__file__).resolve().parents[1]/'src')]
from person_search.static_pair import apply_static_settings,prepare_pair,common_runtime_digest

ROOT=Path(__file__).resolve().parents[1]


def fixture():
    runtime=dict(search_starts=[[0,0,2.2]],confirmation_mode='registry',stop_and_plan=True,
                 execution_safety=True,min_time=False,detector_backend='synthetic_frozen_model',seed=7)
    settings=dict(static_event_min_interval_s=.5,static_weak_enabled=True,
                  static_weak_trajectory_s=2.,static_weak_extra_cost=.1,static_weak_extra_path_m=.1,
                  static_guidance_budget=dict(duration_s=100.,reserve_time_s=10.,observation_s=1.,
                    initial_energy_j=10000.,reserve_energy_j=100.,power_upper_w=10.,model_id='synthetic_contract'))
    return runtime,settings


def test_pair_changes_only_guidance_switches_and_keeps_inputs_immutable():
    runtime,settings=fixture();before=copy.deepcopy((runtime,settings));pair=prepare_pair(runtime,settings)
    a,b=pair['baseline'],pair['guided']
    assert {k for k in a if a[k]!=b[k]}=={'bearing_bias','static_event_replan','static_pair'}
    assert not a['bearing_bias'] and not a['static_event_replan']
    assert b['bearing_bias'] and b['static_event_replan']
    assert a['static_event_audit'] and b['static_event_audit']
    assert common_runtime_digest(a)==common_runtime_digest(b)
    assert a==apply_static_settings(runtime,settings,guided=False)
    assert b==apply_static_settings(runtime,settings,guided=True)
    assert (runtime,settings)==before
    changed=copy.deepcopy(b);changed['seed']=8
    assert common_runtime_digest(changed)!=common_runtime_digest(a)
    b['fleet_starts'][0][0]=8.
    b['static_guidance_budget']['duration_s']=120.
    assert a['fleet_starts']==runtime['search_starts']
    assert a['static_guidance_budget']==settings['static_guidance_budget']


def test_baseline_cannot_hide_invalid_settings_or_mismatched_harness_budget():
    runtime,settings=fixture()
    for malformed in ({},dict(settings,static_weak_extra_path_m=0),dict(settings,unknown_typo=1)):
        with pytest.raises(ValueError):apply_static_settings(runtime,malformed,guided=False)
    with pytest.raises(ValueError):apply_static_settings(runtime,settings,guided=True,seconds=101,budget_clock='sim')
    with pytest.raises(ValueError):apply_static_settings(runtime,settings,guided=True,seconds=100,budget_clock='wall')
    assert apply_static_settings(runtime,settings,guided=False,seconds=100,budget_clock='sim')['static_pair']['arm']=='baseline'


def test_optional_cancellation_requires_explicit_true_without_blocking_pair_setup():
    from tools.racer_ros2.configuration import static_feature_options

    runtime, settings = fixture()
    runtime['static_cancel_enabled'] = True
    for value in ('false', 'true', 1, None, False, True):
        pair = prepare_pair(runtime, dict(settings, static_cancel_enabled=value))
        for arm in pair.values():
            execution = static_feature_options(dict(arm, static_cancel_enabled=value,
                                                     static_cancel_session='123'))
            assert arm['static_cancel_enabled'] is execution['static_cancel_enabled'] is (value is True)
        assert common_runtime_digest(pair['baseline']) == common_runtime_digest(pair['guided'])
    runtime['static_cancel_enabled'] = 'false'
    assert prepare_pair(runtime, dict(settings, static_cancel_enabled=True))['guided']['static_cancel_enabled'] is True


def test_cli_prepares_only_configs_and_refuses_overwrite(tmp_path):
    runtime,settings=fixture();r=tmp_path/'runtime.json';s=tmp_path/'settings.json';out=tmp_path/'pair'
    r.write_text(json.dumps(runtime));s.write_text(json.dumps(settings))
    command=[sys.executable,'-B',str(ROOT/'tools/search/prepare_static_pair.py'),'--runtime',str(r),'--settings',str(s),'--output',str(out)]
    result=subprocess.run(command,check=True,capture_output=True,text=True)
    assert not json.loads(result.stdout)['experiment_started']
    assert sorted(p.name for p in out.iterdir())==['baseline.json','guided.json']
    before=(out/'baseline.json').read_bytes()
    assert subprocess.run(command,capture_output=True).returncode!=0
    assert (out/'baseline.json').read_bytes()==before


def test_invalid_flight_pair_falls_back_before_core_dependency_failure(tmp_path):
    prepared=tmp_path/'prepared'
    (prepared/'models').mkdir(parents=True)
    (prepared/'scene.sdf').write_text('<sdf><world/></sdf>')
    (prepared/'bridge.yaml').write_text('[]')
    (prepared/'runtime.json').write_text(json.dumps(dict(layout='original',performance_profile='legacy',
        vehicles=[{}],fleet_starts=[[0,0,2.2]],static_pair={'arm':'guided'})))
    out=tmp_path/'base-run'
    result=subprocess.run([sys.executable,'-B',str(ROOT/'tools/flight/run.py'),
        '--output',str(out),'--prepared',str(prepared),'--external',str(tmp_path/'missing-dependencies'),
        '--static-guidance-settings',str(tmp_path/'missing-settings.json')],
        capture_output=True,text=True)
    assert result.returncode==1 and 'Existing local SITL dependencies unavailable' in result.stderr
    assert 'static_pair:' in result.stderr
    runtime=json.loads((out/'runtime.json').read_text())
    assert 'static_pair' not in runtime and not runtime['static_event_replan']


def test_moving_c2_is_default_off_and_common_to_both_arms_with_explicit_lead():
    from tools.racer_ros2.configuration import static_event_parameters
    runtime, settings=fixture()
    pair=prepare_pair(runtime,dict(settings,static_moving_c2=False,static_c2_lead_s=None))
    for arm in pair.values():
        params=static_event_parameters(dict(arm,starts=arm['search_starts']))
        assert params['search.static_moving_c2'] is False
        assert params['search.static_c2_lead_s']==-1.
    settings.update(static_cancel_enabled=True,static_moving_c2=True,static_c2_lead_s=1.)
    pair=prepare_pair(runtime,settings)
    for arm in pair.values():
        params=static_event_parameters(dict(arm,starts=arm['search_starts']))
        assert params['search.static_moving_c2'] and params['search.static_c2_lead_s']==1.
    assert common_runtime_digest(pair['baseline'])==common_runtime_digest(pair['guided'])
    for bad in ({'static_moving_c2':'true'},{'static_c2_lead_s':None},
                {'static_c2_lead_s':0.},{'static_c2_lead_s':float('nan')},
                {'static_cancel_enabled':False},{'static_cancel_enabled':'true'}):
        with pytest.raises(ValueError):prepare_pair(runtime,dict(settings,**bad))
    for bad in ({'min_time':True},{'min_time':None},{'execution_safety':False},{'stop_and_plan':False},
                {'search_starts':[[0,0,1],[1,0,1]]},{'confirmation_mode':'visual'}):
        with pytest.raises(ValueError):prepare_pair(dict(runtime,**bad),settings)


def test_prepared_flight_schema_and_native_hello_validate_the_same_c2_settings():
    from tools.racer_ros2.configuration import static_event_parameters
    runtime,settings=fixture()
    runtime['fleet_starts']=runtime.pop('search_starts')
    runtime['racer_min_time']=runtime.pop('min_time')
    settings.update(static_cancel_enabled=True,static_moving_c2=True,static_c2_lead_s=1.)
    pair=prepare_pair(runtime,settings)
    for arm in pair.values():
        # These are the exact field mappings in NativeLeaseAgent.camera().
        hello=dict(arm,starts=arm['fleet_starts'],min_time=arm['racer_min_time'])
        assert static_event_parameters(hello)['search.static_moving_c2']
    for bad in ({'racer_min_time':True},{'racer_min_time':None},
                {'min_time':True},{'search_starts':[[1,0,2.2]]},
                {'fleet_starts':[[0,0,2.2],[1,0,2.2]]}):
        with pytest.raises(ValueError):prepare_pair(dict(runtime,**bad),settings)
    alias_runtime,settings=fixture()
    alias_pair=prepare_pair(alias_runtime,settings)
    assert alias_pair['guided']['fleet_starts']==alias_runtime['search_starts']
    assert alias_pair['guided']['racer_min_time'] is False
