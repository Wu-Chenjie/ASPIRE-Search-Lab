"""Translate the pinned RACER configuration to native ROS2 parameters/remaps."""
import ast
import math
import operator
from pathlib import Path
from sys import float_info
import yaml
from tools.research.frontier_baselines.replay import launch_nodes


def scalar(value, kind, limits):
    if value.startswith('$(eval ') and value.endswith(')'):
        expression=value[7:-1]
        for key, val in [('max_vel',limits['speed']),('max_acc',limits['acceleration'])]:
            expression=expression.replace("arg('"+key+"')",repr(val))
        ops={ast.Add:operator.add,ast.Sub:operator.sub,ast.Mult:operator.mul,ast.Div:operator.truediv}
        def evaluate(n):
            if isinstance(n,ast.Constant) and type(n.value) in (int,float):return n.value
            if isinstance(n,ast.BinOp) and type(n.op) in ops:return ops[type(n.op)](evaluate(n.left),evaluate(n.right))
            if isinstance(n,ast.UnaryOp) and isinstance(n.op,ast.USub):return -evaluate(n.operand)
            raise ValueError('Unsupported RACER parameter expression')
        value=str(evaluate(ast.parse(expression,mode='eval').body))
    if '$(' in value:raise ValueError('Unresolved RACER substitution: '+value)
    if kind=='auto':
        parsed=yaml.safe_load(value)
        if type(parsed) not in (str,int,float,bool):raise ValueError('Unsupported parameter scalar')
        if isinstance(parsed,float) and not math.isfinite(parsed):raise ValueError('Nonfinite parameter')
        return parsed
    if kind=='bool':
        if value.lower() not in ('true','false'):raise ValueError('Invalid boolean')
        return value.lower()=='true'
    if kind=='int':return int(value)
    if kind=='double':
        result=float(value)
        if not math.isfinite(result):raise ValueError('Nonfinite parameter')
        return result
    return value


def native_launch_nodes(output, bounds, intrinsics, limits, template, fleet_size, **geometry):
    for node in launch_nodes(bounds,intrinsics,Path(output),template,limits,fleet_size,**geometry):
        parameters = {'use_sim_time': True}
        parameters.update({p.get('name').replace('/', '.'):
            scalar(p.get('value'), p.get('type', 'auto'), limits) for p in node.findall('param')})
        executable = node.get('type')
        if executable == 'exploration_node':
            # The ROS1 XML labels this count double; C++ consumes an integer.
            parameters['exploration.drone_num'] = int(parameters['exploration.drone_num'])
            parameters['exploration.init_plan_num'] = 0 if fleet_size == 1 else 2
        yield dict(name=node.get('name'), executable=executable, parameters=parameters,
                   remappings=[(r.get('from'), r.get('to')) for r in node.findall('remap')])


def native_node_command(spec, output, executables):
    name = spec['name']
    path = output/(name + '.yaml')
    path.write_text(yaml.safe_dump({name: {'ros__parameters': spec['parameters']}}))
    args = [str(executables/spec['executable']), '--ros-args', '-r', '__node:='+name,
            '--params-file', str(path)]
    for source, target in spec['remappings']:
        args.extend(['-r', source+':='+target])
    return args


def static_event_parameters(hello):
    enabled=hello.get('static_event_replan',False)
    audit=hello.get('static_event_audit',False)
    interval=hello.get('static_event_min_interval_s',-1.)
    if type(enabled) is not bool or type(audit) is not bool:
        raise ValueError('Static event switches must be booleans')
    if type(interval) not in (int,float) or not math.isfinite(interval):
        raise ValueError('Static event interval must be finite')
    if audit and hello.get('confirmation_mode')!='registry':
        raise ValueError('Static event audit requires native registry')
    if enabled and (not audit or hello.get('stop_and_plan') is not True or
                    hello.get('bearing_bias') is not True or hello.get('execution_safety') is not True or
                    len(hello['starts'])!=1 or interval<=0):
        raise ValueError('Static events require single-aircraft safety, stop-and-plan, registry audit, bearing policy and explicit interval')
    weak=hello.get('static_weak_enabled',False)
    if type(weak) is not bool:raise ValueError('Static weak switch must be boolean')
    weak_limits={}
    for name in ('static_weak_trajectory_s','static_weak_extra_cost','static_weak_extra_path_m'):
        value=hello.get(name,-1.)
        usable=type(value) in (int,float) and -float_info.max<=value<=float_info.max
        if enabled and weak and (not usable or value<=0):
            raise ValueError('Single-frame evidence requires explicit low-cost limits')
        weak_limits['search.'+name]=float(value) if usable else -1.
    budget=hello.get('static_guidance_budget') or {}
    if not isinstance(budget,dict):raise ValueError('Static mission envelope must be an object')
    numeric=('duration_s','reserve_time_s','observation_s','initial_energy_j','reserve_energy_j','power_upper_w')
    if enabled:
        if set(budget)!=set(numeric)|{'model_id'} or not isinstance(budget['model_id'],str) or not 0<len(budget['model_id'])<=128:
            raise ValueError('Explicit static mission time/energy envelope and model identity required')
        if any(type(budget[k]) not in (float,int) or not math.isfinite(budget[k]) or budget[k]<0 for k in numeric):
            raise ValueError('Invalid static budget units or limits')
        if (budget['duration_s']<=budget['reserve_time_s'] or budget['initial_energy_j']<=budget['reserve_energy_j'] or budget['power_upper_w']<=0):
            raise ValueError('Static budget must preserve reserves')
    extra={('search.static_budget.'+k):float(budget.get(k,-1.)) for k in numeric}
    extra['search.static_budget.model_id']=str(budget.get('model_id',''))
    session=hello.get('static_cancel_session','')
    if not isinstance(session,str) or (session and (not session.isdecimal() or not 0<int(session)<2**53)):
        raise ValueError('Invalid static cancellation session')
    moving=hello.get('static_moving_c2',False);lead=hello.get('static_c2_lead_s',-1.)
    if lead is None and moving is False:lead=-1.
    if type(moving) is not bool or type(lead) not in (int,float) or not math.isfinite(lead):
        raise ValueError('Invalid moving C2 switch or lead')
    if moving and (len(hello.get('starts',[]))!=1 or hello.get('confirmation_mode')!='registry' or
                   hello.get('static_cancel_enabled') is not True or lead<=0 or
                   hello.get('execution_safety') is not True or hello.get('stop_and_plan') is not True):
        raise ValueError('Moving C2 requires explicit cancellation, safety, fallback stop-and-plan and positive lead')
    if moving and hello.get('min_time',True) is not False:
        raise ValueError('Moving C2 requires explicit min_time=false for fixed-knot boundary constraints')
    return {**extra,**weak_limits,'search.static_moving_c2':moving,'search.static_c2_lead_s':float(lead),'search.static_cancel_session':session,'search.static_weak_enabled':weak,'search.static_event_replan':enabled,'search.static_event_audit':audit,
            'search.static_event_min_interval_s':float(interval)}


def _static_feature_configuration(config):
    options = dict(config)
    starts = options.get('starts', options.get('fleet_starts', []))
    single_safety = options.get('execution_safety') is True and len(starts) == 1
    options['starts'] = starts
    if single_safety:
        options['stop_and_plan'] = True
    registry = options.get('confirmation_mode') == 'registry'
    session = options.get('static_cancel_session', '')
    cancel = (single_safety and options.get('static_cancel_enabled') is True
              and isinstance(session, str) and 0 < len(session) <= 16
              and session.isdecimal() and 0 < int(session) < 2**53)
    options['static_cancel_session'] = session if cancel else ''
    options['static_cancel_enabled'] = cancel
    options['min_time'] = options.get('min_time', options.get('racer_min_time', True))
    lead = options.get('static_c2_lead_s', -1.)
    moving = (cancel and registry and options.get('static_moving_c2') is True
              and options['min_time'] is False
              and type(lead) in (int, float) and math.isfinite(lead) and lead > 0)
    options['static_moving_c2'] = moving
    options['static_c2_lead_s'] = lead if moving else -1.
    options['static_event_audit'] = registry and options.get('static_event_audit') is True
    options['static_event_replan'] = single_safety and options.get('static_event_replan') is True
    try:
        parameters = static_event_parameters(options)
    except (TypeError, ValueError, OverflowError):
        options.update(static_event_replan=False, static_event_min_interval_s=-1.,
                       static_guidance_budget={}, static_weak_enabled=False,
                       static_weak_trajectory_s=-1., static_weak_extra_cost=-1.,
                       static_weak_extra_path_m=-1.)
        parameters = static_event_parameters(options)
    return options, parameters


def static_feature_options(config):
    """Select compatible optional static features for the live runtime."""
    return _static_feature_configuration(config)[0]


def configuration(hello, output, template):
    hello, event_parameters = _static_feature_configuration(hello)
    fleet_size=len(hello['starts'])
    execution_limits=dict(speed=.6,acceleration=.8,jerk=2.,yaw_rate=.65)
    execution_limits.update(hello.get('motion_limits',{}))
    # Upstream uses per-axis translation bounds and soft yaw waypoint costs.
    # Leave margin inside the executor's norm/peak-rate limits; never weaken
    # execution validation or retime/refit the published native curve.
    if hello.get('planning_limits') is not None:
        supplied=hello['planning_limits']
        if set(supplied)!=set(execution_limits) or any(
                not math.isfinite(v) or not 0<v<=execution_limits[k] for k,v in supplied.items()):
            raise ValueError('Invalid explicit planner envelope')
        limits=dict(supplied)
    else:
        scale=hello.get('planning_scale',.85)
        if type(scale) not in (int,float) or not 0<scale<=1:
            raise ValueError('Native planning scale must be in (0,1]')
        limits={key:value*(scale if key in ('speed','acceleration','yaw_rate') else 1.)
                for key,value in execution_limits.items()}
    nodes=list(native_launch_nodes(output, hello['bounds'], hello['intrinsics'], limits, template, fleet_size,
        voxel_resolution=hello.get('voxel_resolution',.3), body_clearance=hello.get('body_clearance',.6)))
    for node in nodes:
        if node['executable']=='exploration_node':
            params=node['parameters']
            i=int(params['exploration.drone_id'])-1
            params.update({**event_parameters,
                'frontier.task_vertical_samples':bool(hello.get('vertical_samples',False)),
                'search.no_path_recovery':bool(hello.get('no_path_recovery',False)),
                'search.stop_and_plan':bool(hello.get('stop_and_plan',False)),
                'search.active_review':bool(hello.get('active_review',False)),
                'search.target_registry':hello.get('confirmation_mode')=='registry',
                'search.bearing_bias':bool(hello.get('bearing_bias',False)),
                'search.review_continuation':bool(hello.get('review_continuation',False)),
                'search.review_max_segments':int(hello.get('review_max_segments',3)),
                'search.review_max_duration_s':float(hello.get('review_max_duration_s',60.)),
                'search.review_max_path_m':float(hello.get('review_max_path_m',6.)),
                'search.review_observation_hold_s':float(hello.get('review_observation_hold_s',5.)),
                'search.recover_frontiers':bool(hello.get('recover_frontiers',hello.get('active_review',False))),
                'fsm.authorized_execution':True,'fsm.replan_time':1.5,
                'manager.min_time':bool(hello.get('min_time',True)),
                'search.hard_rest_boundary':bool(hello.get('feasible_time',False)),
                'search.feasible_time':bool(hello.get('feasible_time',False)),
                **{'search.executor_'+k:float(v) for k,v in hello.get('motion_limits',{}).items()},
                'manager.max_jerk':limits['jerk'],'manager.max_yawdot':limits['yaw_rate'],
                'bspline.limit_ratio':1.,'sdf_map.box_min_z':hello['flight_limits'][0],
                'sdf_map.box_max_z':hello['flight_limits'][1]})
            node['remappings'] += [('/planning/execution_feedback',f'/baseline/feedback_{i}'),
                                   ('/planning/proposal_boundary',f'/baseline/boundary_{i}')]
    return nodes
