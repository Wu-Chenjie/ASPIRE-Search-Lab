"""Visible-mask augmentation and target-domain annotation audit, no training claims."""
import numpy as np

POSTURES={'standing','sitting','lying','unknown'}
VISIBILITIES={'full','partial','heavily_occluded','unknown'}

def occlude_visible_masks(rgb,masks,occluder,colour=(30,30,30)):
    rgb=np.asarray(rgb);masks=np.asarray(masks,bool);occluder=np.asarray(occluder,bool)
    if rgb.dtype!=np.uint8 or rgb.ndim!=3 or rgb.shape[2]!=3 or masks.ndim!=3 or masks.shape[1:]!=rgb.shape[:2] or occluder.shape!=rgb.shape[:2]:raise ValueError('Registered RGB/visible masks required')
    image=rgb.copy();image[occluder]=colour
    visible = masks & ~occluder
    # Empty fragments are removed; no amodal pixels become depth support.
    keep=visible.any(axis=(1,2))
    return image,visible[keep],np.flatnonzero(keep)

def audit_annotations(records):
    """Require capture-group splits so adjacent frames cannot leak into test."""
    groups={};counts={};seen=set();errors=[]
    for index,row in enumerate(records):
        try:
            ident=row['observation_id'];group=row['capture_group'];split=row['split']
            if ident in seen or not ident or not group:raise ValueError('Duplicate/missing observation identity')
            seen.add(ident)
            if split not in ('train','val','test'):raise ValueError('Unknown dataset split')
            if groups.setdefault(group, split) != split:raise ValueError('Capture group leaks across splits')
            if not row.get('source') or not row.get('license'):raise ValueError('Dataset provenance and license required')
            for instance in row.get('instances',[]):
                if instance['class'] not in ('person','partial_human_hint'):raise ValueError('Unsupported human label')
                if instance.get('posture','unknown') not in POSTURES or instance.get('visibility','unknown') not in VISIBILITIES:raise ValueError('Invalid attributes')
                if instance.get('mask_type')!='visible':raise ValueError('Only visible masks may support geometry')
        except (KeyError,TypeError,ValueError) as error:
            errors.append(f'records[{index}]: {error}')
            continue
        counts[split]=counts.get(split,0)+1
    if errors:
        raise ValueError('\n'.join(errors))
    return dict(records=len(seen),capture_groups=len(groups),split_counts=counts)
