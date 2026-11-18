"""Independent invariants on a recorded native map, no scene truth needed."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.ndimage import binary_dilation


def audit(folder):
    folder=Path(folder);d=np.genfromtxt(folder/'map.csv',delimiter=',',names=True)
    axes=[np.unique(d[k]) for k in 'xyz'];shape=tuple(map(len,axes));res=float(np.median(np.diff(axes[0])))
    occ=d['occupancy'].reshape(shape);actual=d['inflated'].reshape(shape)==1
    n=int(np.ceil(.6/res-1e-8));expected=binary_dilation(occ==2,structure=np.ones((2*n+1,)*3,bool))
    unexpected=actual&~expected;missing=expected&~actual
    summary=dict(resolution=res,shape=shape,unknown=int(sum(d['occupancy']==0)),free=int(sum(d['occupancy']==1)),occupied=int(sum(d['occupancy']==2)),
        actual_inflated=int(actual.sum()),expected_inflated=int(expected.sum()),inflated_without_current_obstacle=int(unexpected.sum()),
        missing_inflation=int(missing.sum()),extra_on_known_free=int((unexpected&(occ==1)).sum()),missing_on_known_free=int((missing&(occ==1)).sum()),
        note='Expected = one cubic dilation of current native occupied cells, with original ceil(clearance/resolution). This is map consistency, not geometry or flight acceptance.')
    np.savez_compressed(folder/'dilation-audit.npz',unexpected=np.argwhere(unexpected),missing=np.argwhere(missing),origin=[a[0] for a in axes],resolution=res)
    (folder/'dilation-audit.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary));return summary

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder');a=p.parse_args();audit(a.folder)
