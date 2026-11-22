"""Offline synthetic sweep: which projection stride still samples a thin obstacle?

Scene: 4 m wall, one vertical bar of a given pixel width at 1.5 m, worst-case
phase relative to the sampling grid. Synthetic only; never used online.
"""
import json
import sys
import time
from pathlib import Path
ROOT=Path(r"D:\学习\大创\ASPIRE-Search-Lab")
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'src/annual_planner/src')]
import numpy as np
from aspire_camera.depth_projection import project_depth, SENSOR_TRANSLATION
from core.exploration.voxel_mapping import VoxelMap

FX=525.

def focal(width):return width*0.866  # same ~60 deg horizontal field of view at both resolutions

def scene(width,height,bar_px,phase,distance):
    z=np.full((height,width),4.,np.float32);truth=np.zeros((height,width),bool)
    left=width//2+phase
    z[:,left:left+bar_px]=distance;truth[:,left:left+bar_px]=True
    k=np.array([[focal(width),0.,width/2-.5],[0.,focal(width),height/2-.5],[0.,0.,1.]])
    return z,k,truth

def run(width,height,stride,bar_px,phase,distance,resolution=0.3):
    z,k,truth=scene(width,height,bar_px,phase,distance)
    rows=np.arange(0,height,stride);cols=np.arange(0,width,stride)
    rr,cc=np.meshgrid(rows,cols,indexing='ij')
    valid=(z[::stride,::stride]>=.2)&(z[::stride,::stride]<19.1)
    rr,cc=rr[valid],cc[valid]
    obstacle=truth[rr,cc]
    points,hits=project_depth(z,k,stride=stride)
    voxel=VoxelMap(np.array([[-4.,-6.,-1.],[8.,6.,4.5]]),resolution=resolution)
    origin=np.zeros(3)+SENSOR_TRANSLATION;world=points+origin
    started=time.perf_counter();cells,values=voxel.integrate(origin,world,hits);elapsed=time.perf_counter()-started
    wall=points[~obstacle]
    obstacle_cells=voxel.indices(world[obstacle]) if obstacle.any() else np.empty((0,3),int)
    ids=np.unique(np.ravel_multi_index(tuple(np.clip(obstacle_cells,0,np.array(voxel.shape)-1).T),voxel.shape)) if len(obstacle_cells) else np.empty(0,int)
    return dict(width=width,height=height,stride=stride,bar_px=bar_px,phase=phase,distance=distance,samples=len(points),
        obstacle_samples=int(obstacle.sum()),obstacle_cells=int(len(ids)),occupied_cells=int((values==1).sum()),
        wall_range_p95_error_m=float(np.quantile(np.abs(wall[:,0]-4.),.95)) if len(wall) else None,
        integrate_s=elapsed)

if __name__=='__main__':
    rows=[]
    for width,height in ((320,240),(640,480)):
        for distance in (1.5,4.):
            for bar_px in (1,2,3,4,6,8,12,16):
                for stride in (8,4,2):
                    rows.append(run(width,height,stride,bar_px,phase=1,distance=distance))
    out=Path(__file__).with_name('stride_sweep.json');out.write_text(json.dumps(rows,indent=2))
    for size in ('320x240','640x480'):
        for distance in (1.5,4.):
            for stride in (8,4,2):
                hits=[r['bar_px'] for r in rows if '%dx%d'%(r['width'],r['height'])==size and r['distance']==distance
                      and r['stride']==stride and r['obstacle_cells']>0]
                timing=[r['integrate_s']*1000 for r in rows if '%dx%d'%(r['width'],r['height'])==size
                        and r['distance']==distance and r['stride']==stride][0]
                print('%-8s d=%.1fm stride=%d thinnest_kept_px=%s thinnest_kept_mm=%.1f integrate_ms=%.1f'%(
                    size,distance,stride,min(hits) if hits else 'none',
                    (min(hits)*distance/focal(320 if size=='320x240' else 640)*1000) if hits else float('nan'),timing))
