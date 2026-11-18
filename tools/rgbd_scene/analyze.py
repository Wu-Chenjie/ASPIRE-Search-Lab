#!/usr/bin/env python3
"""Independent known-board image alignment diagnostic; never a planner input."""
import argparse
import json
from pathlib import Path
import numpy as np
from PIL import Image
from scipy.ndimage import label, binary_erosion, distance_transform_edt


def largest(mask):
    labels,n=label(mask)
    if not n:raise ValueError('No board component')
    counts=np.bincount(labels.ravel());counts[0]=0
    return labels==counts.argmax()


def main():
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args()
    views=json.loads((a.run/'viewpoints.json').read_text())['views'];result={}
    for name in ('visible','partial','hidden','revealed'):
        folder=a.run/name;depth=np.load(folder/'depth.npy');rgb=np.asarray(Image.open(folder/'rgb.png')).astype(float)
        record=json.loads((folder/'frame.json').read_text());k=record['depth']['k']
        v,u=np.indices(depth.shape);origin=np.array(views[name]['lens_xyz']);yaw=views[name]['lens_yaw']
        # Convert optical rays to FLU, then apply independently specified manual pose.
        rays=np.stack([np.ones_like(u),-(u-k[2])/k[0],-(v-k[5])/k[4]],axis=-1).astype(float)
        rotation=np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1]])
        rays=rays@rotation.T
        with np.errstate(divide='ignore',invalid='ignore'):
            lo=(np.array([1.96,-.45,0])-origin)/rays;hi=(np.array([2.04,.45,2.2])-origin)/rays
        enter=np.minimum(lo,hi).max(axis=-1);leave=np.maximum(lo,hi).min(axis=-1)
        hit=(leave>=enter)&(enter>0)
        with np.errstate(invalid='ignore'):residual=np.abs(depth-enter)
        depth_board=largest(hit&(residual<.03))
        # Fixed blue-board material; largest component excludes the person's jeans.
        rgb_board=largest((rgb[:,:,0]>25)&(rgb[:,:,2]>1.6*rgb[:,:,0])&(rgb[:,:,1]>1.3*rgb[:,:,0]))
        edge_rgb=rgb_board^binary_erosion(rgb_board);edge_depth=depth_board^binary_erosion(depth_board)
        distances=np.r_[distance_transform_edt(~edge_rgb)[edge_depth],distance_transform_edt(~edge_depth)[edge_rgb]]
        result[name]=dict(board_mask_iou=float((rgb_board&depth_board).sum()/(rgb_board|depth_board).sum()),
            board_boundary_symmetric_p95_pixels=float(np.percentile(distances,95)),
            board_boundary_max_pixels=float(distances.max()),
            board_depth_vs_analytic_aabb_p95_m=float(np.percentile(residual[binary_erosion(depth_board,iterations=2)],95)),
            rgb_board_pixels=int(rgb_board.sum()),depth_board_pixels=int(depth_board.sum()))
    report=dict(scope='Independent appearance and depth alignment check using known calibration-board geometry only; not person recognition or flight evidence.',
        method='Largest fixed-blue RGB component versus depth rays intersecting known AABB within 3 cm; symmetric boundary distance. Pixel centers, antialiasing and mixed silhouette pixels limit exact equality.',views=result)
    (a.run/'image-alignment.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,2,figsize=(8,6.6),dpi=160)
    titles=['Fully visible','Partly occluded','Fully occluded (initial)','Visible after lateral move']
    for ax,name,title in zip(axes.flat,views.keys(),titles):
        ax.imshow(Image.open(a.run/name/'rgb.png'),interpolation='nearest');ax.set_title(title);ax.axis('off')
    fig.suptitle('Actual ROS2 RGB frames · Gazebo Harmonic · 320 × 240',fontsize=12)
    fig.tight_layout();fig.savefig(a.run/'rgb-views.png');plt.close(fig)

if __name__=='__main__':main()
