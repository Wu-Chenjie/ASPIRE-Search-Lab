"""Conservative RGB-D candidates. Geometry alone never confirms a person."""
from dataclasses import dataclass, field
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.ndimage import binary_erosion
from aspire_camera.point_cloud import cloud, validate_transform


@dataclass
class Candidate:
    xyz: np.ndarray
    pixels: np.ndarray
    box: tuple
    semantic: bool
    confidence: float
    depth_m: float
    spread_m: float
    quality: dict = field(default_factory=dict)


def person_depth_pixels(depth):
    return (depth > .25) & (depth < 8.)


class PeopleDetector:
    """Fixed OpenCV Dalal-Triggs HOG; no model download or language model."""
    def __init__(self):
        import cv2
        self.cv2 = cv2
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    def __call__(self, rgb):
        im = self.cv2.resize(rgb[:,:,::-1], None, fx=2, fy=2)
        boxes, weights = self.hog.detectMultiScale(im, winStride=(4,4), padding=(8,8), scale=1.05)
        return [(np.asarray(b, float)/2, float(w)) for b,w in zip(boxes, weights) if w >= 0.7]


def candidates(rgb, depth, k, camera_to_map, detector):
    return candidates_from_cloud(rgb, depth, cloud(depth, k, camera_to_map), detector)


def candidates_from_cloud(rgb, depth, points, detector):
    """Use pixel-aligned map points from this capture, retaining the mask indices.

    Candidate locations come from connected, eroded visible surfaces.

    Assumes a gravity-aligned metric map with floor z=0. This is an explicit
    indoor fixture assumption, not floor estimation for unknown buildings.
    Broad planar boards and horizontal floor surfaces cannot seed person tracks.
    """
    valid = person_depth_pixels(depth) & (points[:,:,2] > .25) & (points[:,:,2] < 2.25)
    h,w = depth.shape
    ids = np.arange(h*w).reshape(h,w)
    links=[]
    for a,b,pa,pb,va,vb in [(ids[:,:-1],ids[:,1:],points[:,:-1],points[:,1:],valid[:,:-1],valid[:,1:]), (ids[:-1],ids[1:],points[:-1],points[1:],valid[:-1],valid[1:])]:
        with np.errstate(invalid='ignore'):
            ok=va & vb & (np.linalg.norm(pa-pb,axis=-1)<.10)
        links.append((a[ok],b[ok]))
    aa=np.concatenate([x[0] for x in links]);bb=np.concatenate([x[1] for x in links])
    _, labels=connected_components(coo_matrix((np.ones(len(aa)),(aa,bb)),shape=(h*w,h*w)),directed=False)
    labels=labels.reshape(h,w)
    counts=np.bincount(labels[valid]); boxes=detector(rgb); result=[]
    for lab in np.flatnonzero(counts>=20):
        mask=(labels==lab)&valid; pts=points[mask]
        extent=np.ptp(pts,axis=0); horizontal=float(np.linalg.norm(extent[:2]))
        if not .7 <= extent[2] <= 2.05 or not .045 <= horizontal <= .85:
            continue
        # Wide flat surfaces are furniture/board evidence, not person candidates.
        eig=np.linalg.eigvalsh(np.cov(pts.T))
        if eig[0]<1e-5:continue
        interior=binary_erosion(mask)
        if interior.sum()<12:continue  # Thin mixed silhouette pixels are unsafe.
        v,u=np.where(interior); visible=points[interior]
        xyz=np.median(visible,axis=0); d=float(np.median(depth[interior]))
        semantic=False; confidence=0.
        if extent[2]>.9:
            for (x,y,bw,bh),score in boxes:
                inside=(u>=x)&(u<x+bw)&(v>=y)&(v<y+bh)
                if inside.mean()>.7:
                    semantic=True;confidence=max(confidence,score)
        result.append(Candidate(xyz,interior,(int(u.min()),int(v.min()),int(u.max()+1),int(v.max()+1)),semantic,confidence,d,float(np.median(np.linalg.norm(visible-xyz,axis=1)))))
    return result
