"""Detect four arena walls without confusing their unknown exterior with free space.

Only Program 1 uses this module. The fitted rectangle describes inner wall
faces; it is not an obstacle map or a claim that its interior is obstacle-free.
"""
from collections import deque
from dataclasses import dataclass
import math

import numpy as np

from .model import Arena


@dataclass(frozen=True)
class ArenaDetection:
    arena: Arena
    side_coverage: tuple[float, float, float, float]
    inlier_fraction: float
    residual_m: float
    score: float


def _component(mask, seeds=None):
    """Flood a supplied seed set, or return the largest 4-connected component."""
    height,width=mask.shape
    visited=np.zeros_like(mask,dtype=bool)
    best=[]
    if seeds is None:
        starts=zip(*np.nonzero(mask))
    else:
        starts=seeds
    for row,col in starts:
        if visited[row,col] or not mask[row,col]:continue
        cells=[];todo=deque([(int(row),int(col))]);visited[row,col]=True
        while todo:
            y,x=todo.popleft();cells.append((y,x))
            for ny,nx in ((y-1,x),(y+1,x),(y,x-1),(y,x+1)):
                if 0<=ny<height and 0<=nx<width and mask[ny,nx] and not visited[ny,nx]:
                    visited[ny,nx]=True;todo.append((ny,nx))
        if seeds is None and len(cells)>len(best):best=cells
    if seeds is not None:return visited
    result=np.zeros_like(mask,dtype=bool)
    if best:
        y,x=zip(*best);result[y,x]=True
    return result


def _dilate(mask,steps):
    for _ in range(steps):
        p=np.pad(mask,1)
        mask=np.logical_or.reduce([p[y:y+mask.shape[0],x:x+mask.shape[1]] for y in range(3) for x in range(3)])
    return mask


def _fit(points,free_points,resolution,angle):
    c,s=math.cos(angle),math.sin(angle)
    u=points[:,0]*c+points[:,1]*s
    v=-points[:,0]*s+points[:,1]*c
    low_u,high_u=np.quantile(u,(.015,.985))
    low_v,high_v=np.quantile(v,(.015,.985))
    # Estimate each face independently; internal clutter is not allowed to
    # shift the rectangle merely because it contributes many occupied cells.
    band=max(1.8*resolution,.035)
    for _ in range(2):
        mid_u=(u>low_u+.12*(high_u-low_u))&(u<high_u-.12*(high_u-low_u))
        mid_v=(v>low_v+.12*(high_v-low_v))&(v<high_v-.12*(high_v-low_v))
        masks=((abs(u-low_u)<band)&mid_v,(abs(u-high_u)<band)&mid_v,
               (abs(v-low_v)<band)&mid_u,(abs(v-high_v)<band)&mid_u)
        if any(np.count_nonzero(mask)<3 for mask in masks):return None
        low_u,high_u,low_v,high_v=[float(np.median(values[mask])) for values,mask in zip((u,u,v,v),masks)]
    width,height=high_u-low_u,high_v-low_v
    if min(width,height)<max(.3,6*resolution) or max(width,height)>10:return None
    distances=np.stack((abs(u-low_u),abs(u-high_u),abs(v-low_v),abs(v-high_v)))
    nearest=np.argmin(distances,axis=0)
    error=np.min(distances,axis=0)
    tolerance=max(.8*resolution,.012)
    coverage=[]
    for side in range(4):
        mask=(nearest==side)&(error<=tolerance)
        coordinates=(v[mask]-low_v)/height if side<2 else (u[mask]-low_u)/width
        coordinates=coordinates[(coordinates>=-.02)&(coordinates<=1.02)]
        # Spatial coverage distinguishes a complete wall from a short obstacle.
        bins=np.clip((coordinates*10).astype(int),0,9)
        coverage.append(len(np.unique(bins))/10)
        if len(coordinates)<4:return None
    fu=free_points[:,0]*c+free_points[:,1]*s
    fv=-free_points[:,0]*s+free_points[:,1]*c
    inside=np.mean((fu>=low_u-resolution)&(fu<=high_u+resolution)&
                   (fv>=low_v-resolution)&(fv<=high_v+resolution))
    fraction=float(np.mean(error<=tolerance))
    residual=float(np.mean(np.minimum(error,3*resolution)))
    score=2*min(coverage)+float(np.mean(coverage))+fraction-residual/max(resolution,.001)
    return dict(angle=angle,low_u=low_u,low_v=low_v,width=width,height=height,
                coverage=coverage,inside=float(inside),fraction=fraction,residual=residual,score=score)


def detect_arena(slam):
    """Fit a rotated rectangle supported by four walls and unknown exterior.

    Refuses incomplete evidence instead of silently labeling a known-cell box
    as detected walls. Raster quantization and noisy SLAM still require visual
    inspection; ``score`` is a fit measure, not a statistical probability.
    """
    states=slam.states
    if states.size>250_000:
        raise ValueError('แผนที่ใหญ่เกินการตรวจขอบอัตโนมัติ ใช้กรอบ 3 จุดหรือแมพสนามที่เล็กลง')
    height,width=states.shape
    if min(height,width)<8:raise ValueError('แมพมีเซลล์น้อยเกินไปสำหรับตรวจผนังสี่ด้าน')
    seeds=[(0,x) for x in range(width)]+[(height-1,x) for x in range(width)]
    seeds += [(y,0) for y in range(height)]+[(y,width-1) for y in range(height)]
    outside=_component(states==2,seeds)
    if np.count_nonzero(outside)<max(8,.01*states.size):
        raise ValueError('ไม่พบพื้นที่ unknown รอบนอกพอให้แยกขอบสนาม')
    interior=_component(states==0)
    if np.count_nonzero(interior)<40:raise ValueError('พื้นที่ว่างที่สำรวจยังน้อยเกินไป')
    # Retain only walls near the exterior, then measure their interface with
    # the interior free component. Deep interior obstacles are discarded.
    near_outside=_dilate(outside,max(2,math.ceil(.20/slam.resolution)))
    walls=(states==1)&near_outside
    rows,cols=np.nonzero(interior)
    faces=[]
    for dy,dx in ((-1,0),(1,0),(0,-1),(0,1)):
        yy,xx=rows+dy,cols+dx
        valid=(yy>=0)&(yy<height)&(xx>=0)&(xx<width)
        keep=np.flatnonzero(valid)
        keep=keep[walls[yy[keep],xx[keep]]]
        if len(keep):
            faces.append(np.column_stack((cols[keep]+.5+.5*dx,height-rows[keep]-.5-.5*dy))*slam.resolution)
    if not faces:raise ValueError('ไม่พบผนังระหว่างพื้นที่ในสนามกับ unknown รอบนอก')
    points=np.concatenate(faces)
    if len(points)<24:raise ValueError('เห็นผนังสนามไม่ครบพอสำหรับตรวจสี่เหลี่ยม')
    if len(points)>4000:points=points[np.linspace(0,len(points)-1,4000).astype(int)]
    free_points=np.column_stack((cols+.5,height-rows-.5))*slam.resolution
    if len(free_points)>4000:free_points=free_points[np.linspace(0,len(free_points)-1,4000).astype(int)]
    options=[]
    for degrees in np.arange(-45,45,.5):
        fit=_fit(points,free_points,slam.resolution,math.radians(float(degrees)))
        if fit is not None:options.append(fit)
    if not options:raise ValueError('ไม่พบแนวผนังสี่ด้านที่รองรับสี่เหลี่ยม')
    best=max(options,key=lambda item:item['score'])
    for degrees in np.arange(math.degrees(best['angle'])-.5,math.degrees(best['angle'])+.501,.05):
        fit=_fit(points,free_points,slam.resolution,math.radians(float(degrees)))
        if fit is not None and fit['score']>best['score']:best=fit
    if min(best['coverage'])<.6 or best['fraction']<.65 or best['inside']<.96 or best['residual']>1.25*slam.resolution:
        raise ValueError('หลักฐานผนังสี่ด้านไม่ชัด อาจมีช่องเปิดหรือสำรวจไม่ครบ ใช้กรอบ 3 จุดช่วยกำหนด')
    angle=best['angle'];c,s=math.cos(angle),math.sin(angle)
    ix=c*best['low_u']-s*best['low_v'];iy=s*best['low_u']+c*best['low_v']
    x,y=slam._from_image_axes(ix,iy)
    arena=Arena(best['width'],best['height'],x,y,slam.origin[2]+angle)
    return ArenaDetection(arena,tuple(best['coverage']),best['fraction'],best['residual'],best['score'])
