"""Stationary, setup-only global scan matching for small imported SLAM maps.

Search the known free space (optionally near a user-selected position), refine distinct position/heading modes, then
validate predicted ranges. A symmetric/poor scene returns ambiguity, not an
arbitrarily selected Start. Nothing in this module is exported as knowledge.
"""
from dataclasses import dataclass
import heapq
import math

import numpy as np

from .model import Pose,wrap


@dataclass(frozen=True)
class ScanMatch:
    pose: Pose
    reason: str
    agreement: float
    separation: float
    trimmed_error: float | None = None
    endpoint_score: float | None = None


def _distance(mask, resolution, maximum=.4):
    distance=np.full(mask.shape,maximum,dtype=float)
    rows,cols=np.nonzero(mask)
    queue=[(0.,int(row),int(col)) for row,col in zip(rows,cols)]
    distance[rows,cols]=0.
    heapq.heapify(queue)
    height,width=mask.shape
    steps=[(dy,dx,resolution*math.hypot(dx,dy)) for dy in (-1,0,1) for dx in (-1,0,1) if dx or dy]
    while queue:
        length,row,col=heapq.heappop(queue)
        if length>distance[row,col]+1e-12:continue
        for dy,dx,cost in steps:
            y,x=row+dy,col+dx
            if 0<=y<height and 0<=x<width and length+cost<distance[y,x]:
                distance[y,x]=length+cost;heapq.heappush(queue,(length+cost,y,x))
    return distance


def match_scan(slam,ranges,angle_min,angle_increment,mount,*,radius=.124,minimum=.16,maximum=8.,
               position_hint=None,hint_radius=.35,cancel=None):
    hint=None
    if position_hint is not None:
        if (len(position_hint)!=2 or not all(math.isfinite(v) for v in position_hint)
                or not math.isfinite(hint_radius) or not .10<=hint_radius<=2.):
            raise ValueError('invalid_localization_hint')
        hint=slam._to_image_axes(*position_hint)
    if slam.states.size>250_000:raise ValueError('automatic_localization_map_too_large')
    if np.count_nonzero(slam.states==1)<10:raise ValueError('no_localization_map_obstacles')
    if (not all(math.isfinite(v) for v in (angle_min,angle_increment,*mount)) or angle_increment==0
            or len(ranges)<30 or not 2*math.pi-.2<=len(ranges)*abs(angle_increment)<=2*math.pi+.2):
        raise ValueError('insufficient_localization_scan')
    raw=np.asarray(ranges,dtype=float)
    valid=np.flatnonzero(np.isfinite(raw)&(raw>=minimum)&(raw<maximum-.01))
    sectors={int(((angle_min+i*angle_increment)%(2*math.pi))/(math.pi/6)) for i in valid}
    if len(valid)<30 or len(sectors)<6:raise ValueError('insufficient_localization_scan')
    if len(valid)>120:valid=valid[np.linspace(0,len(valid)-1,120,dtype=int)]
    lengths=raw[valid];angles=angle_min+valid*angle_increment
    sensor_angles=angles+mount[2]
    px=mount[0]+lengths*np.cos(sensor_angles)
    py=mount[1]+lengths*np.sin(sensor_angles)
    resolution=slam.resolution
    occupied=_distance(slam.states==1,resolution)
    blocked=_distance(slam.states!=0,resolution)
    # Every candidate center keeps a footprint away from unknown/occupied cells.
    free=blocked>=radius+resolution*math.sqrt(2)/2
    stride=max(1,round(.10/resolution))
    rows,cols=np.nonzero(free[::stride,::stride]);rows*=stride;cols*=stride
    if not len(rows):raise ValueError('no_localization_free_space')
    positions=np.column_stack(((cols+.5)*resolution,(slam.height-rows-.5)*resolution))
    if hint is not None:
        positions=positions[np.sum((positions-np.asarray(hint))**2,axis=1)<=hint_radius**2+1e-12]
        if not len(positions):raise ValueError('no_localization_free_space_near_hint')
    if len(positions)>15_000:raise ValueError('automatic_localization_map_too_large')

    def check_cancel():
        if cancel is not None and cancel.is_set():raise ValueError('localization_search_cancelled')

    def scores(poses):
        # Poses are in image-aligned metres/yaw; interpolate distance to avoid
        # a half-cell quantization bias when refining sub-pixel positions.
        x,y,yaw=poses.T;c=np.cos(yaw)[:,None];s=np.sin(yaw)[:,None]
        u=(x[:,None]+c*px-s*py)/resolution-.5
        v=slam.height-.5-(y[:,None]+s*px+c*py)/resolution
        inside=(u>=0)&(u<slam.width-1)&(v>=0)&(v<slam.height-1)
        col=np.clip(np.floor(u).astype(int),0,slam.width-2)
        row=np.clip(np.floor(v).astype(int),0,slam.height-2)
        fx=u-col;fy=v-row
        distance=((1-fy)*((1-fx)*occupied[row,col]+fx*occupied[row,col+1])+
            fy*((1-fx)*occupied[row+1,col]+fx*occupied[row+1,col+1]))
        distance=np.where(inside,distance,.4)
        result=np.mean(np.exp(-.5*(distance/.08)**2),axis=1)
        cx=np.floor(x/resolution).astype(int);cy=slam.height-1-np.floor(y/resolution).astype(int)
        keep=(cx>=0)&(cx<slam.width)&(cy>=0)&(cy<slam.height)
        keep&=free[np.clip(cy,0,slam.height-1),np.clip(cx,0,slam.width-1)]
        if hint is not None:keep&=(x-hint[0])**2+(y-hint[1])**2<=hint_radius**2+1e-12
        return np.where(keep,result,-1.)

    coarse=[];coarse_scores=[]
    for yaw in np.arange(-math.pi,math.pi,math.pi/36):
        check_cancel()
        poses=np.column_stack((positions,np.full(len(positions),yaw)))
        coarse.append(poses);coarse_scores.append(scores(poses))
    coarse=np.concatenate(coarse);coarse_scores=np.concatenate(coarse_scores)
    modes=[]
    for index in np.argsort(coarse_scores)[::-1]:
        pose=coarse[index]
        if all(math.hypot(*(pose[:2]-old[:2]))>.25 or abs(wrap(pose[2]-old[2]))>.35 for old in modes):
            modes.append(pose)
            if len(modes)==16:break
    refined=[];endpoint_scores=[]
    delta=np.array(np.meshgrid(np.arange(-.1,.1001,.025),np.arange(-.1,.1001,.025),
        np.deg2rad(np.arange(-5,5.1,1.)))).reshape(3,-1).T
    for mode in modes:
        check_cancel()
        poses=mode+delta;values=scores(poses)
        candidate=poses[np.argmax(values)]
        if all(math.hypot(*(candidate[:2]-old[:2]))>.20 or abs(wrap(candidate[2]-old[2]))>.30 for old in refined):
            refined.append(candidate)
            endpoint_scores.append(float(np.max(values)))
    evaluated=[]
    for (x,y,yaw),endpoint_score in zip(refined,endpoint_scores):
        check_cancel()
        mx,my=slam._from_image_axes(x,y)
        pose=Pose(float(mx),float(my),float(wrap(yaw+slam.origin[2])))
        if not slam.free(pose.x,pose.y,radius):continue
        c,s=math.cos(pose.yaw),math.sin(pose.yaw)
        laser=Pose(pose.x+c*mount[0]-s*mount[1],pose.y+s*mount[0]+c*mount[1],pose.yaw+mount[2])
        predicted=np.asarray(slam.scan(laser,count=1440,max_range=maximum))
        indices=np.rint(((angles+math.pi)%(2*math.pi))*1440/(2*math.pi)).astype(int)%1440
        errors=np.abs(lengths-predicted[indices])
        agreement=float(np.mean(errors<=max(.10,2*resolution)))
        trimmed=float(np.mean(np.sort(errors)[:max(1,int(len(errors)*.9))]))
        quality=.7*agreement+.3*math.exp(-trimmed/.10)
        evaluated.append((quality,agreement,trimmed,pose,endpoint_score))
    if not evaluated:raise ValueError('no_localization_match')
    evaluated.sort(key=lambda row:row[0],reverse=True)
    best=evaluated[0];separation=best[0]-evaluated[1][0] if len(evaluated)>1 else 1.
    reason=('scan_map_mismatch' if best[1]<.90 or best[2]>.07 or best[4]<.65 else
        'localization_ambiguous' if separation<.04 else 'matched')
    return ScanMatch(best[3],reason,best[1],separation,best[2],best[4])
