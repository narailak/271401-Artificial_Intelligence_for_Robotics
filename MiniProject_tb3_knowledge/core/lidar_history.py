"""Short-lived free-space evidence from live, odometry-aligned laser rays.

This is runtime sensor memory, never a map prior or exported knowledge.  Like
the navigator's beam-spacing filter, free fans assume continuity between two
adjacent valid laser returns.  Missing returns are never assigned max range.
The caller transforms both endpoints and sensor origins into the current base
frame, using recorded odometry without extrapolation.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Ray:
    origin: tuple[float, float]
    endpoint: tuple[float, float]
    stamp: float


@dataclass(frozen=True)
class Sweep:
    # Preserve the ORIGINAL angular indexes: None marks an invalid return.
    rays: tuple[Ray | None, ...]
    angle_increment: float


_EPS=1e-12
_AREA_EPS=1e-13


def _cross(a,b,p):
    return (b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0])


def _area(polygon):
    return sum(a[0]*b[1]-a[1]*b[0] for a,b in zip(polygon,polygon[1:]+polygon[:1]))/2


def _clean(polygon):
    result=[]
    for point in polygon:
        if not result or math.dist(point,result[-1])>_EPS:result.append(point)
    if len(result)>1 and math.dist(result[0],result[-1])<=_EPS:result.pop()
    return result


def _clip(polygon,a,b,inside=True):
    """Clip a convex polygon to one closed half-plane of directed edge a,b."""
    if not polygon:return []
    result=[]
    sign=1 if inside else -1
    previous=polygon[-1];before=sign*_cross(a,b,previous)
    for current in polygon:
        after=sign*_cross(a,b,current)
        previous_in,current_in=before>=-_EPS,after>=-_EPS
        if previous_in!=current_in:
            denominator=before-after
            if abs(denominator)>1e-30:
                fraction=before/denominator
                result.append((previous[0]+fraction*(current[0]-previous[0]),
                               previous[1]+fraction*(current[1]-previous[1])))
        if current_in:result.append(current)
        previous,before=current,after
    result=_clean(result)
    return result if len(result)>=3 and abs(_area(result))>_AREA_EPS else []


def _subtract(polygon,fan):
    """Disjoint convex pieces of polygon outside a convex CCW fan."""
    remaining=polygon
    outside=[]
    for a,b in zip(fan,fan[1:]+fan[:1]):
        piece=_clip(remaining,a,b,False)
        if piece:outside.append(piece)
        remaining=_clip(remaining,a,b,True)
        if not remaining:break
    return outside


def _bounds(polygon):
    xs,ys=zip(*polygon)
    return min(xs),min(ys),max(xs),max(ys)


def _overlap(a,b):
    return not (a[2]<b[0]-_EPS or b[2]<a[0]-_EPS or a[3]<b[1]-_EPS or b[3]<a[1]-_EPS)


def _covered(target,fans):
    """Exact union coverage by repeated convex subtraction, with bounded work."""
    pieces=[target];used=set()
    target_bounds=_bounds(target)
    for index,(fan,bounds,stamp,rays) in enumerate(fans):
        if not _overlap(target_bounds,bounds):continue
        next_pieces=[]
        for piece in pieces:
            if not _overlap(_bounds(piece),bounds):
                next_pieces.append(piece);continue
            remainder=_subtract(piece,fan)
            old_area=abs(_area(piece))
            new_area=sum(abs(_area(p)) for p in remainder)
            if old_area-new_area>_AREA_EPS:used.add(index)
            next_pieces.extend(remainder)
            if len(next_pieces)>512:return False,set()
        pieces=next_pieces
        if not pieces:return True,used
    return False,set()


def _finite_point(point):
    return len(point)==2 and all(math.isfinite(value) for value in point)


def _valid_ray(ray,now,max_age):
    return (ray is not None and _finite_point(ray.origin) and _finite_point(ray.endpoint)
            and math.isfinite(ray.stamp) and 0<=now-ray.stamp<=max_age
            and math.dist(ray.origin,ray.endpoint)>.01)


def _fan(a,b,increment):
    # No arbitrary convex hull: a folded/misaligned fan must remain unknown.
    if math.dist(a.origin,b.origin)>.005:return None
    va=(a.endpoint[0]-a.origin[0],a.endpoint[1]-a.origin[1])
    vb=(b.endpoint[0]-b.origin[0],b.endpoint[1]-b.origin[1])
    angle=math.atan2(math.sin(math.atan2(vb[1],vb[0])-math.atan2(va[1],va[0])),
                     math.cos(math.atan2(vb[1],vb[0])-math.atan2(va[1],va[0])))
    if not 0<angle*(1 if increment>0 else -1)<=2*abs(increment):return None
    ra,rb=math.hypot(*va),math.hypot(*vb)
    extent=min(ra,rb)-.01
    if extent<=0:return None
    ea=(a.origin[0]+va[0]*extent/ra,a.origin[1]+va[1]*extent/ra)
    eb=(b.origin[0]+vb[0]*extent/rb,b.origin[1]+vb[1]*extent/rb)
    polygon=_clean([a.origin,b.origin,eb,ea])
    if len(polygon)<3:return None
    area=_area(polygon)
    if abs(area)<=_AREA_EPS:return None
    if area<0:polygon.reverse()
    if any(_cross(polygon[i-1],polygon[i],polygon[(i+1)%len(polygon)]) < -_EPS
           for i in range(len(polygon))):return None
    return polygon


def fill_missing(current_points,missing_sectors,older_sweeps,now,max_age,max_range,*,minimum_range=.16):
    """Return (points, unresolved_angles, oldest_used_stamp).

    missing_sectors contains (current_sensor_origin_xy, angle, angular_width).
    A virtual return is added only after the ENTIRE angular sector from
    minimum_range to that finite range is covered by fresh historical fans.
    The sensor's blind inner range is never asserted free by this operation.
    All current returns survive. Close endpoints supporting accepted old fans
    also survive, so temporal evidence never averages away a close obstacle.

    The caller must use min(current_stamp, oldest_used_stamp) for navigation
    freshness and braking latency. max_age remains bounded by its sensor timeout.
    The function examines only the three most recent preceding sweeps.
    """
    if not all(math.isfinite(v) for v in (now,max_age,max_range,minimum_range)):
        raise ValueError('nonfinite_lidar_history_limits')
    if max_age<=0 or not 0<minimum_range<=max_range:
        raise ValueError('invalid_lidar_history_limits')
    result=list(current_points)
    if not missing_sectors:return result,[],None
    fans=[]
    for sweep in reversed(list(older_sweeps)[-3:]):
        increment=sweep.angle_increment
        if not math.isfinite(increment) or not 0<abs(increment)<=.2:continue
        rays=sweep.rays
        pairs=list(zip(rays,rays[1:]))
        if len(rays)>=16 and abs(len(rays)*abs(increment)-2*math.pi)<=max(.05,1.5*abs(increment)):
            pairs.append((rays[-1],rays[0]))
        for a,b in pairs:
            if not _valid_ray(a,now,max_age) or not _valid_ray(b,now,max_age):continue
            polygon=_fan(a,b,increment)
            if polygon:fans.append((polygon,_bounds(polygon),min(a.stamp,b.stamp),(a,b)))
    unresolved=[];used_fans=set()
    for origin,angle,width in missing_sectors:
        if not _finite_point(origin) or not math.isfinite(angle) or not math.isfinite(width) or not 0<width<=.2:
            raise ValueError('invalid_missing_lidar_sector')
        half=width/2
        def target(extent):
            # Circumscribed outer chord: an inscribed triangle would omit the
            # circular arc and could certify more range than the evidence.
            outer=extent/math.cos(half)
            # The inner chord lies inside the minimum-range arc, so this
            # quadrilateral contains the whole annular sector being proved.
            # Requiring evidence all the way to the current origin would
            # reject even tiny odometry/mount changes for no sensor benefit.
            points=[(origin[0]+minimum_range*math.cos(angle-half),origin[1]+minimum_range*math.sin(angle-half)),
                    (origin[0]+outer*math.cos(angle-half),origin[1]+outer*math.sin(angle-half)),
                    (origin[0]+outer*math.cos(angle+half),origin[1]+outer*math.sin(angle+half)),
                    (origin[0]+minimum_range*math.cos(angle+half),origin[1]+minimum_range*math.sin(angle+half))]
            return points
        # Angular half-plane separation removes distant fans even when every
        # fan shares the same sensor origin (so their AABBs all touch). Do this
        # once, before bisection; retaining extra fans is conservative.
        full_target=target(min(max_range,.8))
        relevant=[]
        for index,entry in enumerate(fans):
            polygon=entry[0]
            if any(max(_cross(a,b,vertex) for vertex in polygon)<=_EPS
                   for a,b in zip(full_target,full_target[1:]+full_target[:1])):continue
            relevant.append((index,entry))
        sector_fans=[entry for index,entry in relevant]
        covered,used=_covered(target(minimum_range),sector_fans)
        if not covered:
            unresolved.append(angle);continue
        low,high=minimum_range,min(max_range,.8)
        if high<low:
            unresolved.append(angle);continue
        for _ in range(10):
            middle=(low+high)/2
            accepted,witnesses=_covered(target(middle),sector_fans)
            if accepted:low,used=middle,witnesses
            else:high=middle
        result.append((origin[0]+low*math.cos(angle),origin[1]+low*math.sin(angle)))
        used_fans.update(relevant[index][0] for index in used)
    oldest=None;endpoints=set()
    for index in used_fans:
        fan,bounds,stamp,rays=fans[index]
        oldest=stamp if oldest is None else min(oldest,stamp)
        for ray in rays:
            if math.hypot(*ray.endpoint)<=max_range:endpoints.add(ray.endpoint)
    result.extend(sorted(endpoints))
    return result,unresolved,oldest
