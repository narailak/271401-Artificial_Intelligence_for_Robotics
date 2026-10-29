"""Conservative swept-circle checks against live unobserved laser sectors."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class UnknownSector:
    origin: tuple[float,float]
    angle: float
    half_width: float
    minimum: float

    def __post_init__(self):
        if (len(self.origin)!=2 or not all(math.isfinite(v) for v in (*self.origin,self.angle,self.half_width,self.minimum))
                or not 0<=self.half_width<math.pi/2 or self.minimum<0):
            raise ValueError('invalid_unknown_sector')


def sector_distance(point,sector):
    """Exact distance to the infinite annular sector, including its two edges."""
    dx,dy=point[0]-sector.origin[0],point[1]-sector.origin[1]
    distance=math.hypot(dx,dy)
    delta=abs((math.atan2(dy,dx)-sector.angle+math.pi)%(2*math.pi)-math.pi)
    delta=max(0.,delta-sector.half_width)
    radial=max(sector.minimum,distance*math.cos(delta))
    return math.sqrt(max(0.,distance*distance+radial*radial-2*distance*radial*math.cos(delta)))


def sector_path_clearance(start,end,radius,sector):
    """Safe prefix of a line segment swept by a disk, in metres.

    A convex quadrilateral contains the relevant annular sector: its inner
    chord is inside the minimum-range arc and its far chord is beyond every
    possible contact with the swept disk. Ray/capsule intersections with all
    four edges find the first contact without skipping a narrow unknown wedge.
    """
    length=math.dist(start,end)
    if sector_distance(start,sector)<=radius+1e-10:return 0.
    if length<=1e-12:return 0.
    direction=((end[0]-start[0])/length,(end[1]-start[1])/length)
    far=(max(math.dist(start,sector.origin),math.dist(end,sector.origin))+radius+.01)/math.cos(sector.half_width)
    far=max(far,sector.minimum+.01)
    left,right=sector.angle-sector.half_width,sector.angle+sector.half_width
    def at(distance,angle):return (sector.origin[0]+distance*math.cos(angle)-start[0],
                                  sector.origin[1]+distance*math.sin(angle)-start[1])
    polygon=[at(sector.minimum,left),at(far,left),at(far,right),at(sector.minimum,right)]
    crosses=[(b[0]-a[0])*(-a[1])-(b[1]-a[1])*(-a[0])
             for a,b in zip(polygon,polygon[1:]+polygon[:1])]
    if all(c>=0 for c in crosses) or all(c<=0 for c in crosses):return 0.
    result=length
    for a,b in zip(polygon,polygon[1:]+polygon[:1]):
        # Each endpoint contributes a disk; the edge contributes its rectangle.
        along=a[0]*direction[0]+a[1]*direction[1]
        across2=max(0.,a[0]*a[0]+a[1]*a[1]-along*along)
        if across2<=radius*radius:
            root=math.sqrt(max(0.,radius*radius-across2))
            if along+root>=0:result=min(result,max(0.,along-root))
        ex,ey=b[0]-a[0],b[1]-a[1];edge=math.hypot(ex,ey)
        if edge<1e-12:continue
        ex/=edge;ey/=edge
        lower,upper=0.,length
        for initial,slope,low,high in ((-a[0]*ex-a[1]*ey,direction[0]*ex+direction[1]*ey,0.,edge),
                (a[0]*ey-a[1]*ex,-direction[0]*ey+direction[1]*ex,-radius,radius)):
            if abs(slope)<1e-12:
                if not low<=initial<=high:upper=-1.;break
            else:
                t0,t1=(low-initial)/slope,(high-initial)/slope
                lower=max(lower,min(t0,t1));upper=min(upper,max(t0,t1))
        if lower<=upper:result=min(result,max(0.,lower))
    return result
