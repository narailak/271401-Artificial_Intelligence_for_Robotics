"""Bounded interpolation of small NaN dropouts in a single live sweep."""
import math


def bridge_short_dropouts(ranges, increment, minimum, maximum):
    """Return filtered ranges, filled indexes, and conservative beam spacing.

    shortcut: assumes a continuous surface across at most two missing beams
    spanning five degrees; use a second sensor for thinner hidden obstacles.
    Original valid/Inf/out-of-range readings are never overwritten. The closer
    flank minus 2 cm supplies the virtual return; callers retain the raw scan
    completeness and inflate obstacles by the full bridged angular interval.
    """
    result=list(ranges);filled=set();spacing=abs(increment)
    if not math.isfinite(spacing) or spacing<=0:return tuple(result),filled,spacing
    count=len(result)
    # Do not bridge the scan seam: its temporal/origin gap needs history evidence.
    index=1
    while index<count-1:
        if not math.isnan(ranges[index]):index+=1;continue
        start=index
        while index<count and math.isnan(ranges[index]):index+=1
        missing=index-start
        if index>=count or missing>2:continue
        left,right=ranges[start-1],ranges[index]
        width=(missing+1)*spacing
        if (width>math.radians(5) or not all(math.isfinite(v) and max(.25,minimum)<=v<=maximum for v in (left,right))
                or abs(left-right)>.05):continue
        value=min(left,right)-.02
        for position in range(start,index):result[position]=value;filled.add(position)
    if filled:
        # Recompute from original runs; spacing must not compound across holes.
        largest=1;run=0
        for index in range(count):
            run=run+1 if index in filled else 0
            largest=max(largest,run+1)
        spacing*=largest
    return tuple(result),filled,spacing
