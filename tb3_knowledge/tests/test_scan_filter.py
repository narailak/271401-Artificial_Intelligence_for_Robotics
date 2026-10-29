import math
import unittest
from tb3_knowledge.core.scan_filter import bridge_short_dropouts


class ScanFilterTests(unittest.TestCase):
    def test_only_small_continuous_nan_gap_is_interpolated_and_spacing_inflated(self):
        raw=(1.,float('nan'),float('nan'),.98,.4)
        result,filled,spacing=bridge_short_dropouts(raw,math.radians(1.4),.16,3.5)
        self.assertEqual(filled,{1,2})
        self.assertEqual(result,(1.,.96,.96,.98,.4))
        self.assertAlmostEqual(spacing,math.radians(4.2))
        self.assertTrue(math.isnan(raw[1]))

    def test_wide_discontinuous_near_invalid_and_seam_returns_stay_unknown(self):
        for raw,increment in (
            ((1.,float('nan'),float('nan'),float('nan'),1.),1.),
            ((1.,float('nan'),1.),3.),
            ((1.,float('nan'),.8),1.),
            ((.2,float('nan'),.2),1.),
            ((1.,float('inf'),1.),1.),
            ((1.,0.,1.),1.),
            ((float('nan'),1.,1.),1.),
            ((1.,1.,float('nan')),1.)):
            result,filled,_=bridge_short_dropouts(raw,math.radians(increment),.16,3.5)
            self.assertFalse(filled)
            for before,after in zip(raw,result):
                self.assertTrue(math.isnan(after) if math.isnan(before) else before==after)

    def test_multiple_holes_do_not_compound_spacing_or_overwrite_near_hits(self):
        raw=(1.,float('nan'),1.,float('nan'),1.,.17)
        result,filled,spacing=bridge_short_dropouts(raw,-math.radians(1.4),.16,3.5)
        self.assertEqual(filled,{1,3})
        self.assertAlmostEqual(spacing,math.radians(2.8))
        self.assertEqual(result[-1],.17)
