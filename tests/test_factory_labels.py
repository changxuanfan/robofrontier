import unittest
from robofrontier.factory_labels import place_label, overlaps, region_title


class LabelPlacementTests(unittest.TestCase):
    def test_title_stays_centered_and_requires_furniture_to_move(self):
        position,half=region_title(dict(bounds_xy_m=[-11.5,-20.5,11.5,0]))
        self.assertEqual(position[:2],[0,-.6])
        with self.assertRaises(ValueError):place_label([position],half,[[-.5,-1,.5,-.2]])

    def test_caption_is_beyond_table_and_clearance(self):
        position,box=place_label([[0,.18,0],[0,-.45,0]],[1.4,.13],[[-2,0,2,3]])
        self.assertEqual(position,[0,-.45,0])
        self.assertGreater(0-box[3],.1)

    def test_blocked_layout_is_reported(self):
        with self.assertRaises(ValueError):place_label([[0,0,0]],[1,.2],[[-2,-2,2,2]])
