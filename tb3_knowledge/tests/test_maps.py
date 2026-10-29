"""Real raster semantics, coordinate transforms and swept footprint geometry."""
from pathlib import Path
import json
import math
import tempfile
import unittest

import numpy as np

from tb3_knowledge.core.maps import FREE, OCCUPIED, UNKNOWN, MissionWorld, SlamMap
from tb3_knowledge.core.model import Arena, Pose


class MapTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.folder = Path(self.temporary.name)

    def tearDown(self):
        self.temporary.cleanup()

    def raster(self, pixels, *, maximum=255, binary=False, ending=b"\n", **metadata):
        pixels = np.asarray(pixels)
        height, width = pixels.shape
        image = self.folder / "arena.pgm"
        if binary:
            body = pixels.astype(np.uint8 if maximum < 256 else ">u2").tobytes()
            image.write_bytes(f"P5\n# test map\n{width} {height}\n{maximum}".encode() + ending + body)
        else:
            image.write_text(f"P2\n# test map\n{width} {height}\n{maximum}\n" +
                             " ".join(str(int(x)) for x in pixels.flat) + "\n")
        data = dict(image="arena.pgm", mode="trinary", resolution=.1,
                    origin=[0, 0, 0], negate=0, free_thresh=.196, occupied_thresh=.65)
        data.update(metadata)
        source = self.folder / "arena.yaml"
        source.write_text(json.dumps(data))
        return source

    def test_p2_comments_and_standard_unknown(self):
        slam = SlamMap.load(self.raster([[0, 205, 254, 255]]))
        np.testing.assert_array_equal(slam.states, [[OCCUPIED, UNKNOWN, FREE, FREE]])
        self.assertEqual((slam.width, slam.height), (4, 1))
        self.assertFalse(slam.states.flags.writeable)

    def test_p5_binary_whitespace_and_hash_are_samples(self):
        pixels = [[10, 13, 32, 35, 205, 255], [255, 205, 35, 32, 13, 10]]
        expected = SlamMap.load(self.raster(pixels, free_thresh=.7, occupied_thresh=.9)).states.copy()
        actual = SlamMap.load(self.raster(pixels, binary=True, free_thresh=.7, occupied_thresh=.9))
        np.testing.assert_array_equal(actual.states, expected)

    def test_p5_16bit_is_big_endian(self):
        slam = SlamMap.load(self.raster([[0, 32768, 65535]], maximum=65535, binary=True))
        np.testing.assert_array_equal(slam.states, [[OCCUPIED, UNKNOWN, FREE]])

    def test_p5_crlf_header_preserves_first_binary_newline(self):
        slam = SlamMap.load(self.raster([[10, 205, 255]], binary=True, ending=b"\r\n"))
        np.testing.assert_array_equal(slam.states, [[OCCUPIED, UNKNOWN, FREE]])

    def test_negate_and_explicit_image_override(self):
        source = self.raster([[0, 205, 255]], negate=1, image="missing.pgm")
        with self.assertRaises(FileNotFoundError):
            SlamMap.load(source)
        slam = SlamMap.load(source, self.folder / "arena.pgm")
        np.testing.assert_array_equal(slam.states, [[FREE, OCCUPIED, OCCUPIED]])

    def test_invalid_metadata_is_rejected(self):
        for values in (dict(mode="scale"), dict(mode="raw"), dict(resolution=0),
                       dict(resolution=float("nan")), dict(origin=[0, 0]),
                       dict(origin=[0, float("inf"), 0]), dict(negate=2),
                       dict(free_thresh=.7, occupied_thresh=.6), dict(negate=True)):
            with self.subTest(values=values):
                with self.assertRaises(ValueError):
                    SlamMap.load(self.raster([[255]], **values))

    def test_corrupt_raster_is_rejected(self):
        source = self.raster([[255, 255]], binary=True)
        for payload in (b"P5\n2 1\n255\n\xff", b"P2\n2 1\n255\n0 256",
                        b"P3\n1 1\n255\n0 0 0", b"P2\n-1 1\n255\n0"):
            with self.subTest(payload=payload):
                (self.folder / "arena.pgm").write_bytes(payload)
                with self.assertRaises(ValueError):
                    SlamMap.load(source)

    def test_image_centers_and_yaw_roundtrip(self):
        slam = SlamMap(np.zeros((6, 8), dtype=np.uint8), .1, (1, 2, math.pi / 2))
        # Bottom-left outer image corner is exactly the YAML origin.
        np.testing.assert_allclose(slam.pixel_to_map(-.5, 5.5), (1, 2))
        np.testing.assert_allclose(slam.pixel_to_map(0, 0), (.45, 2.05))
        for pixel in ((0, 0), (7, 5), (2.3, 1.7), (-.5, 5.5)):
            np.testing.assert_allclose(slam.map_to_pixel(*slam.pixel_to_map(*pixel)), pixel, atol=1e-12)

    def test_known_bounds_include_walls_and_preserve_rotation(self):
        states = np.full((10, 12), UNKNOWN, dtype=np.uint8)
        states[2:8, 3:10] = FREE
        states[2, 3] = OCCUPIED
        slam = SlamMap(states, .1, (1, 2, .3))
        arena = slam.known_arena()
        self.assertAlmostEqual(arena.width, .7)
        self.assertAlmostEqual(arena.height, .6)
        self.assertEqual(arena.yaw, .3)
        np.testing.assert_allclose(arena.local_to_map(0, 0), slam.pixel_to_map(2.5, 7.5))
        with self.assertRaises(ValueError):
            SlamMap(np.full((10, 10), UNKNOWN), .1, (0, 0, 0)).known_arena()

    def test_circle_rejects_unknown_and_cell_corner_contact(self):
        states = np.zeros((20, 20), dtype=np.uint8)
        states[9, 10] = UNKNOWN  # Rectangle x=1..1.1, y=1..1.1.
        slam = SlamMap(states, .1, (0, 0, 0))
        self.assertTrue(slam.free(.8, .8, .2))
        self.assertFalse(slam.free(.9, .9, math.sqrt(.02)))
        self.assertFalse(slam.free(.9, 1.05, .1))
        self.assertTrue(slam.free(.89, 1.05, .1))
        self.assertFalse(slam.free(1.05, 1.05))
        self.assertFalse(slam.free(.1, .5, .1))
        self.assertFalse(slam.free(float("nan"), .5))

    def test_capsule_detects_obstacle_between_clear_endpoints(self):
        states = np.zeros((20, 20), dtype=np.uint8)
        states[9, 10] = OCCUPIED
        slam = SlamMap(states, .1, (0, 0, 0))
        self.assertTrue(slam.free(.5, 1.05, .05))
        self.assertTrue(slam.free(1.5, 1.05, .05))
        self.assertFalse(slam.segment_free((.5, 1.05), (1.5, 1.05), .05))
        self.assertFalse(slam.segment_free((.5, .9), (1.5, .9), .1))
        self.assertTrue(slam.segment_free((.5, .89), (1.5, .89), .1))
        self.assertTrue(slam.segment_free((.5, .5), (.5, .5), .1))

    def test_exact_rays_detect_single_cell_and_unknown(self):
        states = np.zeros((20, 20), dtype=np.uint8)
        states[9, 12] = OCCUPIED  # Front wall begins at x=1.2.
        states[9, 7] = UNKNOWN   # Rear unknown ends at x=.8.
        slam = SlamMap(states, .1, (0, 0, 0))
        ranges = slam.scan(Pose(1.05, 1.05, 0), count=4, max_range=3.5)
        np.testing.assert_allclose(ranges, (.25, 1.05, .15, .95), atol=1e-12)
        self.assertEqual(slam.scan(Pose(.75, 1.05), 4), (0.,) * 4)
        self.assertTrue(all(math.isfinite(x) for x in ranges))

    def test_rotated_scan_matches_unrotated_geometry(self):
        states = np.zeros((20, 20), dtype=np.uint8)
        states[9, 12] = OCCUPIED
        flat = SlamMap(states, .1, (0, 0, 0))
        tilted = SlamMap(states, .1, (1, -2, .7))
        x, y = tilted._from_image_axes(1.05, 1.05)
        np.testing.assert_allclose(tilted.scan(Pose(x, y, .7), 120), flat.scan(Pose(1.05, 1.05), 120), atol=1e-11)

    def test_rotated_arena_queries_map_in_correct_frame(self):
        states = np.zeros((50, 50), dtype=np.uint8)
        slam = SlamMap(states, .1, (-1, -1, .2))
        arena = Arena(1.5, 1.5, .2, .4, -.5)
        world = MissionWorld(slam, arena)
        local = Pose(.6, .7, .3)
        mapped = arena.pose_to_map(local)
        self.assertTrue(world.free(local.x, local.y, .1))
        self.assertFalse(world.free(.05, .5, .1))
        np.testing.assert_allclose(world.scan(local, 16), slam.scan(mapped, 16))
        self.assertTrue(world.reachable(Pose(.3, .3), (1.2, 1.2), .1))

    def test_connectivity_does_not_cut_diagonal_obstacle(self):
        states = np.zeros((20, 20), dtype=np.uint8)
        states[:, 10] = OCCUPIED
        world = MissionWorld(SlamMap(states, .1, (0, 0, 0)), Arena(2, 2))
        self.assertFalse(world.reachable(Pose(.4, .8), (1.6, .8), .12))
        states[7:13, 10] = FREE
        open_world = MissionWorld(SlamMap(states, .1, (0, 0, 0)), Arena(2, 2))
        self.assertTrue(open_world.reachable(Pose(.4, .4), (1.6, 1.6), .12))

    def test_actual_arena_read_only_when_available(self):
        source = Path.home() / "tb3_maps" / "arena.yaml"
        if not source.exists():
            self.skipTest("Actual user map is not present in this environment")
        before = source.read_bytes()
        # Read the image independently before the loader; a user may resurvey
        # this arena, so do not freeze external map dimensions in the test.
        import yaml
        metadata = yaml.safe_load(before)
        raster_path = Path(metadata["image"]).expanduser()
        if not raster_path.is_absolute():
            raster_path = source.parent / raster_path
        raster = raster_path.read_bytes()
        slam = SlamMap.load(source)
        self.assertGreater(slam.width, 0)
        self.assertGreater(slam.height, 0)
        self.assertGreater(slam.resolution, 0)
        arena = slam.known_arena()
        self.assertLessEqual(arena.width, slam.width * slam.resolution)
        self.assertLessEqual(arena.height, slam.height * slam.resolution)
        self.assertEqual(source.read_bytes(), before)
        self.assertEqual(slam.image_path.read_bytes(), raster)


if __name__ == "__main__":
    unittest.main()
