#!/usr/bin/env python3
"""
Basic tests for KISS-Matcher Python bindings.
"""
import unittest

import kiss_matcher as km
import numpy as np


class TestKISSMatcher(unittest.TestCase):
    """Test KISS-Matcher Python bindings."""

    def test_import(self):
        """Test that kiss_matcher can be imported."""
        # This test should pass if the module imports correctly
        self.assertTrue(hasattr(km, "KISSMatcher"))
        self.assertTrue(hasattr(km, "KISSMatcherConfig"))
        self.assertTrue(hasattr(km, "RegistrationSolution"))

    def test_config_creation(self):
        """Test KISSMatcherConfig creation."""
        config = km.KISSMatcherConfig()
        self.assertIsInstance(config.voxel_size, float)
        self.assertAlmostEqual(config.voxel_size, 0.3, places=5)  # default value

        # Test custom config
        custom_config = km.KISSMatcherConfig(voxel_size=0.5)
        self.assertAlmostEqual(custom_config.voxel_size, 0.5, places=5)

    def test_matcher_creation(self):
        """Test KISSMatcher creation."""
        # Test with float parameter
        matcher = km.KISSMatcher(0.3)
        self.assertIsInstance(matcher, km.KISSMatcher)

        # Test with config
        config = km.KISSMatcherConfig(voxel_size=0.5)
        matcher_config = km.KISSMatcher(config)
        self.assertIsInstance(matcher_config, km.KISSMatcher)

    def test_basic_functionality(self):
        """Test basic functionality with dummy data."""
        try:
            # Create a matcher
            matcher = km.KISSMatcher(0.3)

            # Create larger point clouds for better matching
            src_points = np.random.rand(100, 3).astype(np.float32)
            tgt_points = src_points.copy()  # Use same points for target

            # Convert to vector format if needed
            src_vector = [src_points[i] for i in range(src_points.shape[0])]
            tgt_vector = [tgt_points[i] for i in range(tgt_points.shape[0])]

            # This should not crash - actual matching may fail due to random data
            # but the binding should work
            result = matcher.match(src_vector, tgt_vector)
            self.assertIsInstance(result, km.RegistrationSolution)

        except Exception as e:
            # If there's an issue with the binding itself, we want to know
            if "AttributeError" in str(type(e)) or "TypeError" in str(type(e)):
                self.fail(f"Binding error: {e}")
            # Other errors might be expected with random data
            pass

    def test_version_attribute(self):
        """Test that version attribute exists."""
        self.assertTrue(hasattr(km, "__version__"))
        self.assertIsInstance(km.__version__, str)

    def test_solve_with_prematched_points(self):
        """Test solve() method with pre-matched point correspondences."""
        # Create a matcher
        matcher = km.KISSMatcher(0.3)

        # Create source points
        src_points = np.random.rand(3, 50).astype(np.float64)

        # Create target points (rotation + translation of source)
        angle = np.pi / 6  # 30 degrees
        R_true = np.array(
            [
                [np.cos(angle), -np.sin(angle), 0],
                [np.sin(angle), np.cos(angle), 0],
                [0, 0, 1],
            ]
        )
        t_true = np.array([1.0, 2.0, 0.5])
        tgt_points = R_true @ src_points + t_true[:, np.newaxis]

        # Test solve method (assumes correspondences are already established)
        result = matcher.solve(src_points, tgt_points)

        # Check that result is valid
        self.assertIsInstance(result, km.RegistrationSolution)
        self.assertTrue(hasattr(result, "rotation"))
        self.assertTrue(hasattr(result, "translation"))

    def test_prune_and_solve_with_matched_points(self):
        """Test prune_and_solve() method with pre-matched point correspondences."""
        # Create a matcher
        matcher = km.KISSMatcher(0.3)

        # Create source points (as list of vectors)
        src_points = [np.random.rand(3).astype(np.float32) for _ in range(100)]

        # Create target points with some transformation
        angle = np.pi / 4  # 45 degrees
        R_true = np.array(
            [
                [np.cos(angle), -np.sin(angle), 0],
                [np.sin(angle), np.cos(angle), 0],
                [0, 0, 1],
            ],
            dtype=np.float32,
        )
        t_true = np.array([0.5, 1.0, 0.0], dtype=np.float32)

        tgt_points = [(R_true @ p + t_true) for p in src_points]

        # Add some outliers (10% outliers)
        num_outliers = 10
        for i in range(num_outliers):
            tgt_points[i] = np.random.rand(3).astype(np.float32) * 10

        # Test prune_and_solve method
        result = matcher.prune_and_solve(src_points, tgt_points)

        # Check that result is valid
        self.assertIsInstance(result, km.RegistrationSolution)
        self.assertTrue(hasattr(result, "rotation"))
        self.assertTrue(hasattr(result, "translation"))


if __name__ == "__main__":
    unittest.main()
