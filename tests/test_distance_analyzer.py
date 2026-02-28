"""Unit tests for the distance analyzer."""

import pytest
import numpy as np
from app.services.distance_analyzer import (
    DistanceAnalyzer,
    DistanceAnalysisResult,
    get_distance_analyzer,
    update_analyzer_config,
)


@pytest.fixture
def analyzer():
    """Create a distance analyzer instance for testing."""
    return DistanceAnalyzer(
        bucket_width_m=0.6,
        height_threshold_m=0.20,
        min_points=10,
    )


@pytest.fixture
def sample_point_cloud():
    """Create a sample point cloud for testing."""
    np.random.seed(42)
    # Create 100 points with known distribution
    n_points = 100

    # X: 0.5m to 2.0m (depth)
    x = np.random.uniform(0.5, 2.0, n_points)
    # Y: -0.3m to 0.3m (within bucket width)
    y = np.random.uniform(-0.25, 0.25, n_points)
    # Z: 0.0m to 0.5m (height)
    z = np.random.uniform(0.0, 0.5, n_points)

    return np.column_stack([x, y, z])


@pytest.fixture
def low_sugar_point_cloud():
    """Create a point cloud with low sugar height (< 20cm)."""
    np.random.seed(43)
    n_points = 50

    x = np.random.uniform(0.5, 1.5, n_points)
    y = np.random.uniform(-0.25, 0.25, n_points)
    # Low height: only 10cm
    z = np.random.uniform(0.0, 0.10, n_points)

    return np.column_stack([x, y, z])


class TestDistanceAnalyzer:
    """Test suite for DistanceAnalyzer."""

    def test_initial_config(self, analyzer):
        """Test initial configuration."""
        assert analyzer.bucket_width_m == 0.6
        assert analyzer.bucket_half_width == 0.3
        assert analyzer.height_threshold_m == 0.20
        assert analyzer.min_points == 10

    def test_analyze_valid_point_cloud(self, analyzer, sample_point_cloud):
        """Test analyzing valid point cloud."""
        result = analyzer.analyze(sample_point_cloud)

        assert result is not None
        assert isinstance(result, DistanceAnalysisResult)
        assert result.distance_m > 0
        assert result.sugar_height_m > 0
        assert result.point_count >= 10
        assert len(result.deepest_point) == 3

    def test_analyze_empty_point_cloud(self, analyzer):
        """Test analyzing empty point cloud."""
        empty_cloud = np.array([])
        result = analyzer.analyze(empty_cloud)
        assert result is None

    def test_analyze_point_cloud_outside_bucket_width(self, analyzer):
        """Test analyzing point cloud with points outside bucket width."""
        # All points outside bucket width
        x = np.array([1.0, 1.5, 2.0])
        y = np.array([0.5, 0.6, 0.7])  # Outside bucket width
        z = np.array([0.1, 0.2, 0.3])
        point_cloud = np.column_stack([x, y, z])

        result = analyzer.analyze(point_cloud)
        # Should return None as no points are within bucket width
        assert result is None

    def test_deepest_point_calculation(self, analyzer):
        """Test that the deepest point (minimum X) is correctly identified."""
        # Create points with known deepest point
        x = np.array([1.5, 1.0, 1.8])  # 1.0 is the minimum (deepest)
        y = np.array([0.0, 0.1, -0.1])
        z = np.array([0.2, 0.3, 0.1])
        point_cloud = np.column_stack([x, y, z])

        result = analyzer.analyze(point_cloud)
        assert result is not None
        # Deepest point should have X = 1.0
        assert abs(result.deepest_point[0] - 1.0) < 0.01
        assert result.distance_m == result.deepest_point[0]

    def test_sugar_height_calculation(self, analyzer):
        """Test sugar height calculation."""
        # Create points with known height range
        x = np.array([1.0, 1.2, 1.5])
        y = np.array([0.0, 0.1, -0.1])
        z = np.array([0.0, 0.5, 0.25])  # Height range: 0.5 - 0.0 = 0.5m
        point_cloud = np.column_stack([x, y, z])

        result = analyzer.analyze(point_cloud)
        assert result is not None
        assert abs(result.sugar_height_m - 0.5) < 0.01

    def test_should_switch_to_push_mode(self, analyzer, low_sugar_point_cloud):
        """Test detection of low sugar height triggering push mode."""
        result = analyzer.analyze(low_sugar_point_cloud)
        assert result is not None
        assert result.should_switch_to_push is True
        assert result.sugar_height_m < 0.20

    def test_should_not_switch_to_push_mode(self, analyzer, sample_point_cloud):
        """Test that normal sugar height does not trigger push mode."""
        result = analyzer.analyze(sample_point_cloud)
        assert result is not None
        assert result.should_switch_to_push is False

    def test_set_bucket_width(self, analyzer):
        """Test setting bucket width."""
        analyzer.set_bucket_width(0.8)
        assert analyzer.bucket_width_m == 0.8
        assert analyzer.bucket_half_width == 0.4

    def test_set_height_threshold(self, analyzer):
        """Test setting height threshold."""
        analyzer.set_height_threshold(0.15)
        assert analyzer.height_threshold_m == 0.15

    def test_analyze_with_nan_values(self, analyzer):
        """Test handling of NaN values in point cloud."""
        valid_points = np.array([
            [1.0, 0.0, 0.1],
            [1.2, 0.1, 0.2],
        ])
        nan_points = np.array([
            [np.nan, 0.0, 0.1],
            [1.2, np.nan, 0.2],
        ])
        combined = np.vstack([valid_points, nan_points])

        result = analyzer.analyze(combined)
        # Should analyze only valid points
        assert result is not None
        assert result.point_count == 2

    def test_analyze_with_inf_values(self, analyzer):
        """Test handling of Inf values in point cloud."""
        valid_points = np.array([
            [1.0, 0.0, 0.1],
            [1.2, 0.1, 0.2],
        ])
        inf_points = np.array([
            [np.inf, 0.0, 0.1],
            [1.2, 0.1, -np.inf],
        ])
        combined = np.vstack([valid_points, inf_points])

        result = analyzer.analyze(combined)
        # Should analyze only valid points
        assert result is not None
        assert result.point_count == 2

    def test_insufficient_points_after_filtering(self, analyzer):
        """Test handling of insufficient points after filtering."""
        # Only 5 points within bucket width
        x = np.array([1.0, 1.1, 1.2, 1.3, 1.4])
        y = np.array([0.0, 0.05, 0.1, -0.05, -0.1])
        z = np.array([0.1, 0.15, 0.2, 0.25, 0.3])
        point_cloud = np.column_stack([x, y, z])

        result = analyzer.analyze(point_cloud)
        # Should return None as min_points is 10
        assert result is None


class TestGlobalAnalyzer:
    """Test suite for global analyzer functions."""

    def test_get_distance_analyzer(self):
        """Test getting global analyzer instance."""
        analyzer1 = get_distance_analyzer()
        analyzer2 = get_distance_analyzer()
        # Should return the same config (not necessarily same instance)
        assert analyzer1.bucket_width_m == analyzer2.bucket_width_m

    def test_update_analyzer_config(self):
        """Test updating global analyzer config."""
        update_analyzer_config(bucket_width_m=0.8)
        analyzer = get_distance_analyzer()
        assert analyzer.bucket_width_m == 0.8

        # Reset for other tests
        update_analyzer_config(bucket_width_m=0.6)
