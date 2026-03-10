"""
距离分析器单元测试

本模块测试距离分析器的核心功能，包括：
- 点云数据过滤
- 距离计算（最近点距离）
- 糖堆高度计算
- 推模式触发检测
- 配置管理
"""

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
    """创建用于测试的距离分析器实例"""
    return DistanceAnalyzer(
        bucket_width_m=0.6,
        height_threshold_m=0.20,
        min_points=10,
    )


@pytest.fixture
def sample_point_cloud():
    """创建用于测试的样本点云"""
    np.random.seed(42)
    # 创建 100 个具有已知分布的点
    n_points = 100

    # X: 0.5m 到 2.0m（深度）
    x = np.random.uniform(0.5, 2.0, n_points)
    # Y: -0.3m 到 0.3m（在铲斗宽度内）
    y = np.random.uniform(-0.25, 0.25, n_points)
    # Z: 0.0m 到 0.5m（高度）
    z = np.random.uniform(0.0, 0.5, n_points)

    return np.column_stack([x, y, z])


@pytest.fixture
def low_sugar_point_cloud():
    """创建低糖高度（< 20cm）的点云"""
    np.random.seed(43)
    n_points = 50

    x = np.random.uniform(0.5, 1.5, n_points)
    y = np.random.uniform(-0.25, 0.25, n_points)
    # 低高度：仅 10cm
    z = np.random.uniform(0.0, 0.10, n_points)

    return np.column_stack([x, y, z])


class TestDistanceAnalyzer:
    """距离分析器测试套件"""

    def test_initial_config(self, analyzer):
        """测试初始配置"""
        assert analyzer.bucket_width_m == 0.6
        assert analyzer.bucket_half_width == 0.3
        assert analyzer.height_threshold_m == 0.20
        assert analyzer.min_points == 10

    def test_analyze_valid_point_cloud(self, analyzer, sample_point_cloud):
        """测试分析有效点云"""
        result = analyzer.analyze(sample_point_cloud)

        assert result is not None
        assert isinstance(result, DistanceAnalysisResult)
        assert result.distance_m > 0
        assert result.sugar_height_m > 0
        assert result.point_count >= 10
        assert. len(result.deepest_point) == 3

    def test_analyze_empty_point_cloud(self, analyzer):
        """测试分析空点云"""
        empty_cloud = np.array([])
        result = analyzer.analyze(empty_cloud)
        assert result is None

    def test_analyze_point_cloud_outside_bucket_width(self, analyzer):
        """测试分析点在铲斗宽度外的点云"""
        # 所有点都在铲斗宽度外
        x = np.array([1.0, 1.5, 2.0])
        y = np.array([0.5, 0.6, 0.7])  # 在铲斗宽度外
        z = np.array([0.1, 0.2, 0.3])
        point_cloud = np.column_stack([x, y, z])

        result = analyzer.analyze(point_cloud)
        # 应该返回 None，因为没有点在铲斗宽度内
        assert result is None

    def test_deepest_point_calculation(self, analyzer):
        """测试正确识别最深点（最小 X 值）"""
        # 创建具有已知最深点的点
        x = np.array([1.5, 1.0, 1.8])  # 1.0 是最小值（最深）
        y = np.array([0.0, 0.1, -0.1])
        z = np.array([0.2, 0.3, 0.1])
        point_cloud = np.column_stack([x, y, z])

        result = analyzer.analyze(point_cloud)
        assert result is not None
        # 最深点应该有 X = 1.0
        assert abs(result.deepest_point[0] - 1.0) < 0.01
        assert result.distance_m == result.deepest_point[0]

    def test_sugar_height_calculation(self, analyzer):
        """测试糖堆高度计算"""
        # 创建具有已知高度范围的点
        x = np.array([1.0, 1.2, 1.5])
        y = np.array([0.0, 0.1, -0.1])
        z = np.array([0.0, 0.5, 0.25])  # 高度范围：0.5 - 0.0 = 0.5m
        point_cloud = np.column_stack([x, y, z])

        result = analyzer.analyze(point_cloud)
        assert result is not None
        assert abs(result.sugar_height_m - 0.5) < 0.01

    def test_should_switch_to_push_mode(self, analyzer, low_sugar_point_cloud):
        """测试低糖高度触发推模式检测"""
        result = analyzer.analyze(low_sugar_point_cloud)
        assert result is not None
        assert result.should_switch_to_push is True
        assert result.sugar_height_m < 0.20

    def test_should_not_switch_to_push_mode(self, analyzer, sample_point_cloud):
        """测试正常糖高度不触发推模式"""
        result = analyzer.analyze(sample_point_cloud)
        assert result is not None
        assert result.should_switch_to_push is False

    def test_set_bucket_width(self, analyzer):
        """测试设置铲斗宽度"""
        analyzer.set_bucket_width(0.8)
        assert analyzer.bucket_width_m == 0.8
        assert analyzer.bucket_half_width == 0.4

    def test_set_height_threshold(self, analyzer):
        """测试设置高度阈值"""
        analyzer.set_height_threshold(0.15)
        assert analyzer.height_threshold_m == 0.15

    def test_analyze_with_nan_values(self, analyzer):
        """测试处理点云中的 NaN 值"""
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
        # 应该只分析有效点
        assert result is not None
        assert result.point_count == 2

    def test_analyze_with_inf_values(self, analyzer):
        """测试处理点云中的 Inf 值"""
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
        # 应该只分析有效点
        assert result is not None
        assert result.point_count == 2

    def test_insufficient_points_after_filtering(self, analyzer):
        """测试过滤后点数不足的处理"""
        # 仅 5 个点在铲斗宽度内
        x = np.array([1.0, 1.1, 1.2, 1.3, 1.4])
        y = np.array([0.0, 0.05, 0.1, -0.05, -0.1])
        z = np.array([0.1, 0.15, 0.2, 0.25, 0.3])
        point_cloud = np.column_stack([x, y, z])

        result = analyzer.analyze(point_cloud)
        # 应该返回 None，因为 min_points 是 10
        assert result is None


class TestGlobalAnalyzer:
    """全局分析器函数测试套件"""

    def test_get_distance_analyzer(self):
        """测试获取全局分析器实例"""
        analyzer1 = get_distance_analyzer()
        analyzer2 = get_distance_analyzer()
        # 应该返回相同配置（不一定是相同实例）
        assert analyzer1.bucket_width_m == analyzer2.bucket_width_m

    def test_update_analyzer_config(self):
        """测试更新全局分析器配置"""
        update_analyzer_config(bucket_width_m=0.8)
        analyzer = get_distance_analyzer()
        assert analyzer.bucket_width_m == 0.8

        # 为其他测试重置
        update_analyzer_config(bucket_width_m=0.6)
