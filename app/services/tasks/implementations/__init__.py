"""Task implementations package."""

from app.services.tasks.implementations.object_detection_task import ObjectDetectionTask
from app.services.tasks.implementations.data_collection_task import DataCollectionTask
from app.services.tasks.implementations.point_cloud_analysis_task import PointCloudAnalysisTask
from app.services.tasks.implementations.navigation_task import NavigationTask, ReturnToOriginTask
from app.services.tasks.implementations.sequential_task_queue import SequentialTaskQueue
from app.services.tasks.implementations.sugar_harvest_task import SugarHarvestTask

__all__ = [
    "ObjectDetectionTask",
    "DataCollectionTask",
    "PointCloudAnalysisTask",
    "NavigationTask",
    "ReturnToOriginTask",
    "SequentialTaskQueue",
    "SugarHarvestTask",
]
