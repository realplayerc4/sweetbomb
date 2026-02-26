import os
import json
import logging
from typing import List, Optional
from threading import Lock

from app.models.waypoint import Waypoint, WaypointList

logger = logging.getLogger(__name__)

class WaypointManager:
    """
    Singleton manager for robotic waypoints.
    Handles persistence to rest-api/data/waypoints.json
    """
    _instance: Optional["WaypointManager"] = None
    _lock = Lock()

    def __new__(cls) -> "WaypointManager":
        with cls._lock:
            if cls._instance is None:
                cls._instance = super().__new__(cls)
                cls._instance._initialized = False
        return cls._instance

    def __init__(self, data_path: str = "data/waypoints.json"):
        if self._initialized:
            return
            
        # Ensure absolute path relative to rest-api root
        root_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        self.data_file = os.path.join(root_dir, data_path)
        self._waypoints: List[Waypoint] = []
        self._load()
        self._initialized = True
        logger.info(f"WaypointManager initialized with {len(self._waypoints)} waypoints. File: {self.data_file}")

    def _load(self):
        """Load waypoints from JSON file."""
        if not os.path.exists(self.data_file):
            logger.info(f"Waypoint file not found at {self.data_file}, starting with empty list.")
            self._waypoints = []
            return

        try:
            with open(self.data_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
                wp_list = WaypointList.model_validate(data)
                self._waypoints = wp_list.waypoints
        except Exception as e:
            logger.error(f"Failed to load waypoints: {e}")
            self._waypoints = []

    def _save(self):
        """Save waypoints to JSON file."""
        try:
            os.makedirs(os.path.dirname(self.data_file), exist_ok=True)
            wp_list = WaypointList(waypoints=self._waypoints)
            with open(self.data_file, 'w', encoding='utf-8') as f:
                json.dump(wp_list.model_dump(mode="json"), f, indent=2, ensure_ascii=False)
        except Exception as e:
            logger.error(f"Failed to save waypoints: {e}")

    def get_all(self) -> List[Waypoint]:
        """Get all waypoints."""
        return self._waypoints

    def get_by_name(self, name: str) -> Optional[Waypoint]:
        """Get a waypoint by name."""
        for wp in self._waypoints:
            if wp.name == name:
                return wp
        return None

    def add_waypoint(self, waypoint: Waypoint) -> bool:
        """Add or update a waypoint."""
        for i, wp in enumerate(self._waypoints):
            if wp.name == waypoint.name:
                self._waypoints[i] = waypoint
                self._save()
                return True
        
        self._waypoints.append(waypoint)
        self._save()
        return True

    def remove_waypoint(self, name: str) -> bool:
        """Remove a waypoint by name."""
        original_len = len(self._waypoints)
        self._waypoints = [wp for wp in self._waypoints if wp.name != name]
        if len(self._waypoints) < original_len:
            self._save()
            return True
        return False

    @classmethod
    def get_instance(cls) -> "WaypointManager":
        """Get the singleton instance."""
        return cls()
