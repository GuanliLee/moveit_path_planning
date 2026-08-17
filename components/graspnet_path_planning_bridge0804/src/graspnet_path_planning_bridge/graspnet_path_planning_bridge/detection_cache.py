from threading import Lock
from typing import Dict, Optional

from graspnet_bridge_interfaces.msg import DetectionResult


class DetectionCache:
    def __init__(self) -> None:
        self._lock = Lock()
        self._last: Optional[DetectionResult] = None
        self._by_target: Dict[str, DetectionResult] = {}

    def update(self, msg: DetectionResult) -> None:
        target_name = msg.target_name.strip()
        with self._lock:
            self._last = msg
            if target_name:
                self._by_target[target_name] = msg

    def get(self, target_name: str) -> Optional[DetectionResult]:
        key = target_name.strip()
        with self._lock:
            if not key:
                return self._last
            return self._by_target.get(key)
