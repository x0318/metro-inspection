import json
import os
import threading
from typing import Dict


class CommandGateway:
    """Serialize validated web commands to the isolated ROS worker."""

    def __init__(self, command_fd: int) -> None:
        self._stream = os.fdopen(command_fd, 'w', encoding='utf-8', buffering=1)
        self._lock = threading.Lock()
        self._closed = False

    def send(self, command: Dict[str, object]) -> None:
        with self._lock:
            if self._closed:
                raise RuntimeError('ROS command channel is closed')
            try:
                self._stream.write(json.dumps(command, separators=(',', ':')) + '\n')
            except (BrokenPipeError, OSError) as error:
                self._closed = True
                raise RuntimeError('ROS command worker is unavailable') from error

    def close(self) -> None:
        with self._lock:
            if not self._closed:
                self._closed = True
                self._stream.close()
