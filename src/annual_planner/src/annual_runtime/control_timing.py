"""Opt-in, local buffered timing trace. It never changes control decisions."""
from contextlib import suppress
from pathlib import Path
import json
import logging
import time
import os
from .control_observation import ObservationLog

logger = logging.getLogger(__name__)


class ControlTiming:
    def __init__(self, directory, drone, role):
        self.file = None
        self.observation = (ObservationLog(directory, drone, role, compress=os.environ.get("RACER_CONTROL_OBSERVATION_GZIP")=="1") if directory and os.environ.get("RACER_CONTROL_OBSERVATION")=="1" else None)
        if directory:
            path = Path(directory) / f'drone-{drone}-{role}.jsonl'
            try:
                path.parent.mkdir(parents=True, exist_ok=True)
                self.file = path.open('x', buffering=65536)
            except OSError as error:
                logger.warning('Unable to open timing trace: %s', error)
            self.event('trace_start', clock='host_monotonic_ns', role=role, drone=drone)

    def event(self, kind, **fields):
        if self.file is not None:
            try:
                self.file.write(json.dumps(dict(event=kind, wall_ns=time.monotonic_ns(), **fields), separators=(',', ':'))+'\n')
            except OSError as error:
                logger.warning('Disabling timing trace after a write failure: %s', error)
                with suppress(OSError):
                    self._close_file()

    def publish(self, publisher, message, kind, stamp_ns):
        if self.file is None:
            publisher.publish(message)
            return
        start = time.monotonic_ns()
        publisher.publish(message)
        self.event(kind, start_wall_ns=start, duration_ns=time.monotonic_ns()-start, stamp_ns=int(stamp_ns))

    def close(self):
        if self.observation is not None: self.observation.close()
        self.event('trace_end')
        try:
            self._close_file()
        except OSError as error:
            logger.warning('Unable to close timing trace: %s', error)

    def _close_file(self):
        file = self.file
        self.file = None
        if file is not None:
            file.close()
