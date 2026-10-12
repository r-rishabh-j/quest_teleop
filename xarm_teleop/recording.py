"""Append one observation/action transition per server tick to HDF5."""
from __future__ import annotations

import json
from importlib.metadata import version
from pathlib import Path
import time
import uuid

import h5py
import numpy as np

from .backends import state_vector


class EpisodeRecorder:
    def __init__(self, directory):
        self.directory = Path(directory)
        self.file = None
        self.path = None
        self.frames = 0

    def start(self, description, task="", metadata=None):
        if self.file is not None:
            raise ValueError("An episode is already recording")
        description_json = json.dumps(description)
        metadata_json = json.dumps(metadata or {})
        self.directory.mkdir(parents=True, exist_ok=True)
        name = time.strftime("episode_%Y%m%d_%H%M%S_") + uuid.uuid4().hex[:8]
        self.path = self.directory / f"{name}.partial.h5"
        self.file = h5py.File(self.path, "x")
        self.file.attrs.update(schema_version=1, task=task, started_at=time.time(),
                               description=description_json, metadata=metadata_json,
                               versions=json.dumps({name: version(name) for name in ("numpy", "mujoco", "gymnasium")}),
                               state_fields="tcp_xyz_m, tcp_rotvec_rad, joint_1_to_7_rad, gripper_pulse",
                               action_fields="tcp_xyz_m, tcp_rotvec_rad, gripper_pulse",
                               transition_order="observation[t], action[t], next_observation[t]",
                               complete=False)
        self.names = list(description["arms"])
        self.frames = 0
        return str(self.path)

    def _append(self, key, value):
        value = np.asarray(value)
        if key not in self.file:
            options = {"compression": "lzf"} if value.ndim >= 3 else {}
            self.file.create_dataset(key, shape=(0,) + value.shape, maxshape=(None,) + value.shape,
                                     dtype=value.dtype, chunks=(1,) + value.shape, **options)
        dataset = self.file[key]
        if dataset.shape[1:] != value.shape:
            raise ValueError(f"Observation shape changed for {key}")
        dataset.resize(self.frames + 1, axis=0)
        dataset[self.frames] = value

    def append(self, before, after, requested, applied, active, cameras, timestamp, source_sequence):
        if self.file is None:
            return
        try:
            self._append_transition(before, after, requested, applied, active, cameras, timestamp, source_sequence)
        except Exception:
            # A partial row must not become a valid training transition.
            def trim(_name, item):
                if isinstance(item, h5py.Dataset):
                    item.resize(self.frames, axis=0)
            self.file.visititems(trim)
            self.file.attrs["write_failed"] = True
            raise

    def _append_transition(self, before, after, requested, applied, active, cameras, timestamp, source_sequence):
        self._append("timestamp", np.float64(timestamp))
        self._append("source_sequence", np.int64(source_sequence))
        for name in self.names:
            self._append(f"observation/{name}/state", state_vector(before[name]))
            self._append(f"next_observation/{name}/state", state_vector(after[name]))
            self._append(f"observation/{name}/timestamp", before[name]["timestamp"])
            self._append(f"action/{name}/requested", requested[name])
            self._append(f"action/{name}/applied", applied[name])
            self._append(f"action/{name}/active", np.array(active[name], dtype=bool))
            self._append(f"reward/{name}", after[name]["reward"])
            self._append(f"done/{name}", after[name]["done"])
            self._append(f"sim_time/{name}", before[name]["sim_time"])
        for name, pixels in cameras.items():
            self._append(f"observation/images/{name}", np.asarray(pixels, dtype=np.uint8))
        self.frames += 1
        self.file.attrs["frames"] = self.frames
        if self.frames % 50 == 0:
            self.file.flush()

    def stop(self, success=False, reason="operator"):
        if self.file is None:
            return None
        self.file.attrs.update(complete=True, success=bool(success), stop_reason=reason,
                               stopped_at=time.time(), frames=self.frames)
        self.file.flush()
        self.file.close()
        self.file = None
        final = self.path.with_name(self.path.name.replace(".partial.h5", ".h5"))
        self.path.rename(final)
        return str(final)
