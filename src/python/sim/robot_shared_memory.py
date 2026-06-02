"""Shared-memory robot I/O for the simulator."""

from __future__ import annotations

import hashlib
import inspect
import json
import struct
import time
from dataclasses import dataclass, replace
from multiprocessing import resource_tracker
from multiprocessing import shared_memory
from pathlib import Path
from typing import Any, Sequence


DEFAULT_CONFIG_PATH = (
    Path(__file__).resolve().parents[2]
    / "config"
    / "robot_shared_memory_config.json"
)
DEFAULT_SHM_NAME = "robot_mujoco_io"
COMMAND_MODE_TORQUE = 0

MAGIC = b"ROBOTIO\0"
VERSION = 2
DOUBLE_SIZE = 8
SUPPORTED_DTYPE = "float64"

COMMON_HEADER_STRUCT = struct.Struct("<8sIIQIIIIQ")
STATE_HEADER_STRUCT = struct.Struct("<QIIddd")
COMMAND_HEADER_STRUCT = struct.Struct("<QIId")

COMMON_HEADER_OFFSET = 0
STATE_HEADER_OFFSET = COMMON_HEADER_OFFSET + COMMON_HEADER_STRUCT.size
COMMAND_HEADER_OFFSET = STATE_HEADER_OFFSET + STATE_HEADER_STRUCT.size
ARRAYS_OFFSET = COMMAND_HEADER_OFFSET + COMMAND_HEADER_STRUCT.size

TRACK_PARAMETER_SUPPORTED = "track" in inspect.signature(
    shared_memory.SharedMemory
).parameters
OWNED_SHM_NAMES: set[str] = set()


@dataclass(frozen=True)
class ModelDimensions:
    nq: int
    nv: int
    nu: int
    nsensordata: int

    def as_dict(self) -> dict[str, int]:
        return {
            "nq": self.nq,
            "nv": self.nv,
            "nu": self.nu,
            "nsensordata": self.nsensordata,
        }


@dataclass(frozen=True)
class FieldSpec:
    name: str
    dtype: str
    size: int | str

    @classmethod
    def from_json(cls, raw: dict[str, Any]) -> "FieldSpec":
        name = raw.get("name")
        dtype = raw.get("dtype", SUPPORTED_DTYPE)
        size = raw.get("size")

        if not isinstance(name, str) or not name:
            raise ValueError("shared-memory field requires a non-empty string name")
        if dtype != SUPPORTED_DTYPE:
            raise ValueError(f"unsupported dtype for field {name!r}: {dtype!r}")
        if not isinstance(size, (int, str)):
            raise ValueError(f"field {name!r} size must be an integer or dimension key")

        return cls(name=name, dtype=dtype, size=size)

    def resolved_size(self, dimensions: ModelDimensions) -> int:
        if isinstance(self.size, int):
            size = self.size
        else:
            values = dimensions.as_dict()
            if self.size not in values:
                raise ValueError(
                    f"field {self.name!r} references unknown dimension {self.size!r}"
                )
            size = values[self.size]

        if size < 0:
            raise ValueError(f"field {self.name!r} size must be non-negative")
        return size


@dataclass(frozen=True)
class SharedMemoryConfig:
    shared_memory_name: str
    state_fields: tuple[FieldSpec, ...]
    command_fields: tuple[FieldSpec, ...]
    fingerprint: int

    @classmethod
    def load(cls, path: Path | None = None) -> "SharedMemoryConfig":
        config_path = DEFAULT_CONFIG_PATH if path is None else Path(path)
        raw = json.loads(config_path.read_text(encoding="utf-8"))

        shared_memory_name = raw.get("shared_memory_name", DEFAULT_SHM_NAME)
        if not isinstance(shared_memory_name, str) or not shared_memory_name:
            raise ValueError("shared_memory_name must be a non-empty string")

        state_fields = _load_field_specs(raw, "state_fields")
        command_fields = _load_field_specs(raw, "command_fields")
        _validate_unique_names(state_fields, "state_fields")
        _validate_unique_names(command_fields, "command_fields")

        normalized = {
            "shared_memory_name": shared_memory_name,
            "state_fields": [field.__dict__ for field in state_fields],
            "command_fields": [field.__dict__ for field in command_fields],
        }
        digest = hashlib.sha256(
            json.dumps(normalized, sort_keys=True, separators=(",", ":")).encode(
                "utf-8"
            )
        ).digest()
        fingerprint = int.from_bytes(digest[:8], "little")

        return cls(
            shared_memory_name=shared_memory_name,
            state_fields=tuple(state_fields),
            command_fields=tuple(command_fields),
            fingerprint=fingerprint,
        )


@dataclass(frozen=True)
class SharedMemoryHeader:
    total_size: int
    config_fingerprint: int
    nq: int
    nv: int
    nu: int
    nsensordata: int
    state_seq: int
    command_seq: int
    sim_alive: bool
    command_enabled: bool
    command_mode: int
    sim_time: float
    timestep: float
    state_wall_time: float
    command_wall_time: float


@dataclass(frozen=True)
class _CommonHeader:
    total_size: int
    config_fingerprint: int
    dimensions: ModelDimensions


@dataclass(frozen=True)
class _StateHeader:
    sequence: int
    sim_alive: bool
    sim_time: float
    timestep: float
    wall_time: float


@dataclass(frozen=True)
class _CommandHeader:
    sequence: int
    enabled: bool
    mode: int
    wall_time: float


@dataclass(frozen=True)
class FieldLayout:
    name: str
    offset: int
    size: int


@dataclass(frozen=True)
class SharedMemoryLayout:
    dimensions: ModelDimensions
    state_fields: dict[str, FieldLayout]
    command_fields: dict[str, FieldLayout]
    total_size: int

    @classmethod
    def from_config(
        cls,
        config: SharedMemoryConfig,
        dimensions: ModelDimensions,
    ) -> "SharedMemoryLayout":
        offset = ARRAYS_OFFSET
        state_fields: dict[str, FieldLayout] = {}
        command_fields: dict[str, FieldLayout] = {}

        for field in config.state_fields:
            size = field.resolved_size(dimensions)
            state_fields[field.name] = FieldLayout(field.name, offset, size)
            offset += size * DOUBLE_SIZE

        for field in config.command_fields:
            size = field.resolved_size(dimensions)
            command_fields[field.name] = FieldLayout(field.name, offset, size)
            offset += size * DOUBLE_SIZE

        return cls(
            dimensions=dimensions,
            state_fields=state_fields,
            command_fields=command_fields,
            total_size=offset,
        )


@dataclass(frozen=True)
class RobotState:
    sequence: int
    sim_alive: bool
    sim_time: float
    timestep: float
    wall_time: float
    fields: dict[str, list[float]]

    def __getattr__(self, name: str) -> list[float]:
        try:
            return self.fields[name]
        except KeyError as exc:
            raise AttributeError(name) from exc


@dataclass(frozen=True)
class RobotCommand:
    sequence: int
    enabled: bool
    mode: int
    wall_time: float
    fields: dict[str, list[float]]

    def __getattr__(self, name: str) -> list[float]:
        try:
            return self.fields[name]
        except KeyError as exc:
            raise AttributeError(name) from exc


def _load_field_specs(raw: dict[str, Any], key: str) -> tuple[FieldSpec, ...]:
    fields = raw.get(key)
    if not isinstance(fields, list):
        raise ValueError(f"{key} must be a list")
    return tuple(FieldSpec.from_json(field) for field in fields)


def _validate_unique_names(fields: Sequence[FieldSpec], label: str) -> None:
    names = [field.name for field in fields]
    duplicates = sorted({name for name in names if names.count(name) > 1})
    if duplicates:
        raise ValueError(f"{label} has duplicate field names: {', '.join(duplicates)}")


def _open_shared_memory(
    *,
    name: str,
    create: bool,
    size: int = 0,
    track: bool = True,
    unregister_attached: bool = True,
) -> shared_memory.SharedMemory:
    kwargs = {"name": name, "create": create, "size": size}
    if TRACK_PARAMETER_SUPPORTED:
        kwargs["track"] = track

    shm = shared_memory.SharedMemory(**kwargs)
    if create:
        OWNED_SHM_NAMES.add(shm._name)

    if (
        not create
        and unregister_attached
        and not TRACK_PARAMETER_SUPPORTED
        and shm._name not in OWNED_SHM_NAMES
    ):
        try:
            resource_tracker.unregister(shm._name, "shared_memory")
        except Exception:
            pass

    return shm


def _as_float_list(values: object, expected: int, label: str) -> list[float]:
    if hasattr(values, "tolist"):
        values = values.tolist()
    if isinstance(values, (int, float)):
        result = [float(values)]
    else:
        result = [float(value) for value in values]
    if len(result) != expected:
        raise ValueError(f"{label} must contain {expected} values, got {len(result)}")
    return result


def _next_odd_sequence(sequence: int) -> int:
    odd_sequence = sequence + 1
    if odd_sequence % 2 == 0:
        odd_sequence += 1
    return odd_sequence


class RobotSharedMemory:
    """Read robot state and write torque commands through shared memory."""

    def __init__(
        self,
        shm: shared_memory.SharedMemory,
        layout: SharedMemoryLayout,
        config: SharedMemoryConfig,
        *,
        owner: bool,
        unlink_on_close: bool,
    ) -> None:
        self._shm = shm
        self.layout = layout
        self.config = config
        self.owner = owner
        self.unlink_on_close = unlink_on_close

    @property
    def name(self) -> str:
        return self._shm.name

    @classmethod
    def create(
        cls,
        *,
        name: str | None = None,
        nq: int,
        nv: int,
        nu: int,
        nsensordata: int,
        timestep: float,
        config_path: Path | None = None,
        config: SharedMemoryConfig | None = None,
        unlink_existing: bool = True,
        unlink_on_close: bool = True,
    ) -> "RobotSharedMemory":
        config = SharedMemoryConfig.load(config_path) if config is None else config
        dimensions = ModelDimensions(
            nq=nq,
            nv=nv,
            nu=nu,
            nsensordata=nsensordata,
        )
        layout = SharedMemoryLayout.from_config(config, dimensions)
        shm_name = config.shared_memory_name if name is None else name

        try:
            shm = _open_shared_memory(
                name=shm_name,
                create=True,
                size=layout.total_size,
                track=True,
            )
        except FileExistsError:
            if not unlink_existing:
                raise
            old_shm = _open_shared_memory(
                name=shm_name,
                create=False,
                track=False,
                unregister_attached=False,
            )
            try:
                old_shm.unlink()
            finally:
                old_shm.close()
            shm = _open_shared_memory(
                name=shm_name,
                create=True,
                size=layout.total_size,
                track=True,
            )

        io = cls(
            shm,
            layout,
            config,
            owner=True,
            unlink_on_close=unlink_on_close,
        )
        now = time.time()
        io._write_common_header(
            _CommonHeader(
                total_size=layout.total_size,
                config_fingerprint=config.fingerprint,
                dimensions=dimensions,
            )
        )
        io._write_state_header(
            _StateHeader(
                sequence=0,
                sim_alive=True,
                sim_time=0.0,
                timestep=float(timestep),
                wall_time=now,
            )
        )
        io._write_command_header(
            _CommandHeader(
                sequence=0,
                enabled=False,
                mode=COMMAND_MODE_TORQUE,
                wall_time=now,
            )
        )
        io._zero_arrays()
        return io

    @classmethod
    def attach(
        cls,
        name: str | None = None,
        *,
        config_path: Path | None = None,
        config: SharedMemoryConfig | None = None,
    ) -> "RobotSharedMemory":
        config = SharedMemoryConfig.load(config_path) if config is None else config
        shm_name = config.shared_memory_name if name is None else name
        shm = _open_shared_memory(name=shm_name, create=False, track=False)
        try:
            header = cls._read_header_from(shm)
            if header.config_fingerprint != config.fingerprint:
                raise ValueError(
                    "shared-memory config fingerprint mismatch; "
                    "use the same JSON config for both processes"
                )
            dimensions = ModelDimensions(
                nq=header.nq,
                nv=header.nv,
                nu=header.nu,
                nsensordata=header.nsensordata,
            )
            layout = SharedMemoryLayout.from_config(config, dimensions)
            if layout.total_size != header.total_size:
                raise ValueError(
                    "shared-memory size mismatch: "
                    f"header={header.total_size}, computed={layout.total_size}"
                )
            if shm.size < layout.total_size:
                raise ValueError(
                    f"shared-memory block is too small: {shm.size} < "
                    f"{layout.total_size}"
                )
        except Exception:
            shm.close()
            raise
        return cls(shm, layout, config, owner=False, unlink_on_close=False)

    def read_header(self) -> SharedMemoryHeader:
        return self._read_header()

    def set_alive(self, alive: bool) -> None:
        state_header = self._read_state_header()
        self._write_state_header(replace(state_header, sim_alive=bool(alive)))

    def write_state(
        self,
        *,
        sim_time: float,
        timestep: float,
        **fields: object,
    ) -> None:
        field_values = self._validate_fields(
            fields,
            self.layout.state_fields,
            "state",
        )
        state_header = self._read_state_header()
        odd_seq = _next_odd_sequence(state_header.sequence)

        self._write_state_header(replace(state_header, sequence=odd_seq))
        for name, values in field_values.items():
            self._pack_doubles(self.layout.state_fields[name].offset, values)
        self._write_state_header(
            _StateHeader(
                sequence=odd_seq + 1,
                sim_alive=True,
                sim_time=float(sim_time),
                timestep=float(timestep),
                wall_time=time.time(),
            )
        )

    def read_state(self, timeout: float = 1.0) -> RobotState:
        deadline = time.monotonic() + timeout
        while True:
            state_before = self._read_state_header()
            if state_before.sequence % 2:
                if time.monotonic() >= deadline:
                    raise TimeoutError("Timed out waiting for a stable state write")
                time.sleep(0)
                continue

            fields = self._read_fields(self.layout.state_fields)

            state_after = self._read_state_header()
            if (
                state_before.sequence == state_after.sequence
                and state_after.sequence % 2 == 0
            ):
                return RobotState(
                    sequence=state_after.sequence,
                    sim_alive=state_after.sim_alive,
                    sim_time=state_after.sim_time,
                    timestep=state_after.timestep,
                    wall_time=state_after.wall_time,
                    fields=fields,
                )

            if time.monotonic() >= deadline:
                raise TimeoutError("Timed out waiting for a stable state read")

    def write_command(
        self,
        *,
        enabled: bool = True,
        mode: int = COMMAND_MODE_TORQUE,
        **fields: object,
    ) -> None:
        field_values = self._validate_fields(
            fields,
            self.layout.command_fields,
            "command",
        )
        command_header = self._read_command_header()
        odd_seq = _next_odd_sequence(command_header.sequence)

        self._write_command_header(replace(command_header, sequence=odd_seq))
        for name, values in field_values.items():
            self._pack_doubles(self.layout.command_fields[name].offset, values)
        self._write_command_header(
            _CommandHeader(
                sequence=odd_seq + 1,
                enabled=bool(enabled),
                mode=int(mode),
                wall_time=time.time(),
            )
        )

    def write_torque(
        self,
        torque: Sequence[float],
        *,
        enabled: bool = True,
        mode: int = COMMAND_MODE_TORQUE,
    ) -> None:
        if "torque" not in self.layout.command_fields:
            raise KeyError("command field 'torque' is not configured")
        fields = {
            name: [0.0] * layout.size
            for name, layout in self.layout.command_fields.items()
        }
        fields["torque"] = torque
        self.write_command(enabled=enabled, mode=mode, **fields)

    def disable_command(self) -> None:
        fields = {
            name: [0.0] * layout.size
            for name, layout in self.layout.command_fields.items()
        }
        self.write_command(enabled=False, **fields)

    def read_command(self, timeout: float = 1.0) -> RobotCommand:
        deadline = time.monotonic() + timeout
        while True:
            command_before = self._read_command_header()
            if command_before.sequence % 2:
                if time.monotonic() >= deadline:
                    raise TimeoutError("Timed out waiting for a stable command write")
                time.sleep(0)
                continue

            fields = self._read_fields(self.layout.command_fields)

            command_after = self._read_command_header()
            if (
                command_before.sequence == command_after.sequence
                and command_after.sequence % 2 == 0
            ):
                return RobotCommand(
                    sequence=command_after.sequence,
                    enabled=command_after.enabled,
                    mode=command_after.mode,
                    wall_time=command_after.wall_time,
                    fields=fields,
                )

            if time.monotonic() >= deadline:
                raise TimeoutError("Timed out waiting for a stable command read")

    def close(self) -> None:
        self._shm.close()

    def unlink(self) -> None:
        self._shm.unlink()
        OWNED_SHM_NAMES.discard(self._shm._name)

    def __enter__(self) -> "RobotSharedMemory":
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        if self.owner:
            try:
                self.set_alive(False)
            except Exception:
                pass
        if self.owner and self.unlink_on_close:
            try:
                self.unlink()
            except FileNotFoundError:
                pass
        self.close()

    @staticmethod
    def _read_common_header_from(shm: shared_memory.SharedMemory) -> _CommonHeader:
        values = COMMON_HEADER_STRUCT.unpack_from(shm.buf, COMMON_HEADER_OFFSET)
        magic = values[0]
        if magic != MAGIC:
            raise ValueError(f"Invalid shared-memory magic: {magic!r}")

        version = values[1]
        if version != VERSION:
            raise ValueError(
                f"Unsupported shared-memory version: {version}, expected {VERSION}"
            )

        arrays_offset = values[2]
        if arrays_offset != ARRAYS_OFFSET:
            raise ValueError(
                f"Unsupported shared-memory arrays offset: {arrays_offset}"
            )

        return _CommonHeader(
            total_size=values[3],
            dimensions=ModelDimensions(
                nq=values[4],
                nv=values[5],
                nu=values[6],
                nsensordata=values[7],
            ),
            config_fingerprint=values[8],
        )

    @staticmethod
    def _read_state_header_from(shm: shared_memory.SharedMemory) -> _StateHeader:
        values = STATE_HEADER_STRUCT.unpack_from(shm.buf, STATE_HEADER_OFFSET)
        return _StateHeader(
            sequence=values[0],
            sim_alive=bool(values[1]),
            sim_time=values[3],
            timestep=values[4],
            wall_time=values[5],
        )

    @staticmethod
    def _read_command_header_from(shm: shared_memory.SharedMemory) -> _CommandHeader:
        values = COMMAND_HEADER_STRUCT.unpack_from(shm.buf, COMMAND_HEADER_OFFSET)
        return _CommandHeader(
            sequence=values[0],
            enabled=bool(values[1]),
            mode=values[2],
            wall_time=values[3],
        )

    @classmethod
    def _read_header_from(
        cls,
        shm: shared_memory.SharedMemory,
    ) -> SharedMemoryHeader:
        common = cls._read_common_header_from(shm)
        state = cls._read_state_header_from(shm)
        command = cls._read_command_header_from(shm)
        return SharedMemoryHeader(
            total_size=common.total_size,
            config_fingerprint=common.config_fingerprint,
            nq=common.dimensions.nq,
            nv=common.dimensions.nv,
            nu=common.dimensions.nu,
            nsensordata=common.dimensions.nsensordata,
            state_seq=state.sequence,
            command_seq=command.sequence,
            sim_alive=state.sim_alive,
            command_enabled=command.enabled,
            command_mode=command.mode,
            sim_time=state.sim_time,
            timestep=state.timestep,
            state_wall_time=state.wall_time,
            command_wall_time=command.wall_time,
        )

    def _read_header(self) -> SharedMemoryHeader:
        return self._read_header_from(self._shm)

    def _read_state_header(self) -> _StateHeader:
        return self._read_state_header_from(self._shm)

    def _read_command_header(self) -> _CommandHeader:
        return self._read_command_header_from(self._shm)

    def _write_common_header(self, header: _CommonHeader) -> None:
        COMMON_HEADER_STRUCT.pack_into(
            self._shm.buf,
            COMMON_HEADER_OFFSET,
            MAGIC,
            VERSION,
            ARRAYS_OFFSET,
            int(header.total_size),
            int(header.dimensions.nq),
            int(header.dimensions.nv),
            int(header.dimensions.nu),
            int(header.dimensions.nsensordata),
            int(header.config_fingerprint),
        )

    def _write_state_header(self, header: _StateHeader) -> None:
        STATE_HEADER_STRUCT.pack_into(
            self._shm.buf,
            STATE_HEADER_OFFSET,
            int(header.sequence),
            int(bool(header.sim_alive)),
            0,
            float(header.sim_time),
            float(header.timestep),
            float(header.wall_time),
        )

    def _write_command_header(self, header: _CommandHeader) -> None:
        COMMAND_HEADER_STRUCT.pack_into(
            self._shm.buf,
            COMMAND_HEADER_OFFSET,
            int(header.sequence),
            int(bool(header.enabled)),
            int(header.mode),
            float(header.wall_time),
        )

    def _zero_arrays(self) -> None:
        self._shm.buf[ARRAYS_OFFSET : self.layout.total_size] = b"\0" * (
            self.layout.total_size - ARRAYS_OFFSET
        )

    def _validate_fields(
        self,
        fields: dict[str, object],
        layout_fields: dict[str, FieldLayout],
        label: str,
    ) -> dict[str, list[float]]:
        missing = sorted(set(layout_fields) - set(fields))
        unknown = sorted(set(fields) - set(layout_fields))
        if missing:
            raise ValueError(f"missing {label} field(s): {', '.join(missing)}")
        if unknown:
            raise ValueError(f"unknown {label} field(s): {', '.join(unknown)}")

        return {
            name: _as_float_list(fields[name], layout.size, name)
            for name, layout in layout_fields.items()
        }

    def _read_fields(
        self,
        layout_fields: dict[str, FieldLayout],
    ) -> dict[str, list[float]]:
        return {
            name: self._unpack_doubles(field.offset, field.size)
            for name, field in layout_fields.items()
        }

    def _pack_doubles(self, offset: int, values: Sequence[float]) -> None:
        if not values:
            return
        struct.pack_into(f"<{len(values)}d", self._shm.buf, offset, *values)

    def _unpack_doubles(self, offset: int, count: int) -> list[float]:
        if count == 0:
            return []
        return list(struct.unpack_from(f"<{count}d", self._shm.buf, offset))
