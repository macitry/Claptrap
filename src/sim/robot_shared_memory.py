"""Shared-memory robot I/O for the simulator.

The simulator owns one shared-memory block.  External controllers attach to the
same block, read the latest state, and write torque commands.  The state and
command metadata live in separate headers so the simulator and controller never
overwrite each other's sequence counters.
"""

from __future__ import annotations

import inspect
import struct
import time
from dataclasses import dataclass, replace
from multiprocessing import resource_tracker
from multiprocessing import shared_memory
from typing import Sequence


DEFAULT_SHM_NAME = "robot_mujoco_io"
COMMAND_MODE_TORQUE = 0

MAGIC = b"ROBOTIO\0"
VERSION = 1
DOUBLE_SIZE = 8

COMMON_HEADER_STRUCT = struct.Struct("<8sIIQIIII")
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
class SharedMemoryHeader:
    total_size: int
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
    nq: int
    nv: int
    nu: int
    nsensordata: int


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
class SharedMemoryLayout:
    nq: int
    nv: int
    nu: int
    nsensordata: int
    qpos_offset: int
    qvel_offset: int
    sensordata_offset: int
    ctrl_offset: int
    actuator_force_offset: int
    command_torque_offset: int
    total_size: int

    @classmethod
    def from_dimensions(
        cls,
        *,
        nq: int,
        nv: int,
        nu: int,
        nsensordata: int,
    ) -> "SharedMemoryLayout":
        offset = ARRAYS_OFFSET
        qpos_offset = offset
        offset += nq * DOUBLE_SIZE
        qvel_offset = offset
        offset += nv * DOUBLE_SIZE
        sensordata_offset = offset
        offset += nsensordata * DOUBLE_SIZE
        ctrl_offset = offset
        offset += nu * DOUBLE_SIZE
        actuator_force_offset = offset
        offset += nu * DOUBLE_SIZE
        command_torque_offset = offset
        offset += nu * DOUBLE_SIZE

        return cls(
            nq=nq,
            nv=nv,
            nu=nu,
            nsensordata=nsensordata,
            qpos_offset=qpos_offset,
            qvel_offset=qvel_offset,
            sensordata_offset=sensordata_offset,
            ctrl_offset=ctrl_offset,
            actuator_force_offset=actuator_force_offset,
            command_torque_offset=command_torque_offset,
            total_size=offset,
        )

    @classmethod
    def from_header(cls, header: SharedMemoryHeader) -> "SharedMemoryLayout":
        layout = cls.from_dimensions(
            nq=header.nq,
            nv=header.nv,
            nu=header.nu,
            nsensordata=header.nsensordata,
        )
        if layout.total_size != header.total_size:
            raise ValueError(
                "Shared-memory size mismatch: "
                f"header={header.total_size}, computed={layout.total_size}"
            )
        return layout


@dataclass(frozen=True)
class RobotState:
    sequence: int
    sim_alive: bool
    sim_time: float
    timestep: float
    wall_time: float
    qpos: list[float]
    qvel: list[float]
    sensordata: list[float]
    ctrl: list[float]
    actuator_force: list[float]


@dataclass(frozen=True)
class RobotCommand:
    sequence: int
    enabled: bool
    mode: int
    wall_time: float
    torque: list[float]


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

    # Python < 3.13 tracks attached shared memory in every independent process.
    # Unregister non-owner attachments so a short-lived client does not unlink
    # the simulator-owned block when it exits.
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


def _as_float_list(values: Sequence[float], expected: int, label: str) -> list[float]:
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
        *,
        owner: bool,
        unlink_on_close: bool,
    ) -> None:
        self._shm = shm
        self.layout = layout
        self.owner = owner
        self.unlink_on_close = unlink_on_close

    @property
    def name(self) -> str:
        return self._shm.name

    @classmethod
    def create(
        cls,
        *,
        name: str = DEFAULT_SHM_NAME,
        nq: int,
        nv: int,
        nu: int,
        nsensordata: int,
        timestep: float,
        unlink_existing: bool = True,
        unlink_on_close: bool = True,
    ) -> "RobotSharedMemory":
        layout = SharedMemoryLayout.from_dimensions(
            nq=nq,
            nv=nv,
            nu=nu,
            nsensordata=nsensordata,
        )

        try:
            shm = _open_shared_memory(
                name=name,
                create=True,
                size=layout.total_size,
                track=True,
            )
        except FileExistsError:
            if not unlink_existing:
                raise
            old_shm = _open_shared_memory(
                name=name,
                create=False,
                track=False,
                unregister_attached=False,
            )
            try:
                old_shm.unlink()
            finally:
                old_shm.close()
            shm = _open_shared_memory(
                name=name,
                create=True,
                size=layout.total_size,
                track=True,
            )

        io = cls(shm, layout, owner=True, unlink_on_close=unlink_on_close)
        now = time.time()
        io._write_common_header(
            _CommonHeader(
                total_size=layout.total_size,
                nq=nq,
                nv=nv,
                nu=nu,
                nsensordata=nsensordata,
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
    def attach(cls, name: str = DEFAULT_SHM_NAME) -> "RobotSharedMemory":
        shm = _open_shared_memory(name=name, create=False, track=False)
        try:
            header = cls._read_header_from(shm)
            layout = SharedMemoryLayout.from_header(header)
            if shm.size < layout.total_size:
                raise ValueError(
                    f"Shared-memory block is too small: {shm.size} < "
                    f"{layout.total_size}"
                )
        except Exception:
            shm.close()
            raise
        return cls(shm, layout, owner=False, unlink_on_close=False)

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
        qpos: Sequence[float],
        qvel: Sequence[float],
        sensordata: Sequence[float],
        ctrl: Sequence[float],
        actuator_force: Sequence[float],
    ) -> None:
        qpos_values = _as_float_list(qpos, self.layout.nq, "qpos")
        qvel_values = _as_float_list(qvel, self.layout.nv, "qvel")
        sensor_values = _as_float_list(
            sensordata, self.layout.nsensordata, "sensordata"
        )
        ctrl_values = _as_float_list(ctrl, self.layout.nu, "ctrl")
        force_values = _as_float_list(
            actuator_force, self.layout.nu, "actuator_force"
        )

        state_header = self._read_state_header()
        odd_seq = _next_odd_sequence(state_header.sequence)

        self._write_state_header(replace(state_header, sequence=odd_seq))
        self._pack_doubles(self.layout.qpos_offset, qpos_values)
        self._pack_doubles(self.layout.qvel_offset, qvel_values)
        self._pack_doubles(self.layout.sensordata_offset, sensor_values)
        self._pack_doubles(self.layout.ctrl_offset, ctrl_values)
        self._pack_doubles(self.layout.actuator_force_offset, force_values)
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

            qpos = self._unpack_doubles(self.layout.qpos_offset, self.layout.nq)
            qvel = self._unpack_doubles(self.layout.qvel_offset, self.layout.nv)
            sensordata = self._unpack_doubles(
                self.layout.sensordata_offset,
                self.layout.nsensordata,
            )
            ctrl = self._unpack_doubles(self.layout.ctrl_offset, self.layout.nu)
            actuator_force = self._unpack_doubles(
                self.layout.actuator_force_offset,
                self.layout.nu,
            )

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
                    qpos=qpos,
                    qvel=qvel,
                    sensordata=sensordata,
                    ctrl=ctrl,
                    actuator_force=actuator_force,
                )

            if time.monotonic() >= deadline:
                raise TimeoutError("Timed out waiting for a stable state read")

    def write_torque(
        self,
        torque: Sequence[float],
        *,
        enabled: bool = True,
        mode: int = COMMAND_MODE_TORQUE,
    ) -> None:
        torque_values = _as_float_list(torque, self.layout.nu, "torque")
        command_header = self._read_command_header()
        odd_seq = _next_odd_sequence(command_header.sequence)

        self._write_command_header(replace(command_header, sequence=odd_seq))
        self._pack_doubles(self.layout.command_torque_offset, torque_values)
        self._write_command_header(
            _CommandHeader(
                sequence=odd_seq + 1,
                enabled=bool(enabled),
                mode=int(mode),
                wall_time=time.time(),
            )
        )

    def disable_command(self) -> None:
        self.write_torque([0.0] * self.layout.nu, enabled=False)

    def read_command(self, timeout: float = 1.0) -> RobotCommand:
        deadline = time.monotonic() + timeout
        while True:
            command_before = self._read_command_header()
            if command_before.sequence % 2:
                if time.monotonic() >= deadline:
                    raise TimeoutError("Timed out waiting for a stable command write")
                time.sleep(0)
                continue

            torque = self._unpack_doubles(
                self.layout.command_torque_offset,
                self.layout.nu,
            )

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
                    torque=torque,
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
            nq=values[4],
            nv=values[5],
            nu=values[6],
            nsensordata=values[7],
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
            nq=common.nq,
            nv=common.nv,
            nu=common.nu,
            nsensordata=common.nsensordata,
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
            int(header.nq),
            int(header.nv),
            int(header.nu),
            int(header.nsensordata),
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

    def _pack_doubles(self, offset: int, values: Sequence[float]) -> None:
        if not values:
            return
        struct.pack_into(f"<{len(values)}d", self._shm.buf, offset, *values)

    def _unpack_doubles(self, offset: int, count: int) -> list[float]:
        if count == 0:
            return []
        return list(struct.unpack_from(f"<{count}d", self._shm.buf, offset))
