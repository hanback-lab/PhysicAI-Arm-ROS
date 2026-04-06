#!/usr/bin/env python3
import time
import threading
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import yaml


def clamp(x: float, lo: float, hi: float) -> float:
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


@dataclass
class JointConfig:
    joint_names: List[str]
    servo_id: Dict[str, int]
    sign: Dict[str, int]
    zero_offset_rad: Dict[str, float]
    limit_rad: Dict[str, Tuple[float, float]]
    max_vel_rad_s: Dict[str, float]
    publish_hz: float
    control_hz: float
    command_timeout_ms: int
    device: str
    baudrate: int
    ticks_per_rev: int
    center_tick: int
    servo_type: str


def load_joint_config(path: str) -> JointConfig:
    with open(path, "r", encoding="utf-8") as f:
        d = yaml.safe_load(f)
    joint_names = list(d["joint_names"])
    servo_id = {k: int(v) for k, v in d["servo_id"].items()}
    sign = {k: int(v) for k, v in d.get("sign", {}).items()}
    zero_offset_rad = {k: float(v) for k, v in d.get("zero_offset_rad", {}).items()}
    limit_rad = {k: (float(v[0]), float(v[1])) for k, v in d.get("limit_rad", {}).items()}
    max_vel_rad_s = {k: float(v) for k, v in d.get("max_vel_rad_s", {}).items()}
    publish_hz = float(d.get("publish_hz", 50))
    control_hz = float(d.get("control_hz", 100))
    command_timeout_ms = int(d.get("command_timeout_ms", 300))
    device = str(d.get("device", "/dev/ttyUSB0"))
    baudrate = int(d.get("baudrate", 1000000))
    ft = d.get("feetech", {}) or {}
    ticks_per_rev = int(ft.get("ticks_per_rev", 4096))
    center_tick = int(ft.get("center_tick", ticks_per_rev // 2))
    servo_type = str(ft.get("servo_type", ft.get("model", "sts"))).lower()
    if "sts" in servo_type or "3215" in servo_type:
        servo_type = "sts"
    for jn in joint_names:
        sign.setdefault(jn, 1)
        zero_offset_rad.setdefault(jn, 0.0)
        limit_rad.setdefault(jn, (-3.14159, 3.14159))
        max_vel_rad_s.setdefault(jn, 1.0)
    return JointConfig(
        joint_names=joint_names,
        servo_id=servo_id,
        sign=sign,
        zero_offset_rad=zero_offset_rad,
        limit_rad=limit_rad,
        max_vel_rad_s=max_vel_rad_s,
        publish_hz=publish_hz,
        control_hz=control_hz,
        command_timeout_ms=command_timeout_ms,
        device=device,
        baudrate=baudrate,
        ticks_per_rev=ticks_per_rev,
        center_tick=center_tick,
        servo_type=servo_type,
    )


class FeetechInterface:
    def __init__(self, cfg: JointConfig, simulate: bool = False):
        self.cfg = cfg
        self.simulate = bool(simulate)
        self._sim_pos = [0.0 for _ in cfg.joint_names]
        self._controller = None
        self._servo_ids = [cfg.servo_id[jn] for jn in cfg.joint_names]
        if not self.simulate:
            self._init_real_bus()

    def _init_real_bus(self):
        try:
            from vassar_feetech_servo_sdk import ServoController
        except Exception as e:
            raise RuntimeError("vassar_feetech_servo_sdk import failed. Install: pip install vassar-feetech-servo-sdk") from e
        self._controller = ServoController(
            servo_ids=self._servo_ids,
            servo_type=self.cfg.servo_type,
            port=self.cfg.device,
            baudrate=self.cfg.baudrate,
        )
        self._controller.connect()

    def close(self):
        if self._controller is not None:
            try:
                self._controller.disconnect()
            finally:
                self._controller = None

    def disable_all_servos(self):
        if self.simulate:
            return
        self._controller.disable_all_servos()

    def rad_to_tick(self, jn: str, rad: float) -> int:
        s = self.cfg.sign[jn]
        z = self.cfg.zero_offset_rad[jn]
        x = (rad - z) * s
        return int(round(self.cfg.center_tick + (x / (2.0 * 3.141592653589793)) * self.cfg.ticks_per_rev))

    def tick_to_rad(self, jn: str, tick: int) -> float:
        s = self.cfg.sign[jn]
        z = self.cfg.zero_offset_rad[jn]
        x = (tick - self.cfg.center_tick) * (2.0 * 3.141592653589793) / self.cfg.ticks_per_rev
        return float((x * s) + z)

    def read_positions_rad(self) -> List[float]:
        if self.simulate:
            return list(self._sim_pos)
        positions = self._controller.read_all_positions()
        out = []
        for jn in self.cfg.joint_names:
            sid = self.cfg.servo_id[jn]
            if sid not in positions:
                raise RuntimeError(f"read_all_positions missing servo id {sid}")
            out.append(self.tick_to_rad(jn, int(positions[sid])))
        return out

    def write_positions_rad(self, positions: List[float], speed: int, acceleration: int) -> None:
        if self.simulate:
            self._sim_pos = list(positions)
            return
        cmd = {}
        for jn, rad in zip(self.cfg.joint_names, positions):
            cmd[self.cfg.servo_id[jn]] = self.rad_to_tick(jn, rad)
        results = self._controller.write_position(cmd, speed=speed, acceleration=acceleration)
        if isinstance(results, dict):
            fails = [sid for sid, ok in results.items() if not ok]
            if fails:
                raise RuntimeError(f"write_position failed for servo ids: {fails}")


class FeetechDriverNode(Node):
    def __init__(self):
        super().__init__("feetech_driver_node")
        self.declare_parameter("config_path", "joints.yaml")
        self.declare_parameter("simulate", False)
        self.declare_parameter("joint_targets_topic", "/joint_targets")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("safety_torque_topic", "/safety/torque_enable")
        self.declare_parameter("write_speed", 32767)
        self.declare_parameter("write_acceleration", 50)
        self.declare_parameter("state_read_hz", 0.0)
        self.declare_parameter("write_hz", 0.0)
        self.declare_parameter("command_timeout_ms", -1)
        self.declare_parameter("command_threshold_ticks", 1)
        self.declare_parameter("heartbeat_hz", 10.0)
        self.declare_parameter("hold_last_target_on_timeout", True)

        cfg_path = self.get_parameter("config_path").get_parameter_value().string_value
        self.cfg = load_joint_config(cfg_path)
        simulate = self.get_parameter("simulate").get_parameter_value().bool_value
        self.iface = FeetechInterface(self.cfg, simulate=simulate)

        self.joint_targets_topic = self.get_parameter("joint_targets_topic").get_parameter_value().string_value
        self.joint_states_topic = self.get_parameter("joint_states_topic").get_parameter_value().string_value
        self.safety_torque_topic = self.get_parameter("safety_torque_topic").get_parameter_value().string_value
        self.write_speed = int(self.get_parameter("write_speed").get_parameter_value().integer_value)
        self.write_acc = int(self.get_parameter("write_acceleration").get_parameter_value().integer_value)
        self.state_read_hz = float(self.get_parameter("state_read_hz").get_parameter_value().double_value) or self.cfg.publish_hz
        self.write_hz = float(self.get_parameter("write_hz").get_parameter_value().double_value) or self.cfg.control_hz
        self.command_timeout_ms = int(self.get_parameter("command_timeout_ms").get_parameter_value().integer_value)
        if self.command_timeout_ms < 0:
            self.command_timeout_ms = self.cfg.command_timeout_ms
        self.command_threshold_ticks = max(1, int(self.get_parameter("command_threshold_ticks").get_parameter_value().integer_value))
        self.heartbeat_hz = max(0.1, float(self.get_parameter("heartbeat_hz").get_parameter_value().double_value))
        self.hold_last_target_on_timeout = self.get_parameter("hold_last_target_on_timeout").get_parameter_value().bool_value

        self._state_lock = threading.Lock()
        self._io_lock = threading.Lock()
        self._torque_requested = True
        self._torque_enabled = True
        self._last_cmd_time = self.get_clock().now()
        self._last_write_wall = 0.0

        initial = [0.0 for _ in self.cfg.joint_names]
        try:
            with self._io_lock:
                initial = self.iface.read_positions_rad()
        except Exception as e:
            self.get_logger().warn(f"initial read_positions_rad failed: {e}")

        self._measured = list(initial)
        self._cmded = list(initial)
        self._target_map: Dict[str, float] = {jn: p for jn, p in zip(self.cfg.joint_names, initial)}
        self._last_written_ticks = [self.iface.rad_to_tick(jn, p) for jn, p in zip(self.cfg.joint_names, initial)]

        qos1 = QoSProfile(depth=1)
        self.pub = self.create_publisher(JointState, self.joint_states_topic, 10)
        self.sub = self.create_subscription(JointState, self.joint_targets_topic, self.on_target, qos1)
        self.safety_sub = self.create_subscription(Bool, self.safety_torque_topic, self.on_safety_torque_bool, 10)

        self._state_timer = self.create_timer(1.0 / max(1.0, self.state_read_hz), self.state_step)
        self._write_timer = self.create_timer(1.0 / max(1.0, self.write_hz), self.write_step)

    def destroy_node(self):
        try:
            self.iface.close()
        finally:
            super().destroy_node()

    def on_safety_torque_bool(self, msg: Bool) -> None:
        with self._state_lock:
            self._torque_requested = bool(msg.data)

    def on_target(self, msg: JointState) -> None:
        if not msg.name or not msg.position or len(msg.name) != len(msg.position):
            return
        with self._state_lock:
            for n, p in zip(msg.name, msg.position):
                if n in self._target_map:
                    lo, hi = self.cfg.limit_rad[n]
                    self._target_map[n] = clamp(float(p), lo, hi)
            self._last_cmd_time = self.get_clock().now()

    def _apply_torque_request(self) -> bool:
        with self._state_lock:
            requested = self._torque_requested
        if requested == self._torque_enabled:
            return self._torque_enabled
        with self._io_lock:
            if not requested:
                self.iface.disable_all_servos()
                self._torque_enabled = False
                return False
            hold = list(self._measured)
            self.iface.write_positions_rad(hold, speed=self.write_speed, acceleration=self.write_acc)
            self._torque_enabled = True
            self._cmded = list(hold)
            self._target_map = {jn: p for jn, p in zip(self.cfg.joint_names, hold)}
            self._last_written_ticks = [self.iface.rad_to_tick(jn, p) for jn, p in zip(self.cfg.joint_names, hold)]
            self._last_cmd_time = self.get_clock().now()
            self._last_write_wall = time.monotonic()
            return True

    def state_step(self) -> None:
        try:
            with self._io_lock:
                measured = self.iface.read_positions_rad()
        except Exception as e:
            self.get_logger().warn(f"read_positions_rad failed: {e}")
            return
        with self._state_lock:
            self._measured = list(measured)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.cfg.joint_names)
        msg.position = list(measured)
        self.pub.publish(msg)

    def write_step(self) -> None:
        try:
            torque_enabled = self._apply_torque_request()
        except Exception as e:
            self.get_logger().warn(f"torque apply failed: {e}")
            return
        if not torque_enabled:
            return

        now_wall = time.monotonic()
        with self._state_lock:
            dt_ms = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e6
            timed_out = dt_ms > float(self.command_timeout_ms)
            if timed_out and self.hold_last_target_on_timeout:
                desired = list(self._cmded)
            elif timed_out:
                desired = list(self._measured)
            else:
                desired = [self._target_map[jn] for jn in self.cfg.joint_names]

        desired = [clamp(p, *self.cfg.limit_rad[jn]) for jn, p in zip(self.cfg.joint_names, desired)]
        desired_ticks = [self.iface.rad_to_tick(jn, p) for jn, p in zip(self.cfg.joint_names, desired)]
        changed = any(abs(a - b) >= self.command_threshold_ticks for a, b in zip(desired_ticks, self._last_written_ticks))
        heartbeat_due = (now_wall - self._last_write_wall) >= (1.0 / self.heartbeat_hz)

        if not changed and not heartbeat_due:
            return

        exact = [self.iface.tick_to_rad(jn, t) for jn, t in zip(self.cfg.joint_names, desired_ticks)]
        try:
            with self._io_lock:
                self.iface.write_positions_rad(exact, speed=self.write_speed, acceleration=self.write_acc)
        except Exception as e:
            self.get_logger().warn(f"write_positions_rad failed: {e}")
            return

        with self._state_lock:
            self._cmded = list(exact)
            self._last_written_ticks = list(desired_ticks)
            self._last_write_wall = now_wall


def main():
    rclpy.init()
    node = FeetechDriverNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
