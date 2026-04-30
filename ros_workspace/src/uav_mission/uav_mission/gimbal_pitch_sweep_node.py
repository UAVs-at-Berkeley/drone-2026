#!/usr/bin/env python3
"""
Cyclic pitch sweep for gimbal / camera testing.

Sends goals to the MoveCamera action (default /camera/move) so both hardware
and sim camera_node backends receive the same commands.
"""

import time
from enum import Enum, auto

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from uav_msgs.action import MoveCamera


class _Phase(Enum):
    DELAY = auto()
    HOLD = auto()
    SENDING = auto()
    DONE = auto()


class GimbalPitchSweepNode(Node):
    def __init__(self):
        super().__init__("gimbal_pitch_sweep")

        self.declare_parameter("action_name", "/camera/move")
        self.declare_parameter("pitch_angles", [0.0, -20.0, -40.0, -60.0])
        self.declare_parameter("hold_seconds", 3.0)
        self.declare_parameter("yaw_deg", 0.0)
        self.declare_parameter("roll_deg", 0.0)
        self.declare_parameter("start_delay_seconds", 2.0)
        self.declare_parameter("loop", True)

        action_name = self.get_parameter("action_name").get_parameter_value().string_value
        raw_angles = self.get_parameter("pitch_angles").get_parameter_value().double_array_value
        self._angles = [float(x) for x in raw_angles] if raw_angles else [0.0, -20.0, -40.0, -60.0]
        self._hold = float(self.get_parameter("hold_seconds").value)
        self._yaw = float(self.get_parameter("yaw_deg").value)
        self._roll = float(self.get_parameter("roll_deg").value)
        self._start_delay = float(self.get_parameter("start_delay_seconds").value)
        self._loop = bool(self.get_parameter("loop").value)

        self._client = ActionClient(self, MoveCamera, action_name)
        self._phase = _Phase.DELAY
        self._next_event = time.monotonic() + self._start_delay
        self._index = 0
        self._goal_in_flight = False

        self._timer = self.create_timer(0.1, self._tick)

        self.get_logger().info(
            "Gimbal pitch sweep → %s; angles %s, hold=%.1fs, loop=%s"
            % (action_name, self._angles, self._hold, self._loop)
        )

    def _tick(self):
        if self._phase is _Phase.DONE:
            return
        now = time.monotonic()
        if self._phase is _Phase.DELAY and now >= self._next_event:
            self._try_send()
            return
        if self._phase is _Phase.HOLD and not self._goal_in_flight and now >= self._next_event:
            self._try_send()

    def _try_send(self):
        if self._goal_in_flight:
            return
        if not self._client.server_is_ready():
            return
        if self._index >= len(self._angles) and not self._loop:
            self._phase = _Phase.DONE
            self.get_logger().info("Sweep complete (loop=false).")
            return
        if self._index >= len(self._angles):
            self._index = 0

        pitch = self._angles[self._index]
        goal = MoveCamera.Goal()
        goal.pitch_deg = float(pitch)
        goal.yaw_deg = self._yaw
        goal.roll_deg = self._roll

        self._goal_in_flight = True
        self._phase = _Phase.SENDING
        self.get_logger().info(
            "MoveCamera goal [%d/%d]: pitch=%.1f, yaw=%.1f, roll=%.1f"
            % (self._index + 1, len(self._angles), pitch, self._yaw, self._roll)
        )

        send_future = self._client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("MoveCamera goal rejected or failed to reach server")
            self._goal_in_flight = False
            self._next_event = time.monotonic() + 1.0
            self._phase = _Phase.HOLD
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        self._goal_in_flight = False
        result = future.result()
        r = result.result
        if r.success:
            self.get_logger().info("MoveCamera: %s" % r.message)
        else:
            self.get_logger().warn("MoveCamera failed: %s" % r.message)

        self._index += 1
        if self._index >= len(self._angles) and not self._loop:
            self._phase = _Phase.DONE
            self.get_logger().info("Last pitch reached; stopping (loop=false).")
            return
        self._phase = _Phase.HOLD
        self._next_event = time.monotonic() + self._hold


def main(args=None):
    rclpy.init(args=args)
    node = GimbalPitchSweepNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
