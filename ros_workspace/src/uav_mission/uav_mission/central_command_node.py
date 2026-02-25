#!/usr/bin/env python3
"""
Central Command Node - mission state machine and single authority for flight and camera.

- On start: arms and takeoffs via PX4 DDS (/fmu/in/vehicle_command), waits for completion,
  then sends RunWaypointMission action goal to the waypoint node.
- Sends action goals to Waypoint, Mapping, Detection, Camera nodes.
- Publishes mission status on /central_command/mission_status.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from uav_msgs.msg import MissionStatus
from uav_msgs.action import StartMapping, StartDetection, MoveCamera, RunWaypointMission
from px4_msgs.msg import VehicleCommand, VehicleCommandAck, TakeoffStatus


# PX4 command constants (from px4_msgs)
VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
VEHICLE_CMD_NAV_TAKEOFF = 22
ARMING_ACTION_ARM = 1
VEHICLE_CMD_RESULT_ACCEPTED = 0
TAKEOFF_STATE_FLIGHT = 5

# Default takeoff altitude (m AMSL) when not using current position
DEFAULT_TAKEOFF_ALTITUDE_M = 5.0


class CentralCommandNode(Node):
    def __init__(self):
        super().__init__("central_command_node")

        # --- PX4 DDS: command publisher and status subscribers ---
        self._cmd_pub = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            10,
        )
        qos_sensor = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._ack_sub = self.create_subscription(
            VehicleCommandAck,
            "/fmu/out/vehicle_command_ack",
            self._on_vehicle_command_ack,
            qos_sensor,
        )
        self._takeoff_status_sub = self.create_subscription(
            TakeoffStatus,
            "/fmu/out/takeoff_status",
            self._on_takeoff_status,
            qos_sensor,
        )

        # Action clients
        self._waypoint_client = ActionClient(self, RunWaypointMission, "/waypoint/run_mission")
        self._mapping_client = ActionClient(self, StartMapping, "/mapping/start")
        self._detection_client = ActionClient(self, StartDetection, "/detection/start")
        self._camera_client = ActionClient(self, MoveCamera, "/camera/move")

        self._status_pub = self.create_publisher(
            MissionStatus,
            "/central_command/mission_status",
            10,
        )

        # Startup sequence state
        self._phase = "idle"  # idle -> arming -> arm_sent -> takeoff_sent -> takeoff_wait -> waypoint_sent -> done
        self._pending_ack_command = None
        self._waypoint_goal_handle = None

        self.declare_parameter("takeoff_altitude_m", DEFAULT_TAKEOFF_ALTITUDE_M)

        # Start sequence shortly after bringup so subscribers/publishers are ready
        self._start_timer = self.create_timer(1.0, self._on_start_timer)

        self.get_logger().info(
            "Central Command node started. Will arm, takeoff via PX4 DDS, then send RunWaypointMission to waypoint node."
        )

    def _on_start_timer(self):
        self._start_timer.cancel()
        if self._phase != "idle":
            return
        self._phase = "arming"
        self._send_arm_command()

    def _send_vehicle_command(self, command: int, param1: float = 0.0, param5: float = 0.0, param6: float = 0.0, param7: float = 0.0):
        """Publish a single VehicleCommand to PX4 (DDS)."""
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds // 1000)
        msg.command = command
        msg.param1 = param1
        msg.param5 = param5
        msg.param6 = param6
        msg.param7 = param7
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.confirmation = 0
        msg.from_external = True
        self._cmd_pub.publish(msg)
        self._pending_ack_command = command

    def _send_arm_command(self):
        self.get_logger().info("Sending ARM command to PX4.")
        self._send_vehicle_command(VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=float(ARMING_ACTION_ARM))
        self._phase = "arm_sent"

    def _send_takeoff_command(self):
        alt = self.get_parameter("takeoff_altitude_m").value
        self.get_logger().info("Sending TAKEOFF command to PX4 (alt=%.1f m)." % alt)
        # param5=lat, param6=lon, param7=alt (use 0,0 for current position behavior if supported, else nominal)
        self._send_vehicle_command(VEHICLE_CMD_NAV_TAKEOFF, param5=0.0, param6=0.0, param7=float(alt))
        self._phase = "takeoff_wait"

    def _on_vehicle_command_ack(self, msg: VehicleCommandAck):
        if self._phase != "arm_sent" or self._pending_ack_command is None:
            return
        if msg.command != self._pending_ack_command:
            return
        self._pending_ack_command = None
        if msg.result != VEHICLE_CMD_RESULT_ACCEPTED:
            self.get_logger().error("PX4 rejected command %u (result=%u)." % (msg.command, msg.result))
            self._phase = "idle"
            self.publish_status("error", "arm_rejected", True, "PX4 rejected arm")
            return
        if msg.command == VEHICLE_CMD_COMPONENT_ARM_DISARM:
            self.get_logger().info("Arm accepted. Sending takeoff.")
            self._send_takeoff_command()

    def _on_takeoff_status(self, msg: TakeoffStatus):
        if self._phase != "takeoff_wait":
            return
        if msg.takeoff_state != TAKEOFF_STATE_FLIGHT:
            return
        self.get_logger().info("Takeoff complete (in flight). Sending RunWaypointMission to waypoint node.")
        self._phase = "waypoint_sent"
        self._send_waypoint_mission_goal()

    def _send_waypoint_mission_goal(self):
        self._waypoint_client.wait_for_server(timeout_sec=5.0)
        goal_msg = RunWaypointMission.Goal()
        goal_msg.placeholder = 0
        self._waypoint_client.send_goal_async(
            goal_msg,
            feedback_callback=self._waypoint_feedback_callback,
        ).add_done_callback(self._waypoint_goal_done_callback)

    def _waypoint_feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            "Waypoint mission feedback: phase=%s waypoint %.0f/%.0f" % (fb.phase, fb.current_waypoint_index, fb.total_waypoints),
            throttle_duration_sec=2.0,
        )

    def _waypoint_goal_done_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Waypoint node rejected RunWaypointMission goal.")
            self.publish_status("waypoint", "rejected", False, "Waypoint goal rejected")
            return
        self._waypoint_goal_handle = goal_handle
        self.get_logger().info("RunWaypointMission goal accepted by waypoint node.")
        self.publish_status("waypoint", "running", False, "")

    def publish_status(self, current_mode: str, phase: str, movement_locked: bool, last_error: str = ""):
        msg = MissionStatus()
        msg.current_mode = current_mode
        msg.phase = phase
        msg.movement_locked = movement_locked
        msg.last_error = last_error
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CentralCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
