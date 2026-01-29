#!/usr/bin/env python3
"""
Central Command Node - mission state machine and single authority for movement/camera.

- Sends action goals to Movement, Mapping, Detection, Camera nodes.
- Receives feedback and turns it into ordered movement/camera commands.
- Publishes mission status on /central_command/mission_status.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from uav_msgs.msg import MissionStatus
from uav_msgs.action import MoveToGpsWaypoint, StartMapping, StartDetection, MoveCamera


class CentralCommandNode(Node):
    def __init__(self):
        super().__init__("central_command_node")

        # Action clients (send goals to other nodes)
        self._move_client = ActionClient(self, MoveToGpsWaypoint, "/movement/move_to_gps_waypoint")
        self._mapping_client = ActionClient(self, StartMapping, "/mapping/start")
        self._detection_client = ActionClient(self, StartDetection, "/detection/start")
        self._camera_client = ActionClient(self, MoveCamera, "/camera/move")

        # Publisher: mission status
        self._status_pub = self.create_publisher(
            MissionStatus,
            "/central_command/mission_status",
            10,
        )

        self.get_logger().info("Central Command node started. Implement state machine and action handling.")

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
