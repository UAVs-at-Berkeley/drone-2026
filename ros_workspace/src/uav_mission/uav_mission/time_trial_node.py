"""
Time-trial action server. Waypoints are defined only as ``environment.waypoints.points``
(``[lat, long, alt_m]``) in the mission YAML; ``mission_loader`` fills ``StartTimeTrial``
goals and Central Command sends them here. This node does not read ROS parameters for
waypoint lists.
"""
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPoseStamped
from uav_msgs.action import StartTimeTrial
from uav_mission.utils import haversine, tsp_waypoint_optimizer

class TimeTrialNode(Node):
    def __init__(self):
        super().__init__("time_trial_node")
        self.get_logger().info("Time trial node started")

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_alt = 0.0

        self._time_trial_action_server = ActionServer(
            self,
            StartTimeTrial,
            '/time_trial/start',
            self.execute_goal,
        )

        self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.on_position,
            qos,
        )

        self.setpoint_pub = self.create_publisher(
            GeoPoseStamped,
            '/mavros/setpoint_position/global',
            10,
        )

    def on_position(self, msg: NavSatFix):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_alt = msg.altitude
    
    def execute_goal(self, goal_handle):
        result = StartTimeTrial.Result()
        result.success = False
        result.message = ""
        result.total_time_seconds = 0.0
        
        waypoints = list(zip(goal_handle.request.latitudes, goal_handle.request.longitudes, goal_handle.request.altitudes, strict=True))
        optimized_waypoints = tsp_waypoint_optimizer(waypoints)
        
        for i, waypoint in enumerate(optimized_waypoints):
            if i == 1:
                start_time = self.get_clock().now()
            to_lat, to_lon, to_alt = waypoint[0], waypoint[1], waypoint[2]
            last_feedback_time = self.get_clock().now()
            while haversine(lat1=self.current_lat, lon1=self.current_lon, lat2=to_lat, lon2=to_lon) > 4.0:
                to_waypoint = GeoPoseStamped()
                to_waypoint.pose.position.latitude = to_lat
                to_waypoint.pose.position.longitude = to_lon 
                to_waypoint.pose.position.altitude = to_alt
                to_waypoint.pose.orientation.w = 1.0 # I will default to this value for now

                self.setpoint_pub.publish(to_waypoint)
                time.sleep(0.05)

                if (self.get_clock().now() - last_feedback_time).nanoseconds / 1e9 >= 2.0:
                    feedback = StartTimeTrial.Feedback()
                    feedback.current_waypoint_index = i + 1
                    feedback.total_waypoints = len(optimized_waypoints)
                    feedback.distance_to_waypoint = haversine(lat1=self.current_lat, lon1=self.current_lon, lat2=to_lat, lon2=to_lon)
                    self.get_logger().info(f"[{feedback.current_waypoint_index}/{feedback.total_waypoints}] {feedback.distance_to_waypoint:.1f}m to ({to_lat:.6f}, {to_lon:.6f})")
                    goal_handle.publish_feedback(feedback)
                    
                    last_feedback_time = self.get_clock().now()
        end_time = self.get_clock().now()

        result.success = True
        result.message = "Time trial mission completed successfully."
        result.total_time_seconds = (end_time - start_time).nanoseconds / 1e9
        goal_handle.succeed()

        return result
    
def main():
    rclpy.init()
    mission = TimeTrialNode()
    executor = MultiThreadedExecutor()
    executor.add_node(mission)
    try:
        executor.spin()    
    except KeyboardInterrupt:
        pass
    finally:
        mission.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
