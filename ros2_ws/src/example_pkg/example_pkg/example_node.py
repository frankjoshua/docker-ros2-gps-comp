import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from .waypoint import Waypoint
from .waypoint_follower import WaypointFollower
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy


class ExampleNode(Node):
    def __init__(self):
        super().__init__('gps_waypoint_follower')
        # Initialize twist message
        self.twist_msg = Twist()
        self.waypoint_follower = WaypointFollower(self.twist_msg, self.get_logger())

        self.current_waypoint = Waypoint(0,0,"")
        self.waypoint_list = []  # New list to store Waypoint objects

        # Create a subscriber to the fix topic
        self.fix_subscription = self.create_subscription(
            NavSatFix,
            '/fix',
            self.fix_callback,
            10
        )
        
        # Create a subscriber to the 'waypoint_list_topic'
        self.subscription = self.create_subscription(
            String,
            'waypoint_list_topic',
            self.waypoint_list_callback,  # New callback function
            10
        )

        # Create a publisher to the 'output_string_topic'
        self.publisher = self.create_publisher(
            String,
            'output_string_topic',
            10
        )

        # Initialize the last received message with a default value
        self.last_received_string = "No message received yet!"

        # Create a timer that triggers every 2 seconds
        self.timer = self.create_timer(2.0, self.timer_callback)
        
    
        # Create a subscriber to the heading topic
        self.heading_subscription = self.create_subscription(
            Float32,
            '/heading',
            self.heading_callback,
            10
        )

        # Add a new attribute to store the heading
        self.current_heading = 0.0

        # Define a custom QoS profile
        custom_qos_profile = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create a publisher for Twist messages with the custom QoS profile
        self.twist_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            qos_profile=custom_qos_profile
        )

        # Create a timer for publishing Twist messages (e.g., every 0.1 seconds)
        self.twist_timer = self.create_timer(0.1, self.publish_twist)

    def fix_callback(self, msg):
        self.current_waypoint.latitude = msg.latitude
        self.current_waypoint.longitude = msg.longitude
        self.get_logger().debug(f'Fix: Latitude: {self.current_waypoint.latitude}, Longitude: {self.current_waypoint.longitude}, Altitude: {msg.altitude}')

    def waypoint_list_callback(self, msg):
        # Example of expected CSV format:
        # "latitude1,longitude1,latitude2,longitude2,latitude3,longitude3,..."
        # e.g., "47.6062,-122.3321,34.0522,-118.2437,40.7128,-74.0060"
        # Parse the CSV string and create Waypoint objects
        waypoints = []
        csv_data = msg.data.strip().split(',')
        
        for i in range(0, len(csv_data), 2):
            if i + 1 < len(csv_data):
                lat = float(csv_data[i])
                lon = float(csv_data[i + 1])
                waypoint = Waypoint(latitude=lat, longitude=lon, description="")
                waypoints.append(waypoint)

        self.waypoint_list = waypoints
        self.get_logger().info(f'Received {len(self.waypoint_list)} waypoints')

    def timer_callback(self):
        # Publish the last received string every 2 seconds
        self.waypoint_follower.update(self.waypoint_list, self.current_waypoint, self.current_heading)

    def heading_callback(self, msg):
        self.current_heading = msg.data
        self.get_logger().debug(f'Received heading: {self.current_heading} degrees')

    def publish_twist(self):
        # Publish the Twist message
        self.twist_publisher.publish(self.twist_msg)
        # self.get_logger().info('Published Twist message')


def main(args=None):
    rclpy.init(args=args)
    node = ExampleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


