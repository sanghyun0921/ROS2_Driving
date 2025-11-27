import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import random


ANGLE_MIN = 0.0
ANGLE_MAX = 2.0 * math.pi
ANGLE_INCREMENT = math.radians(1.0) 
SCAN_TIME = 0.1
RANGE_MIN = 0.1
RANGE_MAX = 3.5

WALL_DISTANCE = 0.3  
OPEN_DISTANCE = 3.0 

TIME_INCREMENT = SCAN_TIME / 360.0

class ScanPublisher(Node):
    def __init__(self):
        super().__init__('scan_publisher')
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.get_logger().info("Scan Publisher Started. (Randomly generates walls)")

    def create_base_scan(self):
        return [OPEN_DISTANCE for _ in range(360)]

    def generate_random_scenario(self):
        ranges = self.create_base_scan()
        scenario = random.choice(["FRONT_WALL", "LEFT_WALL", "RIGHT_WALL", "CLEAR"])

        if scenario == "FRONT_WALL":
            for i in range(350, 360): ranges[i] = WALL_DISTANCE
            for i in range(0, 10):    ranges[i] = WALL_DISTANCE
           
        elif scenario == "LEFT_WALL":
            for i in range(80, 100): ranges[i] = WALL_DISTANCE
           
        elif scenario == "RIGHT_WALL":
            for i in range(260, 280): ranges[i] = WALL_DISTANCE
           
        elif scenario == "CLEAR":
            pass

        return ranges, scenario

    def timer_callback(self):
        ranges, scenario_name = self.generate_random_scenario()

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame'
       
        scan.angle_min = ANGLE_MIN
        scan.angle_max = ANGLE_MAX
        scan.angle_increment = ANGLE_INCREMENT
        scan.time_increment = TIME_INCREMENT
        scan.scan_time = SCAN_TIME
        scan.range_min = RANGE_MIN
        scan.range_max = RANGE_MAX
       
        scan.ranges = ranges
        scan.intensities = [] 

        self.publisher_.publish(scan)
        self.get_logger().info(f"Published Scenario: [{scenario_name}]")

def main(args=None):
    rclpy.init(args=args)
    node = ScanPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()