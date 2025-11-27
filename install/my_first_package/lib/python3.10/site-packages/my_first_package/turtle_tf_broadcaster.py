import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import numpy as np

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*cj
    cs = ci*sj
    sc = si*cj
    ss = si*sj

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs # x
    q[1] = cj*ss + sj*cc # y
    q[2] = ck*cs - sk*cc # z
    q[3] = ck*cc + sk*ss # w
    return q

class TurtleTFBroadcaster(Node):

    def __init__(self):
        super().__init__('turtle_tf_broadcaster')
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.handle_turtle_pose,
            10)
        self.subscription  # prevent unused variable warning
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info("Turtlesim -> TF Bridge Started.")

    def handle_turtle_pose(self, msg):
        t = TransformStamped()

        # 현재 시간을 메시지 헤더에 입력
        t.header.stamp = self.get_clock().now().to_msg()
        # 부모 좌표계: 지도(map)
        t.header.frame_id = 'map'
        # 자식 좌표계: 거북이 몸체(base_link)
        t.child_frame_id = 'base_link'

        # Turtlesim의 2D 좌표(x,y)를 3D 좌표로 변환 (z는 0)
        # Turtlesim은 중앙이 (5.5, 5.5)이므로 원점 보정
        t.transform.translation.x = msg.x - 5.5
        t.transform.translation.y = msg.y - 5.5
        t.transform.translation.z = 0.0

        # 거북이의 각도(theta, Yaw값)를 3D 회전(쿼터니언)으로 변환
        q = quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # TF 신호 발송
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = TurtleTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()