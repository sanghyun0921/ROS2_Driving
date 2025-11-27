# main.py 실행할 컴퓨터에서 실행

import roslibpy
import numpy as np
import time

ROS_HOST = '192.168.0.7'  
ROS_PORT = 9090           

velocity_publisher = None

def move_turtle(action):
    if velocity_publisher is None:
        return

    twist = {
        'linear':  {'x': 0.0, 'y': 0.0, 'z': 0.0},
        'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
    }

    if action == "go_forward":
        twist['linear']['x'] = 2.0
        twist['angular']['z'] = 0.0
    elif action == "turn_left":
        twist['linear']['x'] = 0.0
        twist['angular']['z'] = 2.0 
    elif action == "turn_right":
        twist['linear']['x'] = 0.0
        twist['angular']['z'] = -2.0 

    velocity_publisher.publish(roslibpy.Message(twist))

def callback(message):
    global velocity_publisher
    
    ranges = np.array(message['ranges'])
    ranges = np.where(ranges == 0.0, 3.5, ranges)
    ranges = np.where(np.isinf(ranges), 3.5, ranges)

    try:
        front = np.r_[ranges[350:360], ranges[0:10]]
        left  = ranges[80:100]  
        right = ranges[260:280]  

        front_dist = np.mean(front)
        left_dist  = np.mean(left)
        right_dist = np.mean(right)

        safe_dist = 0.5 

        if front_dist < safe_dist:
            if left_dist > right_dist:
                action = "turn_left"
            else:
                action = "turn_right"
        else:
            action = "go_forward"

        print(f"Front: {front_dist:.2f}m -> Action: {action}")

        move_turtle(action)

    except IndexError:
        print("Error: 데이터 개수가 부족합니다.")

try:
    print(f"Connecting to ROS at {ROS_HOST}:{ROS_PORT}...")
    client = roslibpy.Ros(host=ROS_HOST, port=ROS_PORT)
    client.run()

    if client.is_connected:
        print("Connected! Ready to control Turtlesim.")
        
        listener = roslibpy.Topic(client, '/scan', 'sensor_msgs/LaserScan')
        listener.subscribe(callback)

        velocity_publisher = roslibpy.Topic(client, '/turtle1/cmd_vel', 'geometry_msgs/Twist')

        while client.is_connected:
            time.sleep(1)
            
    else:
        print("Failed to connect.")

except Exception as e:
    print(f"Connection Error: {e}")
finally:
    if 'client' in locals() and client.is_connected:
        listener.unsubscribe()
        if velocity_publisher:
            velocity_publisher.unadvertise()
        client.terminate()
