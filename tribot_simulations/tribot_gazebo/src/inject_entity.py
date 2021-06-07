#!/usr/bin python3
import sys
import os
import rclpy
import xacro
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity

def inject(xml: str):
    rclpy.init()

    
    node = rclpy.create_node('inject_node')
    
    client = node.create_client(SpawnEntity, 'spawn_entity')

    if not client.service_is_ready():
        node.get_logger().info("waiting for spawn_entity service...")
        client.wait_for_service()

    request = SpawnEntity.Request()
    request.xml = xml
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    
    if future.result() is not None:
        node.get_logger().info('response: %r' % future.result())
    else:
        raise RuntimeError('exception while calling service: %r' % future.exception())

    node.destroy_node()
    rclpy.shutdown()

if len(sys.argv) < 1:
    print('usage: ros2 run tribot_gazebo inject_entity.py -- foo.urdf')
    sys.exit(1)

xacro_path = os.path.join(get_package_share_directory('tribot_description'), 'urdf', 'tribot.urdf')

doc = xacro.parse(open(xacro_path))
xacro.process_doc(doc)
robot_description = doc.toxml()

inject(robot_description)
#inject(open(sys.argv[1], 'r').read())
