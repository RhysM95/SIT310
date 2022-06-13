from drone_interfaces.srv import Move, MPad
from std_srvs.srv import Empty

import rclpy
import time

def Initilize(node):
    ##Battery##

    cli = node.create_client(Empty, 'battery')
    req = Empty.Request()
    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    try:
        result = future.result()
    except Exception as e:
        node.get_logger().info('Service call failed %r' % (e,))
    else:
        node.get_logger().info("Battery status is shown")

    time.sleep(5)
    
    ##Mon##
    
    cli = node.create_client(Empty, 'mon')
    req = Empty.Request()
    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    try:
        result = future.result()
    except Exception as e:
        node.get_logger().info('Service call failed %r' % (e,))
    else:
        node.get_logger().info("Mission pad On")
    
    time.sleep(2)
    
    ##mdirection##
    
    cli = node.create_client(Empty, 'mdirection')
    req = Empty.Request()
    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    try:
        result = future.result()
    except Exception as e:
        node.get_logger().info('Service call failed %r' % (e,))
    else:
        node.get_logger().info("Mission pad detection set")

    time.sleep(5) 
    
def Take_off(node):
    cli = node.create_client(Empty, 'takeoff')
    req = Empty.Request()
    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    try:
        result = future.result()
    except Exception as e:
        node.get_logger().info('Service call failed %r' % (e,))
    else:
        node.get_logger().info("takeoff successful")

    time.sleep(5)

def Mission_Pad_Jump(node, x, y, z, speed, rot, mpad_s, mpad_f):
    cli = node.create_client(MPad, 'mid_jump')
    req = MPad.Request()
    req.x = x
    req.y = y
    req.z = z
    req.speed = speed
    req.rot = rot
    req.pad1 = mpad_s
    req.pad2 = mpad_f
    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    try:
        result = future.result()
    except Exception as e:
        node.get_logger().info('Service call failed %r' % (e,))
    else:
        node.get_logger().info("Move to new Mission pad successful")

    time.sleep(10)   
	
def Mission_Pad_Go(node, x, y, z, speed, mpad):
    cli = node.create_client(MPad, 'mid_go')
    req = MPad.Request()
    req.x = x
    req.y = y
    req.z = z
    req.speed = speed
    req.pad1 = mpad
    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    try:
        result = future.result()
    except Exception as e:
        node.get_logger().info('Service call failed %r' % (e,))
    else:
        node.get_logger().info("Move on current Mission pad successful")

    time.sleep(2)
    
def Mission_Pad_Curve(node, x1, y1, z1, x2, y2, z2, speed, mpad):
    cli = node.create_client(MPad, 'mid_curve')
    req = MPad.Request()
    req.x = x1
    req.y = y1
    req.z = z1
    req.x2 = x2
    req.y2 = y2
    req.z2= z2
    req.speed = speed
    req.pad1 = mpad
    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    try:
        result = future.result()
    except Exception as e:
        node.get_logger().info('Service call failed %r' % (e,))
    else:
        node.get_logger().info("Curve to new Mission pad successful")

    time.sleep(2)
    
def Backflip(node):
    cli = node.create_client(Empty, 'flip_backward')
    req = Empty.Request()
    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    try:
        result = future.result()
    except Exception as e:
        node.get_logger().info('Service call failed %r' % (e,))
    else:
        node.get_logger().info("Flip backward successful")

    time.sleep(5)
    
def Land(node):
    cli = node.create_client(Empty, 'land')
    req = Empty.Request()
    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    try:
        result = future.result()
    except Exception as e:
        node.get_logger().info('Service call failed %r' % (e,))
    else:
        node.get_logger().info("land successful")
    
         
def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('drone_service_client')
    
    Initilize(node)
    Take_off(node)
    Mission_Pad_Jump(node, 80, 3, 50, 25, 180, 'm1', 'm2')
    Mission_Pad_Go(node, 0, 10, 60, 80, 'm2')     
    Mission_Pad_Go(node, 0, -10, 60, 80, 'm2')     
    Mission_Pad_Go(node, 0, 10, 60, 80, 'm2')     
    Mission_Pad_Go(node, 0, -10, 60, 80, 'm2')
    Mission_Pad_Jump(node, -80, 3, 120, 40, 180, 'm2', 'm1')
    Backflip(node)
    Land(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

