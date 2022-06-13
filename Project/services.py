import socket
import threading
import time

import rclpy
from rclpy.node import Node

from drone_interfaces.srv import Move, MPad
from std_srvs.srv import Empty

class DroneServer(Node):
    drone_response = "no_response"

    def __init__(self):
        super().__init__('drone_server')

        self.local_ip = ''
        self.local_port = 8889
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # socket for sending cmd
        self.socket.bind((self.local_ip, self.local_port))

        # thread for receiving cmd ack
        self.receive_thread = threading.Thread(target=self._receive_thread)
        self.receive_thread.daemon = True
        self.receive_thread.start()
        
        self.socket2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # socket for mission pads
        self.socket2.bind(('', 8890))

        # thread for receiving mission pad ack
        self.receive_thread2 = threading.Thread(target=self._receive_thread_state)
        self.receive_thread2.daemon = True
        self.receive_thread2.start()

        self.tello_ip = '192.168.10.1'
        self.tello_port = 8889
        self.tello_address = (self.tello_ip, self.tello_port)
        self.MAX_TIME_OUT = 15.0


        self.srv = self.create_service(Move, 'move_forward', self.move_forward_callback)
        self.srv = self.create_service(Move, 'move_backward', self.move_backward_callback)

        self.srv = self.create_service(Move, 'move_left', self.move_left_callback)
        self.srv = self.create_service(Move, 'move_right', self.move_right_callback)

        self.srv = self.create_service(Move, 'move_up', self.move_up_callback)
        self.srv = self.create_service(Move, 'move_down', self.move_down_callback)


        self.srv = self.create_service(Empty, 'flip_forward', self.flip_forward_callback)
        self.srv = self.create_service(Empty, 'flip_backward', self.flip_backward_callback)

        self.srv = self.create_service(Empty, 'takeoff', self.takeoff_callback)
        self.srv = self.create_service(Empty, 'land', self.land_callback)

        self.srv = self.create_service(Empty, 'battery', self.battery_callback)
        
        self.srv = self.create_service(Empty, 'mon', self.mon_callback)
        self.srv = self.create_service(Empty, 'mdirection', self.mdirection_callback)
        self.srv = self.create_service(MPad, 'mid_go', self.mid_go_callback)
        self.srv = self.create_service(MPad, 'mid_jump', self.mid_jump_callback)
        self.srv = self.create_service(MPad, 'mid_curve', self.mid_curve_callback)


    def send_command(self, msg):
        command = msg #the actual command string

        self.get_logger().info('I heard: "%s"' % msg)
        self.socket.sendto(command.encode('utf-8'), self.tello_address)
        print('sending command: %s to %s' % (command, self.tello_ip))

        start = time.time()
        now = time.time()
        diff = now - start
        if diff > self.MAX_TIME_OUT:
            print('Max timeout exceeded... command %s' % command)
            return
        print('Done!!! sent command: %s to %s' % (command, self.tello_ip))


    def move_forward_callback(self, request, response):
        global drone_response
        self.get_logger().info('Incoming request: Move Forward: %dcm' % (request.distance))
        command = "forward %d" % request.distance
        print(command)
        self.send_command("command")
        time.sleep(1)
        self.send_command(command)
        time.sleep(1) #wait for the response
        response.result = drone_response
        return response

    def move_backward_callback(self, request, response):
        global drone_response
        self.get_logger().info('Incoming request: Move Backward: %dcm' % (request.distance))
        command = "back %d" % request.distance
        print(command)
        self.send_command("command")
        time.sleep(1)
        self.send_command(command)
        time.sleep(1) #wait for the response
        response.result = drone_response
        return response

    def move_left_callback(self, request, response):
        global drone_response
        self.get_logger().info('Incoming request: Move Left: %dcm' % (request.distance))
        command = "left %d" % request.distance
        print(command)
        self.send_command("command")
        time.sleep(1)
        self.send_command(command)
        time.sleep(1) #wait for the response
        response.result = drone_response
        return response

    def move_right_callback(self, request, response):
        global drone_response
        self.get_logger().info('Incoming request: Move Right: %dcm' % (request.distance))
        command = "right %d" % request.distance
        print(command)
        self.send_command("command")
        time.sleep(1)
        self.send_command(command)
        time.sleep(1) #wait for the response
        response.result = drone_response
        return response

    def move_up_callback(self, request, response):
        global drone_response
        self.get_logger().info('Incoming request: Move Up: %dcm' % (request.distance))
        command = "up %d" % request.distance
        print(command)
        self.send_command("command")
        time.sleep(1)
        self.send_command(command)
        time.sleep(1) #wait for the response
        response.result = drone_response
        return response

    def move_down_callback(self, request, response):
        global drone_response
        self.get_logger().info('Incoming request: Move Down: %dcm' % (request.distance))
        command = "down %d" % request.distance
        print(command)
        self.send_command("command")
        time.sleep(1)
        self.send_command(command)
        time.sleep(1) #wait for the response
        response.result = drone_response
        return response

    def flip_forward_callback(self, request, response):
        self.get_logger().info('Incoming request: flip forward')
        command = "flip f"
        print(command)
        self.send_command(command)
        return response

    def flip_backward_callback(self, request, response):
        self.get_logger().info('Incoming request: flip backward')
        command = "flip b"
        print(command)
        self.send_command(command)
        return response


    def takeoff_callback(self, request, response):
        self.get_logger().info('Incoming request: Takeoff')
        command = "takeoff"
        print(command)
        self.send_command("command")
        time.sleep(2)
        self.send_command(command)
        return response

    def land_callback(self, request, response):
        self.get_logger().info('Incoming request: Land')
        command = "land"
        print(command)
        self.send_command("command")
        time.sleep(2)
        self.send_command(command)
        return response

    def battery_callback(self, request, response):
        global drone_response
        self.get_logger().info('Incoming request: Battery')
        command = "battery?"
        print(command)
        self.send_command("command")
        time.sleep(1)
        self.send_command(command)
        time.sleep(1)
        print("Battery is : %s" % self.response.decode('utf-8'))
        return response
        
    def mon_callback(self, request, response):
        self.get_logger().info('Activate Mission Pads')
        command = "mon"
        print(command)
        self.send_command("command")
        time.sleep(2)
        self.send_command(command)
        return response
        
    def mdirection_callback(self, request, response):
        self.get_logger().info('Detect Mission Pads below')
        command = "mdirection 2"
        print(command)
        self.send_command("command")
        time.sleep(2)
        self.send_command(command)
        return response
        
    def mid_go_callback(self, request, response):
        global drone_response
        self.get_logger().info('Incoming request: Move on Current Mission Pad: %s' % (request.pad1))
        command = "go %d %d %d %d %s" % (request.x, request.y, request.z, request.speed, request.pad1)
        print(command)
        self.send_command("command")
        time.sleep(1)
        self.send_command(command)
        time.sleep(1) #wait for the response
        #response.result = drone_response
        return response   
        
    def mid_jump_callback(self, request, response):
        global drone_response
        self.get_logger().info('Incoming request: Move to Mission Pad: %s' % (request.pad2))
        command = "jump %d %d %d %d %d %s %s" % (request.x, request.y, request.z, request.speed, request.rot, request.pad1, request.pad2)
        print(command)
        self.send_command("command")
        time.sleep(1)
        self.send_command(command)
        time.sleep(1) #wait for the response
        #response.result = drone_response
        return response

    def mid_curve_callback(self, request, response):
        global drone_response
        self.get_logger().info('Incoming request: Curve to Mission Pad: %s' % (request.pad1))
        command = "curve %d %d %d %d %d %d %d %s" % (request.x, request.y, request.z, request.x2, request.y2, request.z2, request.speed, request.pad1)
        print(command)
        self.send_command("command")
        time.sleep(1)
        self.send_command(command)
        time.sleep(1) #wait for the response
        #response.result = drone_response
        return response  
                        
    def _receive_thread(self):
        global drone_response
        #Listen to responses from the Tello.
        while True:
            try:
                self.response, ip = self.socket.recvfrom(1024)
                print('from %s: %s' % (ip, self.response))
                drone_response = str(self.response) #convert from byte string to string
            except (socket.error, exc):
                print("Caught exception socket.error : %s" % exc)
                
    def _receive_thread_state(self):
        #Listen to responses from the Tello.
        while True:
            try:
                self.response, ip = self.socket2.recvfrom(1024)
                print('from %s: %s' % (ip, self.response))
            except (socket.error, exc):
                print("Caught exception socket.error : %s" % exc)



def main(args=None):
    rclpy.init(args=args)

    node = DroneServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - Done automatically when node is garbage collected)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

