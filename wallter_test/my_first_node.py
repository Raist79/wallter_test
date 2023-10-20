#!/usr/bin/env python3
import rclpy
import serial
import time
from rclpy.node import Node
from std_msgs.msg import String
from wallter_interfaces.msg import AkkuStats
from threading import Lock
encodedString = ''
decodedString = ''
tmpTest = []
       
class MyNode(Node):
    def __init__(self):
        super().__init__("first_node")
        self.publisher_ = self.create_publisher(AkkuStats, 'topic', 10)
        self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("sers")
        self.i = 0.0
        self.serial_port = "/dev/ttyACM0"
        self.baud_rate = 115200
        
        self.get_logger().info(f"Connecting to port {self.serial_port} at {self.baud_rate}.")
        self.conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1.0)
        self.get_logger().info("connected to {self.conn}")
        
        

    def timer_callback(self):
        global tmpTest
        global encodedString
        global decodedString
#        print(self.conn.write('v \r\n'.encode()))#  b"v \n\r"))
        #self.conn.flush
        
        #while len(encodedString) < 1:
            #print('while')
        self.conn.write('v \r\n'.encode())#  b"v \n\r"))        
        encodedString = self.conn.readline().decode('Ascii')
        
        print(encodedString)
            
            # Wait until there is data waiting in the serial buffer
            
            #if self.conn.in_waiting > 0:
                #print('h')
                # Read data out of the buffer until a carraige return / new line is found

                # Print the contents of the serial data
        #decodedString = encodedString.decode("Ascii")
        #print(decodedString)
        tmpTest = encodedString.split()
        #print(tmpTest)
        
        #self.get_logger().info(self.conn.readline())  
        
        msg = AkkuStats()
        if len(tmpTest) > 0:
            msg.bus_voltage = round(float(tmpTest[0]),1)
            msg.load_voltage = round(float(tmpTest[1]),1)
            msg.shunt_voltage = round(float(tmpTest[2]),1)
            msg.current = round(float(tmpTest[3]),1)
        #print(tmpTest[4])
            tmpInt = float(tmpTest[4])
         #   print(tmpInt)
            
            tmpInt = float(tmpInt) 

            msg.power = int(tmpInt)
            self.publisher_.publish(msg)
            #print('h: ' + tmpTest[0])
        #self.get_logger().info('Publishing: "%s"' % msg.current)
        #self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


    
