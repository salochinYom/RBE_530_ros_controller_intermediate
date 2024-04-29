#!/usr/bin/env python
import rospy
import socket
from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy


class intermediate:
    UDP_IP = "192.168.0.57" #needs to point at arduino
    UDP_PORT = 2390  #needs to be the same as the arduino

    SELF_IP = "192.168.0.100" #needs to be set to ones own IP address when running as to recieve telemetry


    #robot params
    min_steer_angle = 10
    max_steer_angle = 170

    min_motor_power = 0
    max_motor_power = 255

    controller_state = Joy()
    controller_state.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    controller_state.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    prev_controller_state = controller_state

    #initial robot states
    steer_angle = 90 #a number between 0 and 180 that is how hard we turn.
    motor_1 = 0
    motor_2 = 0
    loopNum = 0


    #inte

    def callback(self,data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

        #do network things
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
        sock.sendto(bytes(data.data, "utf-8"), (self.UDP_IP, self.UDP_PORT))

    def handle_controller_input(self, data):
        #this thing only updates when new information is present
        self.controller_state = data

        #rospy.loginfo(self.controller_state.axes[0])
        #rospy.loginfo(self.controller_state.buttons[4])

        #update output params
        self.handle_steer_angle()
        self.handle_motor_1()
        self.handle_motor_2()

        #pub to network
        #self.publish_network_packets()

    #steering on the left stick
    def handle_steer_angle(self):
        #calculate output angle
        self.steer_angle = int(self.controller_state.axes[0] * self.steer_diff + self.steer_mid)
        #rospy.loginfo(self.steer_angle)

    #motor 1 on the left bumper
    def handle_motor_1(self):
        self.motor_1 = int(self.controller_state.axes[2]* -1 * self.motor_diff + self.motor_mid)
        
    #motor 2 on the right bumper
    def handle_motor_2(self):
        self.motor_2 = int(self.controller_state.axes[5]* -1 * self.motor_diff + self.motor_mid)

    


    
    #publish the network things
    def publish_network_packets(self):
        #make the socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP


        if(self.loopNum == 0):
            #make the message for the steer angle
            str_msg = "STR=" + str(self.steer_angle)
            sock.sendto(bytes(str_msg, "utf-8"), (self.UDP_IP, self.UDP_PORT))
        elif(self.loopNum == 1):
            #make the motor 1 string
            str_msg = "MT1=" + str(self.motor_1)
            sock.sendto(bytes(str_msg, "utf-8"), (self.UDP_IP, self.UDP_PORT))
        elif(self.loopNum == 2):
            #publish motor two value
            str_msg = "MT2=" + str(self.motor_2)
            sock.sendto(bytes(str_msg, "utf-8"), (self.UDP_IP, self.UDP_PORT))

        if(self.loopNum >= 2):
            self.loopNum = 0
        else:
            self.loopNum = self.loopNum + 1

        
        rospy.loginfo(str_msg)


    def start(self):
        #calulate joystick mapping values
        self.steer_mid = (self.min_steer_angle + self.max_steer_angle)/2
        self.steer_diff = (self.max_steer_angle - self.min_steer_angle)/2
        
        self.motor_mid = (self.min_motor_power + self.max_motor_power)/2
        self.motor_diff =(self.max_motor_power - self.min_motor_power)/2

        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('jumping_jackel_controller_intermediate', anonymous=True)

        #sanity check
        #rospy.Subscriber("chatter", String, self.callback)

        #update the controller values
        rospy.Subscriber("/joy", Joy, self.handle_controller_input)

        #create a socket for recieving messages
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
        sock.bind((self.SELF_IP, self.UDP_PORT))

        #rospy.spin()

        while True:
            rospy.sleep(0.01)
            #data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
            #rospy.loginfo("received message: %s" % data)
            #rospy.loginfo(1)

            #publish the network messages.
            self.publish_network_packets()

if __name__ == '__main__':
    joystickIntermediate = intermediate()
    joystickIntermediate.start()