#!/usr/bin/env python
import rospy
import socket
from std_msgs.msg import String


UDP_IP = "192.168.168.207" #needs to point at arduino
UDP_PORT = 2390  #needs to be the same as the arduino

SELF_IP = "192.168.168.47" #needs to be set to ones own IP address when running as to recieve telemetry

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    #do network things
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
    sock.sendto(bytes(data.data, "utf-8"), (UDP_IP, UDP_PORT))

#map the steering to the left right of the left stick


    
def start():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", String, callback)

    #create a socket for recieving messages
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
    sock.bind((SELF_IP, UDP_PORT))

    while True:
        rospy.sleep(0.01)
        data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
        rospy.loginfo("received message: %s" % data)
        #rospy.loginfo(1)

if __name__ == '__main__':
    start()