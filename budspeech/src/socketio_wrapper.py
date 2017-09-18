#!/usr/bin/env python
# Socket io ROS wrapper
from socketIO_client import SocketIO, LoggingNamespace
import rospy
import time
from budspeech.msg import speech
from std_msgs.msg import String

def on_publish_test():
        message = speech()
        message.text=str(msg)
        message.confidence=1
        message.interface_id=2
        self.pub.publish(message)

class socketio_wrapper_node(object):

    def __init__(self):
        self.socketIO = SocketIO('localhost', 3000, LoggingNamespace)
        rospy.init_node('socketio_wrapper')
        self.socketIO.emit('it_works','yeah')
        # Init Ros Publisher
        self.pub = rospy.Publisher('speech', speech,queue_size=100)
        # Init Ros Subcriber
        rospy.Subscriber("budrep", String, self.process_response)
        self.socketIO.on('user_demand', self.on_publish)

    def spin(self):
        self.socketIO.wait()

    def process_response(self,data):
        self.socketIO.emit('resp',data.data)

    def on_publish(self,*msg):
        message = speech()
        message.text=str('buddy '+msg[0])
        message.confidence=1
        message.interface_id=3
        self.pub.publish(message)

if __name__ == '__main__':
    wrap=socketio_wrapper_node()
    wrap.spin()
