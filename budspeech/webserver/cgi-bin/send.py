#!/usr/bin/python
import cgi
import cgitb
import subprocess
import rospy
import roslib
import sys
sys.path.insert(0, '/home/simon/ROS/devel/lib/python2.7/dist-packages/') # MAYBE security risk TODO
from budspeech.msg import speech

if __name__ == '__main__':

    print("Content-Type: text/html\n\n")  # html markup follows

    print("""
    <html>
        <head>
            <title>Publish Speech</title>
        </head>
        <body>
            <h1>Write to FlatBUDDY below:</h1>
            <form action="send.py" method="get">
            <input type="text" name="speech" value="">
            <input type="submit" value="send">
            </form>
        </body>
    </html> """)
    cgitb.enable()

    def talker():
        pub = rospy.Publisher('speech', speech)
        message=speech()
        form = cgi.FieldStorage()
        try:
            message.text='hey buddy '+form['speech'].value
            message.confidence=1
            message.interface_id=0
            rospy.init_node('web_budspeech', anonymous=True)
            r = rospy.Rate(10) #10hz
            pub.publish(message)
        except:
            print('Please enter speech!')


    if __name__ == '__main__':
        try:
            talker()
        except rospy.ROSInterruptException: pass

