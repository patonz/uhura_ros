#!/usr/bin/env python

from __future__ import print_function

# .srv import
from uhura_ros.srv import SendStringData,SendStringDataResponse, SendPositionData, SendPositionDataResponse
import rospy

def handle_send_string_data(req):
    print("Returning [%s : %s]"%(req.type, req.data))
    return SendStringDataResponse(True)

def handle_send_position_data(req):
    print("Returning [%s]"%(req.pos))
    return SendPositionDataResponse(True)

def uhura_server():
    rospy.init_node('uhuranode')
    s = rospy.Service('/send_string_data', SendStringData, handle_send_string_data)
    rospy.Service('/send_position_data', SendPositionData, handle_send_position_data)
    print("Ready to send generic data data")
    rospy.spin()

if __name__ == "__main__":
    uhura_server()
