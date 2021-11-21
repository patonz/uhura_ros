#!/usr/bin/env python

from __future__ import print_function

# .srv import
from uhura_ros.srv import SendStringData, SendStringDataResponse, SendPositionData, SendPositionDataResponse, SetupNetworkDevice, SetupNetworkDeviceResponse
import rospy
from std_msgs.msg import String
from digi.xbee.devices import XBeeDevice


setupDone = False
baudrate = 9600
port = "ttyUSB0"
device_name = "bee_n"

global device


def sendBroadCastData(data):
    print("Sending data to %s >> %s..." % ("BROADCAST", DATA_TO_SEND))
    device.send_data_broadcast(data)


def handle_send_string_data(req):
    print("Returning [%s : %s]" % (req.type, req.data))
    return SendStringDataResponse(True)


def handle_send_position_data(req):
    print("Returning [%s]" % (req))
    sendBroadCastData(req.pos.x)
    return SendPositionDataResponse(True)


def setup(req):
    print("setup: %s" % req)

    if req.baudrate is not None:
        baudrate = req.baudrate
    else:
        print("baudrate not found")

    if req.port is not None:
        port = req.port
    else:
        print("port not found")
    #if setupDone is False:
    global device
    device = XBeeDevice(port, baudrate)
   
    setupDone = True
    print("current setup is baudrate=%s, port=%s" % (baudrate, port))
    start_receiving_data()
    return SetupNetworkDeviceResponse(True)


def start_receiving_data():
    try:
        device.open()

        def data_receive_callback(xbee_message):
            print("From %s >> %s" % (xbee_message.remote_device.get_node_id(),
                                     xbee_message.data.decode()))

        device.add_data_received_callback(data_receive_callback)

        print("Waiting for data...\n")


    finally:
        if device is not None and device.is_open():
            device.close()

        device.open()


def uhura_server():
    rospy.init_node('uhuranode')
    pub = rospy.Publisher('received_message', String, queue_size=10)
    rospy.Service('/send_string_data', SendStringData, handle_send_string_data)
    rospy.Service('/send_position_data', SendPositionData,
                  handle_send_position_data)
    rospy.Service('/setup_network_device', SetupNetworkDevice, setup)

    print("Ready to send generic data data")
    rospy.spin()


if __name__ == "__main__":
    uhura_server()
