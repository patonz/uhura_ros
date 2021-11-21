#!/usr/bin/env python

from __future__ import print_function

# .srv import
from uhura_ros.srv import SendStringData, SendStringDataResponse, SendPositionData, SendPositionDataResponse, SetupNetworkDevice, SetupNetworkDeviceResponse
import rospy
import re
from std_msgs.msg import String
from digi.xbee.devices import XBeeDevice
from uhura_ros.msg import Position, Vehicle, Coordinates


setupDone = False
baudrate = 9600
port = "ttyUSB0"
device_name = "bee_n"

device = None

current_message_id=0
generic_msg_rcv_pub = rospy.Publisher('message_received', String, queue_size=10)
position_msg_rcv_pub = rospy.Publisher('position_message_received', Position, queue_size=10)

def sendBroadCastData(data):
    print("snd %s" % ( data))
    global device
    if setupDone:
        device.send_data_broadcast(data)
        return True
    else:
        print("call the setup service first, idiot")
        return False


def handle_send_string_data(req):
    print("Returning [%s : %s]" % (req.type, req.data))
    
    return SendStringDataResponse(sendBroadCastData(req.data))


def handle_send_position_data(req):
    global current_message_id
    req.pos.id = current_message_id
    current_message_id+=1
    sendBroadCastData(encode_position_to_string(req.pos))
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

    global device
    device = XBeeDevice(port, baudrate)
    global setupDone
    setupDone = True
    print("current setup is baudrate=%s, port=%s" % (baudrate, port))
    start_receiving_data()
    return SetupNetworkDeviceResponse(True)


def start_receiving_data():
    global device
    try:
      
        device.open()

        def data_receive_callback(xbee_message):
            print("rcv %s" % (xbee_message.data.decode()))
            generic_msg_rcv_pub.publish( xbee_message.data.decode())
            message_array = parse_message(xbee_message.data.decode())
            if message_array[0] == '1':
                 pos = Position()

                 pos.vehicle = Vehicle()
                 pos.vehicle.id = message_array[1]

                 pos.coordinates = Coordinates()
                 pos.coordinates.x = message_array[2]
                 pos.coordinates.y = message_array[3]
                 pos.coordinates.z = message_array[4]
                 pos.timestamp = message_array[5]
                 pos.id = message_array[6]
                
                 v = Vehicle()
                 v.id = "555"

                 pos1 = Position()
                 pos1.id = "77"
                 position_msg_rcv_pub.publish(pos)
            else:
                 print('altra roba')




        device.add_data_received_callback(data_receive_callback)

        print("Waiting for data...\n")


    finally:
        if device is not None and device.is_open():
            device.close()

        device.open()


def parse_message(message_string):
    return re.split("\s", message_string)


def encode_position_to_string(pos: Position):

    return '1 %s %s %s %s %s %s' % (pos.vehicle.id, pos.coordinates.x, pos.coordinates.y, pos.coordinates.z, pos.timestamp, pos.id) 



def uhura_server():
    rospy.init_node('uhuranode')
    rospy.Service('/send_string_data', SendStringData, handle_send_string_data)
    rospy.Service('/send_position_data', SendPositionData,
                  handle_send_position_data)
    rospy.Service('/setup_network_device', SetupNetworkDevice, setup)

    print("Ready to send generic data data")
    rospy.spin()


if __name__ == "__main__":
    uhura_server()


