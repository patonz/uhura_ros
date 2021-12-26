#!/usr/bin/env python
"""Uhura module
"""
from __future__ import print_function

import re
import sched
import sys
import threading
import time
from datetime import datetime

import rospy
import serial
from digi.xbee.devices import XBeeDevice
from std_msgs.msg import String
from uhura_ros.msg import Coordinates, Position, Vehicle
# .srv import
from uhura_ros.srv import (SendBitsStringData, SendBitsStringDataResponse,
                           SendPositionData, SendPositionDataResponse,
                           SendStringData, SendStringDataResponse,
                           SetupNetworkDevice, SetupNetworkDeviceResponse,
                           TestBroadcastNetwork, TestBroadcastNetworkResponse)

from message import Message
from tool_manager import ToolManager

__author__ = "Patonz91"
__copyright__ = "Copyright (c) 2021 Leonardo Montecchiari"
__credits__ = ["Leonardo Patonz Montecchiari"]
__license__ = "MIT"
__version__ = "beta"
__maintainer__ = "Patonz91"
__email__ = "patonz91@gmail.com"
__status__ = "Dev"


# custom service message types
TYPE_MESSAGE_POS = 'pos'
TYPE_MESSAGE_TEST_NET_MESH = 'mesh_test'
TYPE_MESSAGE_TEST = 'test'
TYPE_MESSAGE_STRING = 'str'

# ToolManager global instance
toolManager = None

# file_log name setup
START_TIMESTAMP = int(round(time.time()*1000))
current_date = datetime.now()
string_date_time = current_date.strftime("%d-%m-%Y_%H-%H%M-%S")
NAME_FILE = 'net-test_%s.txt' % string_date_time


## test mesh setup params##
TEST_N_PACKETS = 0
TEST_PAYLOAD = bytearray(256)
TEST_DELAY = 1
TEST_CURRENT_N_PACKETS = 0
schedule = sched.scheduler(time.time, time.sleep)

## device setup params ##
debug_mode = None
setupDone = False
baudrate = 9600
port = None
DEVICE_NAME = "bee_n"

# XBeeDevice instance
device = None

# global sent messages counter
current_message_id = 0

# simulation vars @tiziano
uav_name = None
run_type = None
list_names = ["uav1", "uav2", "uav3"]

# topic names
generic_msg_rcv_pub = None
position_msg_rcv_pub = None
bits_string_msg_rcv_pub = None


def sendBroadCastData(data):
    """sendBroadCastData can accept Message or bytearray types

    Args:
        data (Message): Custom Message with headers and some helper method
        data (bytearray): row data

    Returns:
        bool: ack
    """
    global device
    global current_message_id
    if setupDone is not True:
        rospy.logerr("call the setup service first, idiot")
        return False

    if isinstance(data, bytearray):

        rospy.loginfo("snd %s %s" % (current_message_id,
                      ToolManager().bytes_to_bitstring(data)))
        toolManager.log_to_file("snd %s %s" % (
            current_message_id, ToolManager().bytes_to_bitstring(data)))
        device.send_data_broadcast(data)
        current_message_id += 1
        return True

    if isinstance(data, Message):
        data.id = current_message_id
        rospy.loginfo("snd %s %s" %
                      (data.header_to_string(), data.payload_to_string()))
        toolManager.log_to_file("snd %s %s" %
                                (data.header_to_string(), data.payload_to_string()))

        encoded_payload = bytearray(
            data.header_to_string(), encoding='utf-8') + data.payload
        device.send_data_broadcast(encoded_payload)
        current_message_id += 1
        return True


def handle_send_string_data(req):
    message = Message(TYPE_MESSAGE_STRING, current_message_id, DEVICE_NAME, int(round(time.time()*1000)), req.data)
    
    return SendStringDataResponse(sendBroadCastData(message))

# sen string service request, turn off the real send if on simulation.
def handle_send_bits_string_data(req):

    dataByteArray = bytearray(ToolManager().bitstring_to_bytes(req.data))

    # CHECK IF SIMULATION @tiz
    # TODO FIX IT
    global run_type, uav_name
    if run_type == "simulation":
        for uav in list_names:
            if uav == uav_name:
                continue
            tmp = rospy.Publisher('/%s%s/receive_bits_string_data' %
                                  (uav, '/uhuranode'), String, queue_size=10)
            tmp.publish(req.data)
        return SendStringDataResponse(True)
    ############################################################
    return SendBitsStringDataResponse(sendBroadCastData(dataByteArray))




# service request for position custom message
def handle_send_position_data(req):
    global current_message_id
    req.pos.id = current_message_id
    sendBroadCastData(encode_position_to_string(req.pos))
    return SendPositionDataResponse(True)


# setup service handler
def setup(req):  # todo false return on exce
    # CHECK IF SIMULATION
    global run_type
    if run_type == "simulation":
        return SetupNetworkDeviceResponse(True)
    ############################################################
    rospy.logdebug("setup: %s" % req)

    if req.device_name is not None:
        global DEVICE_NAME
        DEVICE_NAME = req.device_name
    else:
        rospy.logerr('device_name not found')

    # check free ports and test it for the xbee device, then bind
    global device, port
    rospy.logdebug("finding the Xbee Device port...")
    for port_free in ToolManager().serial_ports():
        try:
            rospy.logdebug("port %s open" % port_free)

            ser = serial.Serial(port_free, 9600, timeout=1)
            
            ser.close() #close any other old serial connection
            ser.open() 
            time.sleep(1)
            rospy.logdebug('sending ENTER char')
            ser.write(b'\r') # mock an ENTER action
            time.sleep(1)
            rospy.logdebug('sending B char')
            ser.write(b'B') # activate the B - Bypass Mode
            ser.close() #close for the Xbee normal usage
            time.sleep(1)
            device = XBeeDevice(port_free, baudrate)
            device.open()
            
            port = port_free

            rospy.logdebug("device found on %s" % port_free)
            break
        except:
            rospy.logdebug("port %s its not a Xbee device" % port_free)
            continue
    
    if port is None :
        rospy.logerr("Xbee Device Not Found")
        return SetupNetworkDeviceResponse(False)
    global setupDone
    setupDone = True

    rospy.loginfo("current setup is baudrate=%s, port=%s device_name=%s" %
                  (baudrate, port, DEVICE_NAME))
    start_receiving_data()
    return SetupNetworkDeviceResponse(True)

## test mesh functions ###
def thread_func(name):
    schedule.run()


def handle_network_test(req):
    network_test(req.delay, req.n_packets, req.n_bytes, req.to_mesh)
    return TestBroadcastNetworkResponse(True)


def network_test(delay, n_packets, n_bytes, to_mesh):

    if isinstance(delay, str):
        delay = float(delay)

    if isinstance(n_packets, str):
        n_packets = int(n_packets)

    if isinstance(n_bytes, str):
        n_bytes = int(n_bytes)

    if isinstance(to_mesh, str):
        to_mesh = bool(to_mesh)

    rospy.loginfo("network mesh test started...")
    global TEST_DELAY
    TEST_DELAY = delay

    global TEST_N_PACKETS
    TEST_N_PACKETS = n_packets

    global TEST_CURRENT_N_PACKETS
    TEST_CURRENT_N_PACKETS = 0

    global TEST_PAYLOAD
    TEST_PAYLOAD = bytearray(n_bytes)

    if to_mesh:
        sendBroadCastData(encode_test_net_to_string(
            TEST_DELAY, TEST_N_PACKETS, n_bytes))

    schedule.enter(TEST_DELAY, 1, sendBroadCastDataSchedFun, (schedule,))
    x = threading.Thread(target=thread_func, args=(1,))
    x.start()


def sendBroadCastDataSchedFun(params):

    global TEST_CURRENT_N_PACKETS
    if TEST_CURRENT_N_PACKETS < TEST_N_PACKETS:
        message = Message(TYPE_MESSAGE_TEST, 0, DEVICE_NAME,
                          int(round(time.time()*1000)), TEST_PAYLOAD)

        sendBroadCastData(message)
        TEST_CURRENT_N_PACKETS = TEST_CURRENT_N_PACKETS+1
        schedule.enter(TEST_DELAY, 1, sendBroadCastDataSchedFun, (params,))
#############################


### message type handlers #####
def handle_pos_message(message_array_string):
    rospy.logdebug("handle pos called")
    pos = Position()

    pos.vehicle = Vehicle()
    pos.vehicle.id = message_array_string[1]

    pos.coordinates = Coordinates()
    pos.coordinates.x = message_array_string[2]
    pos.coordinates.y = message_array_string[3]
    pos.coordinates.z = message_array_string[4]

    pos.timestamp = message_array_string[5]
    pos.id = message_array_string[6]

    position_msg_rcv_pub.publish(pos)
    return


def handle_mesh_test_message(message_array_string):
    rospy.logdebug("handle mesh_test called")
    network_test(
        message_array_string[1], message_array_string[2], message_array_string[3], False)
    return


def handle_generic_test_message(message_array_string):
    rospy.logdebug("handle generic called")
    return


def handle_unkown_message():
    pass


def handle_type_message(type_message, message_array_string):
    switcher = {
        TYPE_MESSAGE_POS: handle_pos_message,
        TYPE_MESSAGE_TEST: handle_generic_test_message,
        TYPE_MESSAGE_TEST_NET_MESH: handle_mesh_test_message
    }

    func = switcher.get(type_message, lambda: 'message type not found')
    func(message_array_string)
#######################


def start_receiving_data():
    """starts the xbee callback for data rcv

       will create a Message() if is a utf-8 **uhura** format string
          # format: type rssi sender len payloadbits
    """
    global device
    try:

        # device.open()

        def data_receive_callback(xbee_message):
            rssi = 0
            packet_dict = xbee_message.to_dict()

            rospy.logdebug(packet_dict)  # debug mode

            rospy.loginfo("rcv %s %s %s" %
                          (rssi, xbee_message.remote_device, len(xbee_message.data)))
            bits_string_msg_rcv_pub.publish(
                ToolManager().bytes_to_bitstring(xbee_message.data))

            toolManager.log_to_file("rcv %s %s %s" % (
                rssi, xbee_message.remote_device, len(xbee_message.data)))

            message_array = parse_message(
                xbee_message.data.decode(errors='ignore'))
            type_message = message_array[0]
            handle_type_message(type_message, message_array)

        device.add_data_received_callback(data_receive_callback)

        rospy.loginfo("Waiting for data...\n")

    finally:
        if device is not None and device.is_open():

            # device.close()
            pass

        # device.open()


## uhura message helper functions ##
def parse_message(message_string):

    splitted = re.split('\s', message_string)
    message = Message(type, id, source_id, source_timestamp, payload)

    return re.split("\s", message_string)


def encode_position_to_string(pos: Position):

    return '%s %s %s %s %s %s %s' % (TYPE_MESSAGE_POS, pos.vehicle.id, pos.coordinates.x, pos.coordinates.y, pos.coordinates.z, pos.timestamp, pos.id)


def encode_test_net_to_string(delay, n_packets, n_bytes):
    return '%s %s %s %s %s' % (TYPE_MESSAGE_TEST_NET_MESH, delay, n_packets, n_bytes, 1)
####################################


def uhura_server():

    rospy.init_node('uhuranode', anonymous=False)

    global uav_name, run_type, baudrate, debug_mode
    uav_name = rospy.get_param("~UAV_NAME")
    run_type = rospy.get_param("~RUN_TYPE")
    baudrate = rospy.get_param("~baudrate")
    debug_mode = rospy.get_param("~debug_mode")

    global toolManager
    toolManager = ToolManager()

    if debug_mode is True:
        toolManager.set_rospy_log_lvl(rospy.DEBUG)
    else:
        toolManager.set_rospy_log_lvl(rospy.INFO)

    node_name = rospy.get_name()
    rospy.loginfo('node_name: %s ' % (node_name))

    rospy.Service('%s/send_string_data' % (node_name),
                  SendStringData, handle_send_string_data)
    rospy.Service('%s/send_bits_string_data' % (node_name),
                  SendBitsStringData, handle_send_bits_string_data)
    rospy.Service('%s/send_position_data' % (node_name),
                  SendPositionData, handle_send_position_data)
    rospy.Service('%s/setup_network_device' %
                  (node_name), SetupNetworkDevice, setup)
    rospy.Service('%s/test_broadcast_network' % (node_name),
                  TestBroadcastNetwork, handle_network_test)

    global generic_msg_rcv_pub, position_msg_rcv_pub
    generic_msg_rcv_pub = rospy.Publisher(
        '%s/message_received' % (node_name), String, queue_size=10)
    bits_string_msg_rcv_pub = rospy.Publisher(
        '%s/receive_bits_string_data' % (node_name), String, queue_size=10)
    position_msg_rcv_pub = rospy.Publisher(
        '%s/position_message_received' % (node_name), Position, queue_size=10)

    rospy.loginfo("Uhura started")
    rospy.logdebug(ToolManager().serial_ports())
    rospy.spin()


if __name__ == "__main__":
    uhura_server()
