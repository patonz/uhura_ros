#!/usr/bin/env python

from __future__ import print_function

# .srv import

from uhura_ros.srv import SendStringData, SendStringDataResponse, SendPositionData, SendPositionDataResponse, SetupNetworkDevice, SetupNetworkDeviceResponse, TestBroadcastNetwork, TestBroadcastNetworkResponse
from uhura_ros.msg import Position, Vehicle, Coordinates

from tool_manager import ToolManager
from message import Message

from datetime import datetime
from std_msgs.msg import String
from digi.xbee.devices import XBeeDevice

import rospy
import re
import sys
import sched
import time
import threading


# type id_message id_device timestamp

TYPE_MESSAGE_POS = 'pos'
TYPE_MESSAGE_TEST_NET_MESH = 'mesh_test'
TYPE_MESSAGE_TEST = 'test'

toolManager = None


START_TIMESTAMP = int(round(time.time()*1000))
current_date = datetime.now()
string_date_time = current_date.strftime("%d-%m-%Y_%H-%H%M-%S")
NAME_FILE = 'net-test_%s.txt' % string_date_time
TEST_N_PACKETS = 0
TEST_PAYLOAD = bytearray(256)
TEST_DELAY = 1
TEST_CURRENT_N_PACKETS = 0
schedule = sched.scheduler(time.time, time.sleep)

debug_mode = None
setupDone = False
baudrate = 9600
port = None
DEVICE_NAME = "bee_n"

device = None

current_message_id = 0
uav_name = None
run_type = None
generic_msg_rcv_pub = None
position_msg_rcv_pub = None
list_names = ["uav1", "uav2", "uav3"]


def sendBroadCastData(data):
    # print(sys.getsizeof(data))
    global device
    global current_message_id
    if setupDone is not True:
        rospy.logerr("call the setup service first, idiot")
        return False

    if isinstance(data, bytearray):

        #print("snd %s %s" % (current_message_id,data.decode()))
        #log_to_file("snd %s %s" % (current_message_id, ToolManager().bytes_to_bitstring(data)))
        device.send_data_broadcast(data)
        current_message_id += 1
        return True

    if isinstance(data, Message):
        data.id = current_message_id
        rospy.loginfo("snd %s %s" %
                      (data.header_to_string(), data.payload_to_string()))
        log_to_file("snd %s %s" %
                    (data.header_to_string(), data.payload_to_string()))

        encoded_payload = bytearray(
            data.header_to_string(), encoding='utf-8') + data.payload
        device.send_data_broadcast(encoded_payload)
        current_message_id += 1
        return True


def handle_send_string_data(req):

    dataByteArray = bytearray(ToolManager().bitstring_to_bytes(req.data))

    # CHECK IF SIMULATION @tiz
    # TODO FIX IT
    global run_type, uav_name
    if run_type == "simulation":
        for uav in list_names:
            if uav == uav_name:
                continue
            tmp = rospy.Publisher('/%s%s/message_received' %
                                  (uav, '/uhuranode'), String, queue_size=10)
            tmp.publish(req.data)
        return SendStringDataResponse(True)
    ############################################################
    return SendStringDataResponse(sendBroadCastData(dataByteArray))


def handle_send_position_data(req):
    global current_message_id
    req.pos.id = current_message_id
    sendBroadCastData(encode_position_to_string(req.pos))
    return SendPositionDataResponse(True)


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

    global device, port
    for port_free in ToolManager().serial_ports():
        try:
            device = XBeeDevice(port_free, baudrate)
            device.open()
            port = port_free
            break
        except:
            continue

    global setupDone
    setupDone = True

    rospy.loginfo("current setup is baudrate=%s, port=%s device_name=%s" %
                  (baudrate, port, DEVICE_NAME))
    start_receiving_data()
    return SetupNetworkDeviceResponse(True)


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

    #bytesstring = bytearray('%s %s %s %s' % (TYPE_MESSAGE_TEST, DEVICE_NAME, TEST_CURRENT_N_PACKETS, int(round(time.time()*1000))), encoding='utf-8')

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
        # print("Sending data to %s >> %s..." % ("BROADCAST", TEST_PAYLOAD))
        message = Message(TYPE_MESSAGE_TEST, 0, DEVICE_NAME,
                          int(round(time.time()*1000)), TEST_PAYLOAD)

        sendBroadCastData(message)
        TEST_CURRENT_N_PACKETS = TEST_CURRENT_N_PACKETS+1
        schedule.enter(TEST_DELAY, 1, sendBroadCastDataSchedFun, (params,))


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


def start_receiving_data():
    global device
    try:

        # device.open()

        def data_receive_callback(xbee_message):
            rssi = 0
            packet_dict = xbee_message.to_dict()

            # print(packet_dict)  #debug mode

            # format: type rssi sender len payloadbits
            rospy.loginfo("rcv %s %s %s" %
                          (rssi, xbee_message.remote_device, len(xbee_message.data)))
            generic_msg_rcv_pub.publish(
                ToolManager().bytes_to_bitstring(xbee_message.data))

            log_to_file("rcv %s %s %s" % (
                rssi, xbee_message.remote_device, len(xbee_message.data)))
            #log_to_file("rcv %s %s" % (rssi, xbee_message.data.decode(errors='ignore').rstrip('\x00')))

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


def parse_message(message_string):

    splitted = re.split('\s', message_string)
    message = Message(type, id, source_id, source_timestamp, payload)

    return re.split("\s", message_string)


def encode_position_to_string(pos: Position):

    return '%s %s %s %s %s %s %s' % (TYPE_MESSAGE_POS, pos.vehicle.id, pos.coordinates.x, pos.coordinates.y, pos.coordinates.z, pos.timestamp, pos.id)


def encode_test_net_to_string(delay, n_packets, n_bytes):
    return '%s %s %s %s %s' % (TYPE_MESSAGE_TEST_NET_MESH, delay, n_packets, n_bytes, 1)


def log_to_file(data):

    try:
        with open(NAME_FILE, 'a') as f:
            print(data, file=f)
    except:
        rospy.logerr('err')
    return


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
    rospy.Service('%s/send_position_data' % (node_name),
                  SendPositionData, handle_send_position_data)
    rospy.Service('%s/setup_network_device' %
                  (node_name), SetupNetworkDevice, setup)
    rospy.Service('%s/test_broadcast_network' % (node_name),
                  TestBroadcastNetwork, handle_network_test)

    global generic_msg_rcv_pub, position_msg_rcv_pub
    generic_msg_rcv_pub = rospy.Publisher(
        '%s/message_received' % (node_name), String, queue_size=10)
    position_msg_rcv_pub = rospy.Publisher(
        '%s/position_message_received' % (node_name), Position, queue_size=10)

    rospy.loginfo("Uhura started")
    rospy.logdebug(ToolManager().serial_ports())
    rospy.spin()


if __name__ == "__main__":
    uhura_server()
