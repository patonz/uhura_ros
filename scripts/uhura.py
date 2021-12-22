#!/usr/bin/env python

from __future__ import print_function

# .srv import

from uhura_ros.srv import SendStringData, SendStringDataResponse, SendPositionData, SendPositionDataResponse, SetupNetworkDevice, SetupNetworkDeviceResponse, TestBroadcastNetwork, TestBroadcastNetworkResponse
import rospy
import re
import sys
import sched
import time
import threading
from tool_manager import ToolManager
from message import Message
from datetime import datetime
from std_msgs.msg import String
from digi.xbee.devices import XBeeDevice
from uhura_ros.msg import Position, Vehicle, Coordinates


# type id_message id_device timestamp

TYPE_MESSAGE_POS = 'pos'
TYPE_MESSAGE_TEST_NET_MESH = 'mesh_test'
TYPE_MESSAGE_TEST = 'test'

START_TIMESTAMP =int(round(time.time()*1000))
current_date = datetime.now()
string_date_time = current_date.strftime("%d-%m-%Y_%H-%H%M-%S");
NAME_FILE = 'net-test_%s.txt' % string_date_time
TEST_N_PACKETS = 0
TEST_PAYLOAD = bytearray(256)
TEST_DELAY = 1
TEST_CURRENT_N_PACKETS = 0
schedule = sched.scheduler(time.time, time.sleep)

setupDone = False
baudrate = 9600
port = "ttyUSB0"
DEVICE_NAME = "bee_n"

device = None

current_message_id = 0
generic_msg_rcv_pub = rospy.Publisher(
    'message_received', String, queue_size=10)
position_msg_rcv_pub = rospy.Publisher(
    'position_message_received', Position, queue_size=10)


def sendBroadCastData(data):
    #print(sys.getsizeof(data))
    global device
    global current_message_id
    if setupDone is not True:
        print("call the setup service first, idiot")
        return False



    if  isinstance(data, bytearray):

        #print("snd %s %s" % (current_message_id,data.decode()))
        #log_to_file("snd %s %s" % (current_message_id, ToolManager().bytes_to_bitstring(data)))
        device.send_data_broadcast(data)
        current_message_id += 1
        return True
    
    if isinstance(data, Message):
        data.id = current_message_id
        print("snd %s %s" % (data.header_to_string(), data.payload_to_string()))
        log_to_file("snd %s %s" % (data.header_to_string(), data.payload_to_string()))

        encoded_payload= bytearray(data.header_to_string(), encoding='utf-8') + data.payload
        device.send_data_broadcast(encoded_payload)
        current_message_id += 1
        return True
    



def handle_send_string_data(req):
    #print("Returning [%s : %s]" % (req.type, req.data))
   
    print(req.data)
    dataByteArray = bytearray(ToolManager().bitstring_to_bytes(req.data))
    print(dataByteArray)
    print(len(dataByteArray))
    return SendStringDataResponse(sendBroadCastData(dataByteArray))


def handle_send_position_data(req):
    global current_message_id
    req.pos.id = current_message_id
    sendBroadCastData(encode_position_to_string(req.pos))
    return SendPositionDataResponse(True)


def setup(req): #todo false return on exce
    print("setup: %s" % req)

    if req.baudrate is not None:
        baudrate = req.baudrate
    else:
        print("baudrate not found")

    if req.port is not None:
        port = req.port
    else:
        print("port not found")

    if req.port is not None:
        global DEVICE_NAME
        DEVICE_NAME = req.device_name
    else:
        print('device_name not found')

    global device
    device = XBeeDevice(port, baudrate)
    global setupDone
    setupDone = True

    print("current setup is baudrate=%s, port=%s device_name=%s" %
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

    print("network mesh test started...")
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
        message = Message(TYPE_MESSAGE_TEST, 0, DEVICE_NAME, int(round(time.time()*1000)), TEST_PAYLOAD)
      
        sendBroadCastData(message)
        TEST_CURRENT_N_PACKETS = TEST_CURRENT_N_PACKETS+1
        schedule.enter(TEST_DELAY, 1, sendBroadCastDataSchedFun, (params,))


def handle_pos_message(message_array_string):
    print("handle pos called")
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
    print("handle mesh_test called")
    network_test(
        message_array_string[1], message_array_string[2], message_array_string[3], False)
    return


def handle_generic_test_message(message_array_string):
    print("handle generic called")
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

        device.open()

        def data_receive_callback(xbee_message):
            rssi = 0
            packet_dict = xbee_message.to_dict()
            print(packet_dict)

            print("rcv %s %s" % (rssi, ToolManager().bytes_to_bitstring(xbee_message.data)))
            #log_to_file("rcv %s %s" % (rssi, xbee_message.data.decode(errors='ignore').rstrip('\x00')))
            generic_msg_rcv_pub.publish(ToolManager().bytes_to_bitstring(xbee_message.data))
            message_array = parse_message(xbee_message.data.decode(errors='ignore'))

            type_message = message_array[0]
            handle_type_message(type_message, message_array)

        device.add_data_received_callback(data_receive_callback)

        print("Waiting for data...\n")
 
    finally:
        if device is not None and device.is_open():
            device.close()

        device.open()


def parse_message(message_string):

    splitted = re.split('\s', message_string)
    message = Message(type, id, source_id, source_timestamp, payload)


    return re.split("\s", message_string)


def encode_position_to_string(pos: Position):

    return '%s %s %s %s %s %s %s' % (TYPE_MESSAGE_POS, pos.vehicle.id, pos.coordinates.x, pos.coordinates.y, pos.coordinates.z, pos.timestamp, pos.id)


def encode_test_net_to_string(delay, n_packets, n_bytes):
    return '%s %s %s %s %s' % (TYPE_MESSAGE_TEST_NET_MESH, delay, n_packets, n_bytes, 1)

def log_to_file(data):

    with open(NAME_FILE, 'a') as f:
        print(data, file=f)

    return
def uhura_server():

    # rospy.resolve_name(name)
    rospy.init_node('uhuranode', anonymous=False)  # turn off anonymous

    node_name = rospy.get_name()
    print('node_name: %s ' % (node_name))
    rospy.Service('%s/send_string_data' % node_name,
                  SendStringData, handle_send_string_data)
    rospy.Service('%s/send_position_data' % node_name, SendPositionData,
                  handle_send_position_data)
    rospy.Service('%s/setup_network_device' %
                  node_name, SetupNetworkDevice, setup)
    rospy.Service('%s/test_broadcast_network' % node_name,
                  TestBroadcastNetwork, handle_network_test)

    print("Uhura started")

    tool_manager_instance = ToolManager()
    print(tool_manager_instance.serial_ports())
    rospy.spin()


if __name__ == "__main__":
    uhura_server()
