#!/usr/bin/env python
import roslib; roslib.load_manifest('ur_driver')
import time, sys, threading, math
import copy
import datetime
import socket, select
import struct
import traceback, code
import SocketServer

import rospy

from sensor_msgs.msg import JointState
from ur_driver.deserialize import SecondaryClientPacket, RobotState, RobotMessage
from ur_msgs.msg import *

# renaming classes
DigitalIn = Digital
DigitalOut = Digital
Flag  = Digital

MULT_analog = 1000000.0
MULT_analog_robotstate = 0.1

joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
joint_offsets = {}


def __on_packet(buf):
    scp = SecondaryClientPacket.unpack(buf)
    
    if(scp.robot_message == None):
        print("RobotMessage is None")
        #but this is ok as it is only sent once as first packet
    
    if(scp.robot_state == None):
        print("RobotState is None")
        return
    
    
    state = scp.robot_state
    
    mb_msg = MasterboardDataMsg()
    mb_msg.digital_input_bits = state.masterboard_data.digital_input_bits
    mb_msg.digital_output_bits = state.masterboard_data.digital_output_bits
    mb_msg.analog_input_range0 = state.masterboard_data.analog_input_range0
    mb_msg.analog_input_range1 = state.masterboard_data.analog_input_range1
    mb_msg.analog_input0 = state.masterboard_data.analog_input0
    mb_msg.analog_input1 = state.masterboard_data.analog_input1
    mb_msg.analog_output_domain0 = state.masterboard_data.analog_output_domain0
    mb_msg.analog_output_domain1 = state.masterboard_data.analog_output_domain1
    mb_msg.analog_output0 = state.masterboard_data.analog_output0
    mb_msg.analog_output1 = state.masterboard_data.analog_output1
    mb_msg.masterboard_temperature = state.masterboard_data.masterboard_temperature
    mb_msg.robot_voltage_48V = state.masterboard_data.robot_voltage_48V
    mb_msg.robot_current = state.masterboard_data.robot_current
    mb_msg.master_io_current = state.masterboard_data.master_io_current
    #mb_msg.master_safety_state = state.masterboard_data.master_safety_state    #not available in V30
    #mb_msg.master_onoff_state = state.masterboard_data.master_onoff_state      #not available in V30
    pub_masterboard_state.publish(mb_msg)
    
    # Use information from the robot state packet to publish IOStates        
    msg = IOStates()
    #gets digital in states
    for i in range(0, 10):
        msg.digital_in_states.append(DigitalIn(i, (state.masterboard_data.digital_input_bits & (1<<i))>>i))
    #gets digital out states
    for i in range(0, 10):
        msg.digital_out_states.append(DigitalOut(i, (state.masterboard_data.digital_output_bits & (1<<i))>>i))
    #gets analog_in[0] state
    inp = state.masterboard_data.analog_input0 / MULT_analog_robotstate
    msg.analog_in_states.append(Analog(0, inp))
    #gets analog_in[1] state
    inp = state.masterboard_data.analog_input1 / MULT_analog_robotstate
    msg.analog_in_states.append(Analog(1, inp))      
    #gets analog_out[0] state
    inp = state.masterboard_data.analog_output0 / MULT_analog_robotstate
    msg.analog_out_states.append(Analog(0, inp))     
    #gets analog_out[1] state
    inp = state.masterboard_data.analog_output1 / MULT_analog_robotstate
    msg.analog_out_states.append(Analog(1, inp))     
    #print "Publish IO-Data from robot state data"
    pub_io_states.publish(msg)
    
    # Notes: 
    # - Where are the flags coming from? Do we need flags? No, as 'prog' does not use them and other scripts are not running!
    # - analog_input2 and analog_input3 are within ToolData
    # - What to do with the different analog_input/output_range/domain?
    # - Shall we have appropriate ur_msgs definitions in order to reflect MasterboardData, ToolData,...?





def main():
    rospy.init_node('test_comm', disable_signals=True)
    
    global pub_masterboard_state
    pub_masterboard_state = rospy.Publisher('masterboard_state', MasterboardDataMsg, queue_size=1)
    global pub_io_states
    pub_io_states = rospy.Publisher('io_states', IOStates, queue_size=1)
    
    
    robot_hostname = '192.168.110.129'
    port = 30002
    
    test_socket = socket.create_connection((robot_hostname, port))
    buf = ""
    
    while not rospy.is_shutdown():
        more = test_socket.recv(4096)
        if more:
            buf = buf + more

            # Attempts to extract a packet
            packet_length = struct.unpack_from("!i", buf)[0]
            print("PacketLength: ", packet_length, "; BufferSize: ", len(buf))
            if len(buf) >= packet_length:
                packet, buf = buf[:packet_length], buf[packet_length:]
                __on_packet(packet)
        else:
            print("There is no more...")
    
    test_socket.close()




if __name__ == '__main__': main()
