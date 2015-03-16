import struct

class RobotStatePackageType(object):
    ROBOT_MODE_DATA = 0
    JOINT_DATA = 1
    TOOL_DATA = 2
    MASTERBOARD_DATA = 3
    CARTESIAN_INFO = 4
    KINEMATICS_INFO = 5
    CONFIGURATION_DATA = 6
    FORCE_MODE_DATA = 7
    ADDITIONAL_INFO = 8
    CALIBRATION_DATA = 9
    
class RobotMessagePackageType(object):
    ROBOT_MESSAGE_TEXT = 0
    ROBOT_MESSAGE_PROGRAM_LABEL = 1
    ROBOT_MESSAGE_VERSION = 3
    ROBOT_MESSAGE_SAFETY_MODE = 5
    ROBOT_MESSAGE_ERROR_CODE = 6
    ROBOT_MESSAGE_KEY = 7
    ROBOT_MESSAGE_REQUEST_VALUE = 9
    ROBOT_MESSAGE_RUNTIME_EXCEPTION = 10

#valid until revision11
class RobotMode_V18(object):
    ROBOT_RUNNING_MODE = 0
    ROBOT_FREEDRIVE_MODE = 1
    ROBOT_READY_MODE = 2
    ROBOT_INITIALIZING_MODE = 3
    ROBOT_SECURITY_STOPPED_MODE = 4
    ROBOT_EMERGENCY_STOPPED_MODE = 5
    ROBOT_FATAL_ERROR_MODE = 6
    ROBOT_NO_POWER_MODE = 7
    ROBOT_NOT_CONNECTED_MODE = 8
    ROBOT_SHUTDOWN_MODE = 9
    ROBOT_SAFEGUARD_STOP_MODE = 10

#valid since revision12
class RobotMode_V30(object):
    ROBOT_MODE_DISCONNECTED = 0
    ROBOT_MODE_CONFIRM_SAFETY = 1
    ROBOT_MODE_BOOTING = 2
    ROBOT_MODE_POWER_OFF = 3
    ROBOT_MODE_POWER_ON = 4
    ROBOT_MODE_IDLE = 5
    ROBOT_MODE_BACKDRIVE = 6
    ROBOT_MODE_RUNNING = 7
    ROBOT_MODE_UPDATING_FIRMWARE = 8

class JointMode(object):
    PART_D_CALIBRATION = 237
    BACKDRIVE = 238
    POWER_OFF = 239
    EMERGENCY_STOPPED = 240
    CALVAL_INITIALIZATION = 241
    ERROR = 242
    FREEDRIVE = 243
    SIMULATED = 244
    NOT_RESPONDING = 245
    MOTOR_INITIALISATION = 246
    ADC_CALIBRATION = 247
    DEAD_COMMUTATION = 248
    BOOTLOADER = 249
    CALIBRATION = 250
    STOPPED = 251
    FAULT = 252
    RUNNING = 253
    INITIALIZATION = 254
    IDLE = 255

class ToolMode(object):
    BOOTLOADER = 249
    RUNNING = 253
    IDLE = 255

class MasterSafetyState(object):
    UNDEFINED = 0
    BOOTLOADER = 1
    FAULT = 2
    BOOTING = 3
    INITIALIZING = 4
    ROBOT_EMERGENCY_STOP = 5
    EXTERNAL_EMERGENCY_STOP = 6
    SAFEGUARD_STOP = 7
    OK = 8

class MasterOnOffState(object):
    OFF = 0
    TURNING_ON = 1
    ON = 2
    TURNING_OFF = 3

#this class handles different protocol versions
class RobotModeData(object):
    @staticmethod
    def unpack(buf):
        rmd = RobotModeData()
        (plen, ptype) = struct.unpack_from("!IB", buf)
        if plen == 29:
            return RobotModeData_V18.unpack(buf)
        elif plen == 38:
            return RobotModeData_V30.unpack(buf)
        else:
            print "RobotModeData has wrong length: " + str(plen)
            return rmd

#this parses RobotModeData for versions <= v1.8 (i.e. 1.6, 1.7, 1.8)
class RobotModeData_V18(object):
    __slots__ = ['timestamp', 'robot_connected', 'real_robot_enabled',
                 'power_on_robot', 'emergency_stopped',
                 'security_stopped', 'program_running', 'program_paused',
                 'robot_mode', 'speed_fraction']
    @staticmethod
    def unpack(buf):
        rmd = RobotModeData_V18()
        (_, _,
         rmd.timestamp, rmd.robot_connected, rmd.real_robot_enabled,
         rmd.power_on_robot, rmd.emergency_stopped, rmd.security_stopped,
         rmd.program_running, rmd.program_paused, rmd.robot_mode,
         rmd.speed_fraction) = struct.unpack_from("!IBQ???????Bd", buf)
        return rmd

#this parses RobotModeData for versions >=3.0 (i.e. 3.0)
class RobotModeData_V30(object):
    __slots__ = ['timestamp', 'robot_connected', 'real_robot_enabled',
                 'power_on_robot', 'emergency_stopped',
                 'security_stopped', 'program_running', 'program_paused',
                 'robot_mode', 'control_mode', 'target_speed_fraction',
                 'speed_scaling']
    @staticmethod
    def unpack(buf):
        rmd = RobotModeData_V30()
        (_, _,
         rmd.timestamp, rmd.robot_connected, rmd.real_robot_enabled,
         rmd.power_on_robot, rmd.emergency_stopped, rmd.security_stopped,
         rmd.program_running, rmd.program_paused, rmd.robot_mode, rmd.control_mode,
         rmd.target_speed_fraction, rmd.speed_scaling) = struct.unpack_from("!IBQ???????BBdd", buf)
        return rmd

#this parses JointData for all versions (i.e. 1.6, 1.7, 1.8, 3.0)
class JointData(object):
    __slots__ = ['q_actual', 'q_target', 'qd_actual',
                 'I_actual', 'V_actual', 'T_motor', 'T_micro', 'joint_mode']
    @staticmethod
    def unpack(buf):
        all_joints = []
        offset = 5
        for i in range(6):
            jd = JointData()
            (jd.q_actual, jd.q_target, jd.qd_actual, jd.I_actual, jd.V_actual,
             jd.T_motor, jd.T_micro,
             jd.joint_mode) = struct.unpack_from("!dddffffB", buf, offset)
            offset += 41
            all_joints.append(jd)
        return all_joints

#this parses JointData for all versions (i.e. 1.6, 1.7, 1.8, 3.0)
class ToolData(object):
    __slots__ = ['analog_input_range2', 'analog_input_range3',
                 'analog_input2', 'analog_input3',
                 'tool_voltage_48V', 'tool_output_voltage', 'tool_current',
                 'tool_temperature', 'tool_mode']
    @staticmethod
    def unpack(buf):
        td = ToolData()
        (_, _,
         td.analog_input_range2, td.analog_input_range3,
         td.analog_input2, td.analog_input3,
         td.tool_voltage_48V, td.tool_output_voltage, td.tool_current,
         td.tool_temperature, td.tool_mode) = struct.unpack_from("!IBbbddfBffB", buf)
        return td

#this class handles different protocol versions
class MasterboardData(object):
    @staticmethod
    def unpack(buf):
        md = MasterboardData()
        (plen, ptype) = struct.unpack_from("!IB", buf)
        if (plen == 64) or (plen == 76): # Euromap67 interface = 12 bytes
            return MasterboardData_V18.unpack(buf)
        elif (plen == 72) or (plen == 92): # Euromap67 interface = 20 bytes
            return MasterboardData_V30.unpack(buf)
        else:
            print "MasterboardData has wrong length: " + str(plen)
            print "Euromap67Interface is ignored"
            return md

#this parses MasterboardData for versions <= v1.8 (i.e. 1.6, 1.7, 1.8)
class MasterboardData_V18(object):
    __slots__ = ['digital_input_bits', 'digital_output_bits',
                 'analog_input_range0', 'analog_input_range1',
                 'analog_input0', 'analog_input1',
                 'analog_output_domain0', 'analog_output_domain1',
                 'analog_output0', 'analog_output1',
                 'masterboard_temperature',
                 'robot_voltage_48V', 'robot_current',
                 'master_io_current', 'master_safety_state',
                 'master_onoff_state']#subsequent slots related to 'euromap' ignored
    @staticmethod
    def unpack(buf):
        md = MasterboardData_V18()
        (_, _,
         md.digital_input_bits, md.digital_output_bits,
         md.analog_input_range0, md.analog_input_range1,
         md.analog_input0, md.analog_input1,
         md.analog_output_domain0, md.analog_output_domain1,
         md.analog_output0, md.analog_output1,
         md.masterboard_temperature,
         md.robot_voltage_48V, md.robot_current,
         md.master_io_current, md.master_safety_state,
         md.master_onoff_state) = struct.unpack_from("!IBhhbbddbbddffffBB", buf)
        return md

#this parses MasterboardData for versions >=3.0 (i.e. 3.0)
class MasterboardData_V30(object):
    __slots__ = ['digital_input_bits', 'digital_output_bits',
                 'analog_input_range0', 'analog_input_range1',
                 'analog_input0', 'analog_input1',
                 'analog_output_domain0', 'analog_output_domain1',
                 'analog_output0', 'analog_output1',
                 'masterboard_temperature',
                 'robot_voltage_48V', 'robot_current',
                 'master_io_current', 'safety_mode',
                 'in_reduced_mode']#subsequent slots related to 'euromap' ignored
    @staticmethod
    def unpack(buf):
        md = MasterboardData_V30()
        (_, _,
         md.digital_input_bits, md.digital_output_bits,
         md.analog_input_range0, md.analog_input_range1,
         md.analog_input0, md.analog_input1,
         md.analog_output_domain0, md.analog_output_domain1,
         md.analog_output0, md.analog_output1,
         md.masterboard_temperature,
         md.robot_voltage_48V, md.robot_current,
         md.master_io_current, md.safety_mode,
         md.in_reduced_mode) = struct.unpack_from("!IBiibbddbbddffffBB", buf)
        return md

#this parses JointData for all versions (i.e. 1.6, 1.7, 1.8, 3.0)
class CartesianInfo(object):
    __slots__ = ['x', 'y', 'z', 'rx', 'ry', 'rz']
    @staticmethod
    def unpack(buf):
        ci = CartesianInfo()
        (_, _,
         ci.x, ci.y, ci.z, ci.rx, ci.ry, ci.rz) = struct.unpack_from("!IB6d", buf)
        return ci

#this parses KinematicsInfo for versions (i.e. 1.8, 3.0)
#KinematicsInfo is not available in 1.6 and 1.7
class KinematicsInfo(object):
    @staticmethod
    def unpack(buf):
        return KinematicsInfo()

#helper class for ConfigurationData
class JointLimitData(object):
    __slots__ = ['min_limit', 'max_limit', 'max_speed', 'max_acceleration']

#this parses ConfigurationData for versions (i.e. 1.8, 3.0)
#ConfigurationData is not available in 1.6 and 1.7
class ConfigurationData(object):
    __slots__ = ['joint_limit_data',
                 'v_joint_default', 'a_joint_default',
                 'v_tool_default', 'a_tool_default', 'eq_radius',
                 'dh_a', 'dh_d', 'dh_alpha', 'dh_theta',
                 'masterboard_version', 'controller_box_type',
                 'robot_type', 'robot_subtype']#in v1.8 there is an additional slot 'motor_type' for each joint, which currently is ignored!
    @staticmethod
    def unpack(buf):
        cd = ConfigurationData()
        cd.joint_limit_data = []
        for i in range(6):
            jld = JointLimitData()
            (jld.min_limit, jld.max_limit) = struct.unpack_from("!dd", buf, 5+16*i)
            (jld.max_speed, jld.max_acceleration) = struct.unpack_from("!dd", buf, 5+16*6+16*i)
            cd.joint_limit_data.append(jld)
        (cd.v_joint_default, cd.a_joint_default, cd.v_tool_default, cd.a_tool_default,
         cd.eq_radius) = struct.unpack_from("!ddddd", buf, 5+32*6)
        (cd.masterboard_version, cd.controller_box_type, cd.robot_type,
         cd.robot_subtype) = struct.unpack_from("!iiii", buf, 5+32*6+5*8+6*32)
        return cd

#this parses KinematicsInfo for versions (i.e. 1.8, 3.0)
#KinematicsInfo is not available in 1.6 and 1.7
class ForceModeData(object):
    __slots__ = ['x', 'y', 'z', 'rx', 'ry', 'rz', 'robot_dexterity']
    @staticmethod
    def unpack(buf):
        fmd = ForceModeData()
        (_, _, fmd.x, fmd.y, fmd.z, fmd.rx, fmd.ry, fmd.rz,
         fmd.robot_dexterity) = struct.unpack_from("!IBddddddd", buf)
        return fmd

#this class handles different protocol versions
class AdditionalInfo(object):
    @staticmethod
    def unpack(buf):
        ai = AdditionalInfo()
        (plen, ptype) = struct.unpack_from("!IB", buf)
        if plen == 10:
            return AdditionalInfoOld.unpack(buf)
        elif plen == 7:
            return AdditionalInfoNew.unpack(buf)
        else:
            print "AdditionalInfo has wrong length: " + str(plen)
            return ai

class AdditionalInfoOld(object):
    __slots__ = ['ctrl_bits', 'teach_button']
    @staticmethod
    def unpack(buf):
        ai = AdditionalInfoOld()
        (_,_,ai.ctrl_bits, ai.teach_button) = struct.unpack_from("!IBIB", buf)
        return ai

class AdditionalInfoNew(object):
    __slots__ = ['teach_button_enabled','teach_button_pressed']
    @staticmethod
    def unpack(buf):
        ai = AdditionalInfoNew()
        (_,_,ai.teach_button_enabled, ai.teach_button_pressed) = struct.unpack_from("!IBBB", buf)
        return ai


class RobotState(object):
    __slots__ = ['robot_mode_data', 'joint_data', 'tool_data',
                 'masterboard_data', 'cartesian_info',
                 'kinematics_info', 'configuration_data',
                 'force_mode_data', 'additional_info',
                 'unknown_ptypes']

    def __init__(self):
        self.unknown_ptypes = []

    @staticmethod
    def unpack(buf):
        rs = RobotState()
        offset = 5
        while offset < len(buf):
            length, ptype = struct.unpack_from("!IB", buf, offset)
            assert offset + length <= len(buf)
            package_buf = buffer(buf, offset, length)
            offset += length

            if ptype == RobotStatePackageType.ROBOT_MODE_DATA:
                rs.robot_mode_data = RobotModeData.unpack(package_buf)
            elif ptype == RobotStatePackageType.JOINT_DATA:
                rs.joint_data = JointData.unpack(package_buf)
            elif ptype == RobotStatePackageType.TOOL_DATA:
                rs.tool_data = ToolData.unpack(package_buf)
            elif ptype == RobotStatePackageType.MASTERBOARD_DATA:
                rs.masterboard_data = MasterboardData.unpack(package_buf)
            elif ptype == RobotStatePackageType.CARTESIAN_INFO:
                rs.cartesian_info = CartesianInfo.unpack(package_buf)
            elif ptype == RobotStatePackageType.KINEMATICS_INFO:
                rs.kinematics_info = KinematicsInfo.unpack(package_buf)
            elif ptype == RobotStatePackageType.CONFIGURATION_DATA:
                rs.configuration_data = ConfigurationData.unpack(package_buf)
            elif ptype == RobotStatePackageType.FORCE_MODE_DATA:
                rs.force_mode_data = ForceModeData.unpack(package_buf)
            elif ptype == RobotStatePackageType.ADDITIONAL_INFO:
                rs.additional_info = AdditionalInfo.unpack(package_buf)
            elif ptype == RobotStatePackageType.CALIBRATION_DATA:
                pass # internal data, should be skipped
            else:
                rs.unknown_ptypes.append(ptype)
        return rs



class RobotMessageVersion(object):
    __slots__ = ['messageSize', 'messageType',
                 'timestamp', 'source', 'robotMessageType',
                 'projectNameSize', 'projectName',
                 'majorVersion', 'minorVersion',
                 'svnRevision', 'buildDate']
    @staticmethod
    def unpack(buf):
        rmv = RobotMessageVersion()
        offset = 0
        
        # messageSize: 1x int (1x 4byte)
        rmv.messageSize = struct.unpack_from("!I",buf, offset)[0]
        offset+=4
        
        # messageType: 1x unsigned char (1x 1byte)
        rmv.messageType = struct.unpack_from("!B",buf, offset)[0]
        offset+=1
        
        # timestamp: 1x uint64_t (1x 8byte)
        rmv.timestamp = struct.unpack_from("!Q",buf, offset)[0]
        offset+=8
        
        # source: 1x char (1x 1byte)
        rmv.source = struct.unpack_from("!B",buf, offset)[0]
        offset+=1
        
        # robotMessageType: 1x char (1x 1byte)
        rmv.robotMessageType = struct.unpack_from("!B",buf, offset)[0]
        offset+=1
        
        # projectNameSize: 1x char (1x 1byte)
        rmv.projectNameSize = struct.unpack_from("!B",buf, offset)[0]
        offset+=1
        
        # projectName: char array (projectNameSize x 1byte)
        #rmv.projectName = struct.unpack_from("!%iB" % int(rmv.projectNameSize), buf, offset)[0]
        #print rmv.projectName
        offset+=rmv.projectNameSize
        
        # majorVersion: 1x unsigned char (1x 1byte)
        rmv.majorVersion = struct.unpack_from("!B",buf, offset)[0]
        offset+=1
        
        # minorVersion: 1x unsigned char (1x 1byte)
        rmv.minorVersion = struct.unpack_from("!B",buf, offset)[0]
        offset+=1
        
        # svnRevision: 1x int (1x 4byte)
        rmv.minorVersion = struct.unpack_from("!I",buf, offset)[0]
        offset+=4
        
        # buildDate: char array (projectNameSize x 1byte)
        #rmv.buildDate = struct.unpack_from("!%iB" % int(rmv.messageSize - offset), buf, offset)[0]
        
        return rmv


class RobotMessage(object):
    __slots__ = ['version_message', 'unknown_ptypes']

    def __init__(self):
        self.unknown_ptypes = []
    
    @staticmethod
    def unpack(buf):
        rm = RobotMessage()
        offset = 0
        
        while offset < len(buf):
            messageSize, messageType, timestamp, source, robotMessageType = struct.unpack_from("!iBQBB", buf, offset)
            assert offset + messageSize <= len(buf)
            package_buf = buffer(buf, offset, messageSize)
            offset += messageSize

            if robotMessageType == RobotMessagePackageType.ROBOT_MESSAGE_VERSION:
                rm.version_message = RobotMessageVersion.unpack(package_buf)
            else:
                print("UNKNOWN_MESSAGE")
                print robotMessageType
                rm.unknown_ptypes.append(robotMessageType)
        return rm


class SecondaryClientPacket(object):
    __slots__ = ['robot_state', 'robot_message']

    def __init__(self):
        self.robot_state = None
        self.robot_message = None
        
    @staticmethod
    def unpack(buf):
        length, mtype = struct.unpack_from("!IB", buf)
        if length != len(buf):
            raise Exception("Could not unpack packet: length field is incorrect")
        
        scp = SecondaryClientPacket()
        if mtype == 16:
            scp.robot_state = RobotState.unpack(buf)
        elif mtype == 20:
            scp.robot_message = RobotMessage.unpack(buf)
        else:
            print "Unknown MessageType"
            raise Exception("Fatal error when unpacking SecondaryClientPacket")

        return scp


def pstate(o, indent=''):
    for s in o.__slots__:
        child = getattr(o, s, None)
        if child is None:
            print "%s%s: None" % (indent, s)
        elif hasattr(child, '__slots__'):
            print "%s%s:" % (indent, s)
            pstate(child, indent + '    ')
        elif hasattr(child, '__iter__'):
            print "%s%s:" % (indent, s)
            for i, c in enumerate(child):
                print "%s  [%i]:" % (indent, i)
                pstate(c, indent + '    ')
        else:
            print "%s%s: %s" % (indent, s, child)
