"""
Mambo class holds all of the methods needed to pilot the drone from python and to ask for sensor
data back from the drone

Author: Amy McGovern, dramymcgovern@gmail.com
"""
from bluepy.btle import Peripheral, UUID, DefaultDelegate, BTLEException
import untangle
import struct
import time

class MamboDelegate(DefaultDelegate):
    """
    Handle BLE notififications
    """
    def __init__(self, handle_map, mambo):
        DefaultDelegate.__init__(self)
        self.handle_map = handle_map
        self.mambo = mambo
        self.mambo._debug_print("initializing notification delegate", 10)

    def handleNotification(self, cHandle, data):
        #print "handling notificiation from channel %d" % cHandle
        #print "handle map is %s " % self.handle_map[cHandle]
        #print "channel map is %s " % self.mambo.characteristic_receive_uuids[self.handle_map[cHandle]]
        #print "data is %s " % data

        channel = self.mambo.characteristic_receive_uuids[self.handle_map[cHandle]]
        if channel == 'ACK_DRONE_DATA':
            # data received from drone (needs to be ack on 1e)
            self.mambo._update_sensors(data, ack=True)
        elif channel == 'NO_ACK_DRONE_DATA':
            # data from drone (including battery and others), no ack
            self.mambo._debug_print("drone data - no ack needed", 2)
            self.mambo._update_sensors(data, ack=False)
        elif channel == 'ACK_COMMAND_SENT':
            # ack 0b channel, SEND_WITH_ACK
            self.mambo._debug_print("Ack!  command received!", 2)
            self.mambo._set_command_received('SEND_WITH_ACK', True)
        elif channel == 'ACK_HIGH_PRIORITY':
            # ack 0c channel, SEND_HIGH_PRIORITY
            self.mambo._debug_print("Ack!  high priority received", 2)
            self.mambo._set_command_received('SEND_HIGH_PRIORITY', True)
        else:
            self.mambo._debug_print("unknown channel %s sending data " % channel, 10)
            self.mambo._debug_print(cHandle)
            

class MamboSensors:
    """
    Store the mambo's last known sensor values
    """

    def __init__(self):

        # default to full battery
        self.battery = 100

        # drone on the ground
        self.flying_state = "landed"

        self.unknown_sensors = dict()

        self.gun_id = 0
        self.gun_state = None

        self.claw_id = 0
        self.claw_state = None

        # new SDK sends speed, altitude, and quaternions
        self.speed_x = 0
        self.speed_y = 0
        self.speed_z = 0
        self.speed_ts = 0

        self.altitude = 0
        self.altitude_ts = 0

        self.quaternion_w = 0
        self.quaternion_x = 0
        self.quaternion_y = 0
        self.quaternion_z = 0
        self.quaternion_ts = 0

    def update(self, name, value, sensor_enum):
        """
        Update the sensor

        :param name: name of the sensor to update
        :param value: new value for the sensor
        :param sensor_enum: enum list for the sensors that use enums so that we can translate from numbers to strings
        :return:
        """
        if (name, "enum") in sensor_enum:
            # grab the string value
            if (value > len(sensor_enum[(name, "enum")])):
                value = "UNKNOWN_ENUM_VALUE"
            else:
                enum_value = sensor_enum[(name, "enum")][value]
                value = enum_value

        # add it to the sensors
        if (name == "BatteryStateChanged_battery_percent"):
            self.battery = value
        elif (name == "FlyingStateChanged_state"):
            self.flying_state = value
        elif (name == "ClawState_id"):
            self.claw_id = value
        elif (name == "ClawState_state"):
            self.claw_state = value
        elif (name == "GunState_id"):
            self.gun_id = value
        elif (name == "GunState_state"):
            self.gun_state = value
        elif (name == "DroneSpeed_speed_x"):
            self.speed_x = value
        elif (name == "DroneSpeed_speed_y"):
            self.speed_y = value
        elif (name == "DroneSpeed_speed_z"):
            self.speed_z = value
        elif (name == "DroneSpeed_ts"):
            self.speed_ts = value
        elif (name == "DroneAltitude_altitude"):
            self.altitude = value
        elif (name == "DroneAltitude_altitude_ts"):
            self.altitude_ts = value
        elif (name == "DroneQuaternion_q_w"):
            self.quaternion_w = value
        elif (name == "DroneQuaternion_q_x"):
            self.quaternion_x = value
        elif (name == "DroneQuaternion_q_y"):
            self.quaternion_y = value
        elif (name == "DroneQuaternion_q_z"):
            self.quaternion_z = value
        elif (name == "DroneQuaternion_tz"):
            self.quaternion_ts = value
        else:
            #print "new sensor - add me to the struct but saving in the dict for now"
            self.unknown_sensors[name] = value

    def __str__(self):
        """
        Make a nicely printed struct for debugging

        :return: string for print calls
        """
        my_str = "mambo state: battery %d, " % self.battery
        my_str += "flying state is %s, " % self.flying_state
        my_str += "speed (x, y, z) and ts is (%f, %f, %f) at %f " % (self.speed_x, self.speed_y, self.speed_z, self.speed_ts)
        my_str += "altitude (m) %f and ts is %f " % (self.altitude, self.altitude_ts)
        my_str += "quaternion (w, x, y, z) and ts is (%f, %f, %f, %f) at %f " % (
            self.quaternion_w, self.quaternion_x, self.quaternion_y, self.quaternion_z, self.quaternion_ts)
        my_str += "gun id: %d, state %s, " % (self.gun_id, self.gun_state)
        my_str += "claw id: %d, state %s, " % (self.claw_id, self.claw_state)
        my_str += "unknown sensors: %s," % self.unknown_sensors
        return my_str

class Mambo:
    def __init__(self, address, debug_level=None):
        """
        Initialize with its address - if you don't know the address, call findMambo
        and that will discover it for you.

        You can specify a debugging level for printing output.  debug level = 0 will print everything,
        1 fewer things, etc.  Each statement has a priority associated with it.  None (default) means print nothing.
        Internally, 10 is a high priority command to see whereas 1 is simply debugging info.

        :param address: unique address for this mambo
        :param debugLevel: use to control the amount of print statements.  Valid choices are None (default) or any integer >= 0

        """
        self.address = address
        self.drone = Peripheral()
        self.debug_level = debug_level

        # the following UUID segments come from the Mambo and from the documenation at
        # http://forum.developer.parrot.com/t/minidrone-characteristics-uuid/4686/3
        # the 3rd and 4th bytes are used to identify the service
        self.service_uuids = {
            'fa00' : 'ARCOMMAND_SENDING_SERVICE',
            'fb00' : 'ARCOMMAND_RECEIVING_SERVICE',
            'fc00' : 'PERFORMANCE_COUNTER_SERVICE',
            'fd21' : 'NORMAL_BLE_FTP_SERVICE',
            'fd51' : 'UPDATE_BLE_FTP',
            'fe00' : 'UPDATE_RFCOMM_SERVICE',
            '1800' : 'Device Info',
            '1801' : 'unknown',
            }
        # the following characteristic UUID segments come from the documentation at
        # http://forum.developer.parrot.com/t/minidrone-characteristics-uuid/4686/3
        # the 4th bytes are used to identify the characteristic
        # the usage of the channels are also documented here
        # http://forum.developer.parrot.com/t/ble-characteristics-of-minidrones/5912/2
        self.characteristic_send_uuids = {
            '0a' : 'SEND_NO_ACK',           # not-ack commands (PCMD only)
            '0b' : 'SEND_WITH_ACK',         # ack commands (all piloting commands)
            '0c' : 'SEND_HIGH_PRIORITY',    # emergency commands
            '1e' : 'ACK_COMMAND'            # ack for data sent on 0e
            }

        # counters for each packet (required as part of the packet)
        self.characteristic_send_counter = {
            'SEND_NO_ACK': 0,
            'SEND_WITH_ACK': 0,
            'SEND_HIGH_PRIORITY': 0,
            'ACK_COMMAND': 0,
            'RECEIVE_WITH_ACK': 0
        }

        # the following characteristic UUID segments come from the documentation at
        # http://forum.developer.parrot.com/t/minidrone-characteristics-uuid/4686/3
        # the 4th bytes are used to identify the characteristic
        # the types of commands and data coming back are also documented here
        # http://forum.developer.parrot.com/t/ble-characteristics-of-minidrones/5912/2
        self.characteristic_receive_uuids = {
            '0e' : 'ACK_DRONE_DATA',        # drone data that needs an ack (needs to be ack on 1e)
            '0f' : 'NO_ACK_DRONE_DATA',     # data from drone (including battery and others), no ack
            '1b' : 'ACK_COMMAND_SENT',      # ack 0b channel, SEND_WITH_ACK
            '1c' : 'ACK_HIGH_PRIORITY',     # ack 0c channel, SEND_HIGH_PRIORITY
            }

        # these are the FTP incoming and outcoming channels
        # the handling characteristic seems to be the one to send commands to (per the SDK)
        # information gained from reading ARUTILS_BLEFtp.m in the SDK
        self.characteristic_ftp_uuids = {
            '22' : 'NORMAL_FTP_TRANSFERRING',
            '23' : 'NORMAL_FTP_GETTING',
            '24' : 'NORMAL_FTP_HANDLING',
            '52' : 'UPDATE_FTP_TRANSFERRING',
            '53': 'UPDATE_FTP_GETTING',
            '54': 'UPDATE_FTP_HANDLING',
        }

        # FTP commands (obtained via ARUTILS_BLEFtp.m in the SDK)
        self.ftp_commands = {
            "list" : "LIS",
            "get" : "GET"
        }

        # need to save for communication (but they are initialized in connect)
        self.services = None
        self.send_characteristics = dict()
        self.receive_characteristics = dict()
        self.handshake_characteristics = dict()
        self.ftp_characteristics = dict()

        # parse the command files from XML (so we don't have to store ids and can use names
        # for readability and portability!)
        self.common_commands = untangle.parse('common.xml')
        self.minidrone_commands = untangle.parse('minidrone.xml')

        self.data_types = {
            'ACK' : 1,
            'DATA_NO_ACK': 2,
            'LOW_LATENCY_DATA': 3,
            'DATA_WITH_ACK' : 4
        }

        # store whether a command was acked
        self.command_received = {
            'SEND_WITH_ACK': False,
            'SEND_HIGH_PRIORITY': False,
            'ACK_COMMAND': False
        }
        
        # sensors are stored in a MamboSensor object
        self.sensors = MamboSensors()

        # instead of parsing the XML file every time, cache the results
        self.command_tuple_cache = dict()
        self.sensor_tuple_cache = dict()

        # maximum number of times to try a packet before assuming it failed
        self.max_packet_retries = 3

    def _debug_print(self, print_str, level):
        """
        Internal method to only print based on the debugging level

        :param print_str: string to print
        :param level: level of debugging that this statement is
        :return:
        """
        # special case: do not print anything
        if (self.debug_level is None):
            return

        # handle null cases
        if (print_str is None):
            print_str = ""

        # prints the messages in color according to their level
        if (level >= self.debug_level):
            if (level >= 10):
                print('\033[38;5;196m' + print_str + '\033[0m')
            elif (level >= 9):
                print('\033[38;5;202m' + print_str + '\033[0m')
            elif (level >= 5):
                print('\033[38;5;22m' + print_str + '\033[0m')
            elif (level >= 2):
                print('\033[38;5;33m' + print_str + '\033[0m')
            else:
                print print_str

    def connect(self, num_retries):
        """
        Connects to the drone and re-tries in case of failure the specified number of times

        :param: num_retries is the number of times to retry

        :return: True if it succeeds and False otherwise
        """

        try_num = 1
        while (try_num < num_retries):
            try:
                self._connect()
                return True
            except BTLEException:
                self._debug_print("retrying connections", 10)
                try_num += 1

        # if we fell through the while loop, it failed to connect
        return False


    def _reconnect(self, num_retries):
        """
        Reconnect to the drone (assumed the BLE crashed)

        :param: num_retries is the number of times to retry

        :return: True if it succeeds and False otherwise
        """
        try_num = 1
        success = False
        while (try_num < num_retries and not success):
            try:
                self._debug_print("trying to re-connect to the mambo at address %s" % self.address, 10)
                self.drone.connect(self.address, "random")
                self._debug_print("connected!  Asking for services and characteristics", 5)
                success = True
            except BTLEException:
                self._debug_print("retrying connections", 10)
                try_num += 1

        if (success):
            # do the magic handshake
            self._perform_handshake()

        return success
        
    def _connect(self):
        """
        Connect to the mambo to prepare for flying - includes getting the services and characteristics
        for communication

        :return: throws an error if the drone connection failed.  Returns void if nothing failed.
        """
        self._debug_print("trying to connect to the mambo at address %s" % self.address, 10)
        self.drone.connect(self.address, "random")
        self._debug_print("connected!  Asking for services and characteristics", 5)

        # re-try until all services have been found
        allServicesFound = False

        # used for notifications
        handle_map = dict()

        while not allServicesFound:
            # get the services
            self.services = self.drone.getServices()

            # loop through the services
            for s in self.services:
                hex_str = self._get_byte_str_from_uuid(s.uuid, 3, 4)

                # store the characteristics for receive & send
                if (self.service_uuids[hex_str] == 'ARCOMMAND_RECEIVING_SERVICE'):
                    # only store the ones used to receive data
                    for c in s.getCharacteristics():
                        hex_str = self._get_byte_str_from_uuid(c.uuid, 4, 4)
                        if hex_str in self.characteristic_receive_uuids:
                            self.receive_characteristics[self.characteristic_receive_uuids[hex_str]] = c
                            handle_map[c.getHandle()] = hex_str


                elif (self.service_uuids[hex_str] == 'ARCOMMAND_SENDING_SERVICE'):
                    # only store the ones used to send data
                    for c in s.getCharacteristics():
                        hex_str = self._get_byte_str_from_uuid(c.uuid, 4, 4)
                        if hex_str in self.characteristic_send_uuids:
                            self.send_characteristics[self.characteristic_send_uuids[hex_str]] = c


                elif (self.service_uuids[hex_str] == 'UPDATE_BLE_FTP'):
                    # store the FTP info
                    for c in s.getCharacteristics():
                        hex_str = self._get_byte_str_from_uuid(c.uuid, 4, 4)
                        if hex_str in self.characteristic_ftp_uuids:
                            self.ftp_characteristics[self.characteristic_ftp_uuids[hex_str]] = c

                elif (self.service_uuids[hex_str] == 'NORMAL_BLE_FTP_SERVICE'):
                    # store the FTP info
                    for c in s.getCharacteristics():
                        hex_str = self._get_byte_str_from_uuid(c.uuid, 4, 4)
                        if hex_str in self.characteristic_ftp_uuids:
                            self.ftp_characteristics[self.characteristic_ftp_uuids[hex_str]] = c

                # need to register for notifications and write 0100 to the right handles
                # this is sort of magic (not in the docs!) but it shows up on the forum here
                # http://forum.developer.parrot.com/t/minimal-ble-commands-to-send-for-take-off/1686/2
                # Note this code snippet below more or less came from the python example posted to that forum (I adapted it to my interface)
                for c in s.getCharacteristics():
                    if self._get_byte_str_from_uuid(c.uuid, 3, 4) in \
                            ['fb0f', 'fb0e', 'fb1b', 'fb1c', 'fd22', 'fd23', 'fd24', 'fd52', 'fd53', 'fd54']:
                        self.handshake_characteristics[self._get_byte_str_from_uuid(c.uuid, 3, 4)] = c


            # check to see if all 8 characteristics were found
            allServicesFound = True
            for r_id in self.characteristic_receive_uuids.itervalues():
                if r_id not in self.receive_characteristics:
                    self._debug_print("setting to false in receive on %s" % r_id, 5)
                    allServicesFound = False

            for s_id in self.characteristic_send_uuids.itervalues():
                if s_id not in self.send_characteristics:
                    self._debug_print("setting to false in send", 5)
                    allServicesFound = False

            for f_id in self.characteristic_ftp_uuids.itervalues():
                if f_id not in self.ftp_characteristics:
                    self._debug_print("setting to false in ftp", 5)
                    allServicesFound = False

            # and ensure all handshake characteristics were found
            if len(self.handshake_characteristics.keys()) != 10:
                self._debug_print("setting to false in len", 5)
                allServicesFound = False


        # do the magic handshake
        self._perform_handshake()

        # initialize the delegate to handle notifications
        self.drone.setDelegate(MamboDelegate(handle_map, self))

    def _perform_handshake(self):
        """
        Magic handshake
        Need to register for notifications and write 0100 to the right handles
        This is sort of magic (not in the docs!) but it shows up on the forum here
        http://forum.developer.parrot.com/t/minimal-ble-commands-to-send-for-take-off/1686/2

        :return: nothing
        """
        self._debug_print("magic handshake to make the drone listen to our commands", 2)
        
        # Note this code snippet below more or less came from the python example posted to that forum (I adapted it to my interface)
        for c in self.handshake_characteristics.itervalues():
            # for some reason bluepy characteristic handle is two lower than what I need...
            # Need to write 0x0100 to the characteristics value handle (which is 2 higher)
            self.drone.writeCharacteristic(c.handle + 2, struct.pack("<BB", 1, 0))

    def disconnect(self):
        """
        Disconnect the BLE connection.  Always call this at the end of your programs to
        cleanly disconnect.

        :return: void
        """
        self.drone.disconnect()

    def _update_sensors(self, data, ack):
        """
        Update the sensors with the data in the BLE packet
        :param data: BLE packet of sensor data
        :return:
        """
        print "updating sensors with "
        header_tuple = struct.unpack_from("<BBBBBB", data)
        print header_tuple
        (names, data_sizes) = self._parse_sensor_tuple(header_tuple)
        print "name of sensor is %s" % names
        print "data size is %s" % data_sizes

        if names is not None:
            for idx, name in enumerate(names):
                data_size = data_sizes[idx]

                if (data_size == "u8" or data_size == "enum"):
                    # unsigned 8 bit, single byte
                    sensor_data = struct.unpack_from("<B", data, offset=6)
                    sensor_data = int(sensor_data[0])
                elif (data_size == "i8"):
                    # signed 8 bit, single byte
                    sensor_data = struct.unpack_from("<b", data, offset=6)
                    sensor_data = int(sensor_data[0])
                elif (data_size == "u16"):
                    sensor_data = struct.unpack_from("<H", data, offset=6)
                    sensor_data = int(sensor_data[0])
                elif (data_size == "i16"):
                    sensor_data = struct.unpack_from("<h", data, offset=6)
                    sensor_data = int(sensor_data[0])
                elif (data_size == "u32"):
                    sensor_data = struct.unpack_from("<I", data, offset=6)
                    sensor_data = int(sensor_data[0])
                elif (data_size == "i32"):
                    sensor_data = struct.unpack_from("<i", data, offset=6)
                    sensor_data = int(sensor_data[0])
                elif (data_size == "u64"):
                    sensor_data = struct.unpack_from("<Q", data, offset=6)
                    sensor_data = int(sensor_data[0])
                elif (data_size == "i64"):
                    sensor_data = struct.unpack_from("<q", data, offset=6)
                    sensor_data = int(sensor_data[0])
                elif (data_size == "float"):
                    sensor_data = struct.unpack_from("<f", data, offset=6)
                    sensor_data = float(sensor_data[0])
                elif (data_size == "double"):
                    sensor_data = struct.unpack_from("<d", data, offset=6)
                    sensor_data = float(sensor_data[0])
                elif (data_size == "string"):
                    # string
                    sensor_data = struct.unpack_from("<s", data, offset=6)
                    sensor_data = sensor_data[0]
                else:
                    sensor_data = None
                    self._debug_print("Write the parser for this value", 10)
                    self._debug_print(data_size, 10)

                self._debug_print("updating the sensor!", 1)
                self.sensors.update(name, sensor_data, self.sensor_tuple_cache)
        else:
            #print header_tuple
            self._debug_print("Error parsing sensor information!", 10)

        self._debug_print(self.sensors, 1)

        if (ack):
            self._ack_packet(header_tuple[1])

    def _parse_sensor_tuple(self, sensor_tuple):
        """
        Parses the sensor information from the command id bytes and returns the name
        of the sensor and the size of the data (so it can be unpacked properly)

        :param sensor_tuple: the command id tuple to be parsed (type, packet id, command tuple 3 levels deep)
        :return: a tuple with (name of the sensor, data size to be used for grabbing the rest of the data)
        """
        # grab the individual values
        (ack_id, packet_id, project_id, myclass_id, cmd_id, extra_id) = sensor_tuple

        # return the cache if it is there
        if (project_id, myclass_id, cmd_id, extra_id) in self.sensor_tuple_cache:
            return self.sensor_tuple_cache[(project_id, myclass_id, cmd_id, extra_id)]


        self._debug_print("looking for project id %d in minidrone" % project_id, 1)
        if (project_id == int(self.minidrone_commands.project['id'])):
            self._debug_print("looking for myclass_id %d" % myclass_id, 1)
            for c in self.minidrone_commands.project.myclass:
                self._debug_print("looking for cmd_id %d" % cmd_id, 1)
                if int(c['id']) == myclass_id:
                    for cmd_child in c.cmd:
                        if int(cmd_child['id']) == cmd_id:
                            cmd_name = cmd_child['name']
                            sensor_names = list()
                            data_sizes = list()

                            if (hasattr(cmd_child, 'arg')):
                                for arg_child in cmd_child.arg:
                                    sensor_name = cmd_name + "_" + arg_child['name']
                                    data_size = arg_child['type']

                                    # special case, if it is an enum, need to add the enum mapping into the cache
                                    if (data_size == 'enum'):
                                        enum_names = list()
                                        for eitem in arg_child.enum:
                                            self._debug_print(eitem, 1)
                                            enum_names.append(eitem['name'])
                                        self.sensor_tuple_cache[sensor_name, "enum"] = enum_names
                                        self._debug_print("added to sensor cache %s" % enum_names, 1)

                                    # save the name and sizes to a list
                                    sensor_names.append(sensor_name)
                                    data_sizes.append(data_size)
                            else:
                                # there is no sub-child argument meaning this is just a pure notification
                                # special case values just use the command name and None for size
                                sensor_names.append(cmd_name)
                                data_sizes.append(None)

                            # cache the results
                            self.sensor_tuple_cache[(project_id, myclass_id, cmd_id, extra_id)] = (sensor_names, data_sizes)
                            return (sensor_names, data_sizes)
                        

        # need to look in the common.xml file instead
        self._debug_print("looking for project id %d in common" % project_id, 1)
        if (project_id == int(self.common_commands.project['id'])):
            self._debug_print("looking for myclass_id %d" % myclass_id, 1)
            for c in self.common_commands.project.myclass:
                self._debug_print("looking for cmd_id %d" % cmd_id, 1)
                if int(c['id']) == myclass_id:
                    for cmd_child in c.cmd:
                        if int(cmd_child['id']) == cmd_id:
                            cmd_name = cmd_child['name']
                            sensor_names = list()
                            data_sizes = list()

                            if (hasattr(cmd_child, 'arg')):
                                for arg_child in cmd_child.arg:
                                    sensor_name = cmd_name + "_" + arg_child['name']
                                    data_size = arg_child['type']

                                    # special case, if it is an enum, need to add the enum mapping into the cache
                                    if (data_size == 'enum'):
                                        enum_names = list()
                                        for eitem in arg_child.enum:
                                            self._debug_print(eitem, 1)
                                            enum_names.append(eitem['name'])
                                        self.sensor_tuple_cache[sensor_name, "enum"] = enum_names
                                        self._debug_print("added to sensor cache %s" % enum_names, 1)

                                    # save the name and sizes to a list
                                    sensor_names.append(sensor_name)
                                    data_sizes.append(data_size)
                            else:
                                # there is no sub-child argument meaning this is just a pure notification
                                # special case values just use the command name and None for size
                                sensor_names.append(cmd_name)
                                data_sizes.append(None)

                            # cache the results
                            self.sensor_tuple_cache[(project_id, myclass_id, cmd_id, extra_id)] = (sensor_names, data_sizes)
                            return (sensor_names, data_sizes)
                        
        # didn't find it, return an error
        # cache the results
        self.sensor_tuple_cache[(project_id, myclass_id, cmd_id, extra_id)] = (None, None)
        return (None, None)

    def get_camera_files(self):
        """
        Get the listing of files from the ftp on the drone

        :return:
        """
        BLE_PACKET_MAX_SIZE=132

        num_zeros = BLE_PACKET_MAX_SIZE-len(self.ftp_commands["list"])
        my_zeros = "\0" * num_zeros
        print len(my_zeros)
        print self.ftp_commands["list"]
        fmt = format("<3s%dss" % num_zeros)
        print fmt

        packet = struct.pack(fmt, self.ftp_commands["list"], my_zeros, "/.")
        print len(packet)
        print packet
                
        self.ftp_characteristics['NORMAL_FTP_HANDLING'].write(packet)


    def _ack_packet(self, packet_id):
        """
        Ack the packet id specified by the argument on the ACK_COMMAND channel

        :param packet_id: the packet id to ack
        :return: nothing
        """
        self._debug_print("ack last packet on the ACK_COMMAND channel", 1)
        self.characteristic_send_counter['ACK_COMMAND'] = (self.characteristic_send_counter['ACK_COMMAND'] + 1) % 256
        packet = struct.pack("<BBB", self.data_types['ACK'], self.characteristic_send_counter['ACK_COMMAND'],
                             packet_id)
        self._debug_print("sending packet %d %d %d" % (self.data_types['ACK'], self.characteristic_send_counter['ACK_COMMAND'],
                                           packet_id), 1)

        self._safe_ble_write(characteristic=self.send_characteristics['ACK_COMMAND'], packet=packet)
        #self.send_characteristics['ACK_COMMAND'].write(packet)



    def _get_byte_str_from_uuid(self, uuid, byte_start, byte_end):
        """
        Extract the specified byte string from the UUID btle object.  This is an ugly hack
        but it was necessary because of the way the UUID object is represented and the documentation
        on the byte strings from Parrot.  You give it the starting byte (counting from 1 since
        that is how their docs count) and the ending byte and it returns that as a string extracted
        from the UUID.  It is assumed it happens before the first - in the UUID.

        :param uuid: btle UUID object
        :param byte_start: starting byte (counting from 1)
        :param byte_end: ending byte (counting from 1)
        :return: string with the requested bytes (to be used as a key in the lookup tables for services)
        """
        uuid_str = format("%s" % uuid)
        idx_start = 2 * (byte_start - 1)
        idx_end = 2 * (byte_end)
        
        my_hex_str = uuid_str[idx_start:idx_end]
        return my_hex_str

    def _set_command_received(self, channel, val):
        """
        Set the command received on the specified channel to the specified value (used for acks)

        :param channel: channel
        :param val: True or False
        :return:
        """
        self.command_received[channel] = val

    def _get_command_tuple(self, myclass, cmd):
        """
        Parses the command XML for the specified class name and command name

        :param myclass: class name (renamed to myclass to avoid reserved name) in the xml file
        :param cmd: command to execute (from XML file)
        :return:
        """
        # only search if it isn't already in the cache
        if (myclass, cmd) in self.command_tuple_cache:
            return self.command_tuple_cache[(myclass, cmd)]

        # run the search first in minidrone xml and then hit common if that failed
        project_id = int(self.minidrone_commands.project['id'])

        for child in self.minidrone_commands.project.myclass:
            if child['name'] == myclass:
                class_id = int(child['id'])
                #print child['name']

                for subchild in child.cmd:
                    #print subchild
                    if subchild['name'] == cmd:
                        #print subchild['name']
                        cmd_id = int(subchild['id'])

                        # cache the result
                        self.command_tuple_cache[(myclass, cmd)] = (project_id, class_id, cmd_id)
                        return (project_id, class_id, cmd_id)

        # do the search in common since minidrone failed
        project_id = int(self.common_commands.project['id'])

        for child in self.common_commands.project.myclass:
            if child['name'] == myclass:
                class_id = int(child['id'])
                #print child['name']

                for subchild in child.cmd:
                    #print subchild
                    if subchild['name'] == cmd:
                        #print subchild['name']
                        cmd_id = int(subchild['id'])

                        # cache the result
                        self.command_tuple_cache[(myclass, cmd)] = (project_id, class_id, cmd_id)
                        return (project_id, class_id, cmd_id)


    def _get_command_tuple_with_enum(self, myclass, cmd, enum_name):
        """
        Parses the command XML for the specified class name and command name and checks for enum_name

        :param myclass: class name (renamed to myclass to avoid reserved name) in the xml file
        :param cmd: command to execute (from XML file)
        :return:
        """
        print "get command tuple with enum"
        # only search if it isn't already in the cache
        if (myclass, cmd, enum_name) in self.command_tuple_cache:
            print "using the cache"
            print self.command_tuple_cache[(myclass, cmd, enum_name)]
            return self.command_tuple_cache[(myclass, cmd, enum_name)]

        # run the search first in minidrone xml and then hit common if that failed
        project_id = int(self.minidrone_commands.project['id'])

        for child in self.minidrone_commands.project.myclass:
            if child['name'] == myclass:
                class_id = int(child['id'])
                #print child['name']

                for subchild in child.cmd:
                    #print subchild
                    if subchild['name'] == cmd:
                        #print subchild['name']
                        cmd_id = int(subchild['id'])

                        for arg_child in subchild.arg:
                            if arg_child['type'] == "enum":
                                for e_idx, echild in enumerate(arg_child.enum):
                                    if echild['name'] == enum_name:
                                        enum_id = e_idx

                                        # cache the result
                                        self.command_tuple_cache[(myclass, cmd, enum_name)] = ((project_id, class_id, cmd_id), enum_id)

                                        print  ((project_id, class_id, cmd_id), enum_id)
                                        return ((project_id, class_id, cmd_id), enum_id)

        # common
        project_id = int(self.common_commands.project['id'])

        for child in self.common_commands.project.myclass:
            if child['name'] == myclass:
                class_id = int(child['id'])
                #print child['name']

                for subchild in child.cmd:
                    #print subchild
                    if subchild['name'] == cmd:
                        #print subchild['name']
                        cmd_id = int(subchild['id'])

                        for arg_child in subchild.arg:
                            if arg_child['type'] == "enum":
                                for e_idx, echild in enumerate(arg_child.enum):
                                    if echild['name'] == enum_name:
                                        enum_id = e_idx

                                        # cache the result
                                        self.command_tuple_cache[(myclass, cmd, enum_name)] = ((project_id, class_id, cmd_id), enum_id)

                                        print ((project_id, class_id, cmd_id), enum_id)
                                        return ((project_id, class_id, cmd_id), enum_id)


    def _safe_ble_write(self, characteristic, packet):
        """
        Write to the specified BLE characteristic but first ensure the connection is valid

        :param characteristic:
        :param packet:
        :return:
        """

        success = False

        while (not success):
            try:
                characteristic.write(packet)
                success = True
            except BTLEException:
                self._debug_print("reconnecting to send packet", 10)
                self._reconnect(3)


    def _send_command_packet_ack(self, packet):
        """
        Sends the actual packet on the ack channel.  Internal function only.

        :param packet: packet constructed according to the command rules (variable size, constructed elsewhere)
        :return: True if the command was sent and False otherwise
        """
        try_num = 0
        self._set_command_received('SEND_WITH_ACK', False)
        while (try_num < self.max_packet_retries and not self.command_received['SEND_WITH_ACK']):
            self._debug_print("sending command packet on try %d" % try_num, 2)
            self._safe_ble_write(characteristic=self.send_characteristics['SEND_WITH_ACK'], packet=packet)
            #self.send_characteristics['SEND_WITH_ACK'].write(packet)
            try_num += 1
            self._debug_print("sleeping for a notification", 2)
            #notify = self.drone.waitForNotifications(1.0)
            self.smart_sleep(0.5)
            #self._debug_print("awake %s " % notify, 2)

        return self.command_received['SEND_WITH_ACK']



    def _send_noparam_command_packet_ack(self, command_tuple):
        """
        Send a command on the ack channel - where all commands except PCMD go, per
        http://forum.developer.parrot.com/t/ble-characteristics-of-minidrones/5912/2

        the id of the last command sent (for use in ack) is the send counter (which is incremented before sending)

        Ensures the packet was received or sends it again up to a maximum number of times.

        :param command_tuple: 3 tuple of the command bytes.  0 padded for 4th byte
        :return: True if the command was sent and False otherwise
        """
        self.characteristic_send_counter['SEND_WITH_ACK'] = (self.characteristic_send_counter['SEND_WITH_ACK'] + 1) % 256
        packet = struct.pack("<BBBBBB", self.data_types['DATA_WITH_ACK'], self.characteristic_send_counter['SEND_WITH_ACK'],
                             command_tuple[0], command_tuple[1], command_tuple[2], 0)
        return self._send_command_packet_ack(packet)



    def _send_enum_command_packet_ack(self, command_tuple, enum_value, usb_id=None):
        """
        Send a command on the ack channel with enum parameters as well (most likely a flip).
        All commands except PCMD go on the ack channel per
        http://forum.developer.parrot.com/t/ble-characteristics-of-minidrones/5912/2

        the id of the last command sent (for use in ack) is the send counter (which is incremented before sending)

        :param command_tuple: 3 tuple of the command bytes.  0 padded for 4th byte
        :param enum_value: the enum index
        :return: nothing
        """
        self.characteristic_send_counter['SEND_WITH_ACK'] = (self.characteristic_send_counter['SEND_WITH_ACK'] + 1) % 256
        if (usb_id is None):
            packet = struct.pack("<BBBBBBI", self.data_types['DATA_WITH_ACK'], self.characteristic_send_counter['SEND_WITH_ACK'],
                                 command_tuple[0], command_tuple[1], command_tuple[2], 0,
                                 enum_value)
        else:
            self._debug_print(self.data_types['DATA_WITH_ACK'], self.characteristic_send_counter['SEND_WITH_ACK'],
                              command_tuple[0], command_tuple[1], command_tuple[2], 0, usb_id, enum_value, 1)
            packet = struct.pack("<BBBBBBBI", self.data_types['DATA_WITH_ACK'], self.characteristic_send_counter['SEND_WITH_ACK'],
                                 command_tuple[0], command_tuple[1], command_tuple[2], 0,
                                 usb_id, enum_value)
        return self._send_command_packet_ack(packet)


    def takeoff(self):
        """
        Sends the takeoff command to the mambo.  Gets the codes for it from the xml files.  Ensures the
        packet was received or sends it again up to a maximum number of times.

        :return: True if the command was sent and False otherwise
        """
        command_tuple = self._get_command_tuple("Piloting", "TakeOff")
        #print command_tuple
        return self._send_noparam_command_packet_ack(command_tuple)

    def safe_takeoff(self, timeout):
        """
        Sends commands to takeoff until the mambo reports it is taking off
        """
        
        start_time = time.time()
        # take off until it really listens
        while (self.sensors.flying_state != "takingoff" and (time.time() - start_time < timeout)):
            self.smart_sleep(1)
            success = self.takeoff()

        # now wait until it finishes takeoff before returning
        while ((self.sensors.flying_state != "flying" or self.sensors.flying_state != "hovering") and
               (time.time() - start_time < timeout)):
            self.smart_sleep(1)


    def land(self):
        """
        Sends the land command to the mambo.  Gets the codes for it from the xml files.  Ensures the
        packet was received or sends it again up to a maximum number of times.

        :return: True if the command was sent and False otherwise
        """
        command_tuple = self._get_command_tuple("Piloting", "Landing")
        #print command_tuple
        return self._send_noparam_command_packet_ack(command_tuple)

    def safe_land(self):
        """
        Ensure the mambo lands by sending the command until it shows landed on sensors
        """
        
        while (self.sensors.flying_state != "landed"):
            self._debug_print("trying to land", 10)
            self.smart_sleep(1)
            success = self.land()
        

    def hover(self):
        """
        Sends the command execute a flat trim to the mambo.  This is basically a hover command.
        Gets the codes for it from the xml files. Ensures the
        packet was received or sends it again up to a maximum number of times.

        :return: True if the command was sent and False otherwise
        """
        command_tuple = self._get_command_tuple("Piloting", "FlatTrim")
        #print command_tuple
        return self._send_noparam_command_packet_ack(command_tuple)


    def flip(self, direction):
        """
        Sends the flip command to the mambo.  Gets the codes for it from the xml files. Ensures the
        packet was received or sends it again up to a maximum number of times.
        Valid directions to flip are: front, back, right, left

        :return: True if the command was sent and False otherwise
        """
        if (direction not in ("front", "back", "right", "left")):
            print "Error: %s is not a valid direction.  Must be one of %s" % direction, "front, back, right, or left"
            print "Ignoring command and returning"
            return

        (command_tuple, enum_tuple) = self._get_command_tuple_with_enum("Animations", "Flip", direction)
        #print command_tuple
        #print enum_tuple

        return self._send_enum_command_packet_ack(command_tuple, enum_tuple)

    def turn_degrees(self, degrees):
        """
        Turn the mambo the specified number of degrees (-180, 180)

        This is called cap in the xml but it means degrees per
        http://forum.developer.parrot.com/t/what-does-cap-stand-for/6213/2

        :param degrees:
        :return:
        """
        command_tuple = self._get_command_tuple("Animations", "Cap")
        #print command_tuple
        self.characteristic_send_counter['SEND_WITH_ACK'] = (
                                                            self.characteristic_send_counter['SEND_WITH_ACK'] + 1) % 256
        packet = struct.pack("<BBBBBBh", self.data_types['DATA_WITH_ACK'],
                             self.characteristic_send_counter['SEND_WITH_ACK'],
                             command_tuple[0], command_tuple[1], command_tuple[2], 0,
                             degrees)

        return self._send_command_packet_ack(packet)

    def smart_sleep(self, timeout):
        """
        Sleeps the requested number of seconds but wakes up for notifications

        Note: NEVER use regular time.sleep!  It is a blocking sleep and it will likely
        cause the BLE to disconnect due to dropped notifications.  Always use smart_sleep instead!

        :param timeout: number of seconds to sleep
        :return:
        """

        start_time = time.time()
        while (time.time() - start_time < timeout):
            try:
                notify = self.drone.waitForNotifications(0.1)
            except:
                self._debug_print("reconnecting to wait", 10)
                self._reconnect(3)
            

    def turn_on_auto_takeoff(self):
        """
        Turn on the auto take off (throw mode)
        :return:
        """
        command_tuple = self._get_command_tuple("Piloting", "AutoTakeOffMode")
        #print command_tuple
        self.characteristic_send_counter['SEND_WITH_ACK'] = (
                                                            self.characteristic_send_counter['SEND_WITH_ACK'] + 1) % 256
        packet = struct.pack("<BBBBBBB", self.data_types['DATA_WITH_ACK'],
                             self.characteristic_send_counter['SEND_WITH_ACK'],
                             command_tuple[0], command_tuple[1], command_tuple[2], 0,
                             1)

        return self._send_command_packet_ack(packet)

    def take_picture(self):
        """
        Ask the drone to take a picture

        :return:
        """
        command_tuple = self._get_command_tuple("MediaRecord", "PictureV2")
        #print command_tuple
        return self._send_noparam_command_packet_ack(command_tuple)


    def ask_for_state_update(self):
        """
        Ask for a full state update (likely this should never be used but it can be called if you want to see
        everything the mambo is storing)

        :return: nothing but it will eventually fill the MamboSensors with all of the state variables as they arrive
        """
        command_tuple = self._get_command_tuple("Common", "AllStates")
        #print command_tuple
        return self._send_noparam_command_packet_ack(command_tuple)

    def _ensure_fly_command_in_range(self, value):
        """
        Ensure the fly direct commands are in range

        :param value: the value sent by the user
        :return: a value in the range -100 to 100
        """
        if (value < -100):
            return -100
        elif (value > 100):
            return 100
        else:
            return value

    def fly_direct(self, roll, pitch, yaw, vertical_movement, duration):
        """
        Direct fly commands using PCMD.  Each argument ranges from -100 to 100.  Numbers outside that are clipped
        to that range.

        Note that the xml refers to gaz, which is apparently french for vertical movements:
        http://forum.developer.parrot.com/t/terminology-of-gaz/3146

        :param roll:
        :param pitch:
        :param yaw:
        :param vertical_movement:
        :return:
        """

        my_roll = self._ensure_fly_command_in_range(roll)
        my_pitch = self._ensure_fly_command_in_range(pitch)
        my_yaw = self._ensure_fly_command_in_range(yaw)
        my_vertical = self._ensure_fly_command_in_range(vertical_movement)
        command_tuple = self._get_command_tuple("Piloting", "PCMD")

        start_time = time.time()
        while (time.time() - start_time < duration):
            self.characteristic_send_counter['SEND_NO_ACK'] = (self.characteristic_send_counter['SEND_NO_ACK'] + 1) % 256
            packet = struct.pack("<BBBBBBBbbbbI", self.data_types['DATA_NO_ACK'],
                                 self.characteristic_send_counter['SEND_NO_ACK'],
                                 command_tuple[0], command_tuple[1], command_tuple[2], 0,
                                 1, my_roll, my_pitch, my_yaw, my_vertical, 0)

            self._safe_ble_write(characteristic=self.send_characteristics['SEND_NO_ACK'], packet=packet)
            #self.send_characteristics['SEND_NO_ACK'].write(packet)
            notify = self.drone.waitForNotifications(0.1)
        

    def open_claw(self):
        """
        Open the claw
        :return: nothing
        """
        #print "open claw"
        (command_tuple, enum_tuple) = self._get_command_tuple_with_enum("UsbAccessory", "ClawControl", "OPEN")
        #print command_tuple
        #print enum_tuple

        return self._send_enum_command_packet_ack(command_tuple, enum_tuple, self.sensors.claw_id)

    def close_claw(self):
        """
        Open the claw
        :return: nothing
        """
        #print "close claw"
        (command_tuple, enum_tuple) = self._get_command_tuple_with_enum("UsbAccessory", "ClawControl", "CLOSE")
        #print command_tuple
        #print enum_tuple

        return self._send_enum_command_packet_ack(command_tuple, enum_tuple, self.sensors.claw_id)

    def fire_gun(self):
        """
        Fire the gun (assumes it is attached)

        :return: nothing
        """
        #print "firing gun"
        (command_tuple, enum_tuple) = self._get_command_tuple_with_enum("UsbAccessory", "GunControl", "FIRE")
        #print command_tuple
        #print enum_tuple

        return self._send_enum_command_packet_ack(command_tuple, enum_tuple, self.sensors.gun_id)

