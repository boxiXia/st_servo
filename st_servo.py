
import serial
import time
import struct

# Constants from SMS_STS.h and INST.h
SMS_STS_TORQUE_ENABLE = 40
SMS_STS_ACC = 41
SMS_STS_GOAL_POSITION_L = 42
SMS_STS_GOAL_POSITION_H = 43
SMS_STS_GOAL_TIME_L = 44
SMS_STS_GOAL_TIME_H = 45
SMS_STS_GOAL_SPEED_L = 46
SMS_STS_GOAL_SPEED_H = 47
SMS_STS_LOCK = 55

SMS_STS_MIN_ANGLE_LIMIT_L = 9
SMS_STS_MAX_ANGLE_LIMIT_L = 11
SMS_STS_OFS_L = 31
SMS_STS_MODE = 33

SMS_STS_PRESENT_POSITION_L = 56
SMS_STS_PRESENT_POSITION_H = 57
SMS_STS_PRESENT_SPEED_L = 58
SMS_STS_PRESENT_SPEED_H = 59
SMS_STS_PRESENT_LOAD_L = 60
SMS_STS_PRESENT_LOAD_H = 61
SMS_STS_PRESENT_VOLTAGE = 62
SMS_STS_PRESENT_TEMPERATURE = 63
SMS_STS_MOVING = 66
SMS_STS_PRESENT_CURRENT_L = 69
SMS_STS_PRESENT_CURRENT_H = 70

INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03
INST_REG_WRITE = 0x04
INST_REG_ACTION = 0x05
INST_SYNC_READ = 0x82
INST_SYNC_WRITE = 0x83


class StServo:
    def __init__(self, port, baudrate=1000000, timeout=0.1):
        self.serial = serial.Serial(port, baudrate, timeout=timeout)
        self.big_endian = True  # Based on End=0 analysis of SCS.cpp

    def close(self):
        self.serial.close()

    def _calc_checksum(self, id, length, instruction, params):
        # Optimized using sum() instead of loop
        return (~(id + length + instruction + sum(params))) & 0xFF

    def _write_packet(self, id, instruction, params):
        length = len(params) + 2
        checksum = self._calc_checksum(id, length, instruction, params)
        # Pre-allocate bytearray for better performance
        packet = bytearray(6 + len(params))
        packet[0] = packet[1] = 0xFF
        packet[2] = id
        packet[3] = length
        packet[4] = instruction
        packet[5:5+len(params)] = params
        packet[5+len(params)] = checksum
        self.serial.write(packet)
        # print(f"Sent: {[hex(x) for x in packet]}")

    def _read_packet(self):
        # Header 0xFF 0xFF
        header = self.serial.read(2)
        if len(header) < 2 or header[0] != 0xFF or header[1] != 0xFF:
            return None

        # ID, Length
        resp_info = self.serial.read(2)
        if len(resp_info) < 2:
            return None
        id, length = resp_info

        # Payload: Error + Params + Checksum
        # Length value is size of payload
        payload = self.serial.read(length)
        if len(payload) != length:
            # print(f"Payload incomplete: needed {length}, got {len(payload)}")
            return None

        checksum = payload[-1]
        data_bytes = payload[:-1]  # Error + Params

        # Verify checksum
        # Checksum is calculated on ID + Length + Error + Params
        pkt_bytes = list(resp_info) + list(data_bytes)

        s = sum(pkt_bytes)
        calculated = (~s) & 0xFF

        if calculated != checksum:
            # print(f"Checksum mismatch: calc {hex(calculated)} vs recv {hex(checksum)}")
            return None

        error = data_bytes[0]
        params = data_bytes[1:]

        # if error != 0:
        #     print(f"Servo Error bitmask: {hex(error)}")

        return id, error, params

    def sync_write(self, address, data_len, data_list):
        # packet: Header(2), ID(1=FE), Len(1), Instr(1=83), Addr(1), DataLen(1), [ID, Data...]..., Checksum(1)
        # Length = 4 (Instr, Addr, DataLen, Checksum) + N * (data_len + 1)
        # Note: data_list is list of tuples (id, [data_bytes])

        flat_data = []
        for sid, data in data_list:
            flat_data.append(sid)
            flat_data.extend(data)

        params = [address, data_len] + flat_data

        # ID 0xFE (254) for Broadcast/Sync
        self._write_packet(254, INST_SYNC_WRITE, params)

    def write_register(self, id, address, data, wait_response=True):
        # Instruction WRITE
        # Params: Address, Data...
        params = [address] + list(data)
        self._write_packet(id, INST_WRITE, params)
        if wait_response:
            return self._read_packet()
        return None

    def read_register(self, id, address, length):
        # Instruction READ
        # Params: Address, Length
        params = [address, length]
        self._write_packet(id, INST_READ, params)
        ret = self._read_packet()
        if ret is None:
            return None
        _, err, data = ret
        if err != 0:
            # print(f"Read Error 0x{err:02X}")
            pass
        return data

    def read_byte(self, id, address):
        data = self.read_register(id, address, 1)
        if data and len(data) >= 1:
            return data[0]
        return None

    def read_word(self, id, address):
        data = self.read_register(id, address, 2)
        if data and len(data) >= 2:
            return data[0] + (data[1] << 8)
        return None

    def read_s16(self, id, address):
        val = self.read_word(id, address)
        if val is None:
            return None
        if val > 32767:
            val -= 65536
        return val

    def set_position(self, id, position, time=0, speed=0):
        # Support list of IDs for Sync Write
        # If id is int: normal write
        # If id is list: sync write

        # Prepare data function
        def get_data(pos):
            pos_l = pos & 0xFF
            pos_h = (pos >> 8) & 0xFF
            time_l = time & 0xFF
            time_h = (time >> 8) & 0xFF
            speed_l = speed & 0xFF
            speed_h = (speed >> 8) & 0xFF
            return [pos_l, pos_h, time_l, time_h, speed_l, speed_h]

        if isinstance(id, int):
            data = get_data(position)
            return self.write_register(id, SMS_STS_GOAL_POSITION_L, data)
        elif isinstance(id, list) or isinstance(id, tuple):
            # Sync write
            # Handle if position is single value (broadcast) or list
            data_list = []
            if isinstance(position, list) or isinstance(position, tuple):
                if len(position) != len(id):
                    print("Error: Length of IDs and Positions must match")
                    return
                for i, sid in enumerate(id):
                    data_list.append((sid, get_data(position[i])))
            else:
                d = get_data(position)
                for sid in id:
                    data_list.append((sid, d))

            self.sync_write(SMS_STS_GOAL_POSITION_L, 6, data_list)

    def set_mode(self, id, mode):
        # 0: Position, 1: Wheel (Speed)
        if isinstance(id, int):
            return self.write_register(id, SMS_STS_MODE, [mode], wait_response=True)
        else:
            # Sync write - no response expected
            data_list = []
            for sid in id:
                data_list.append((sid, [mode]))
            self.sync_write(SMS_STS_MODE, 1, data_list)

    def set_speed(self, id, speed):
        # Set speed for Wheel Mode
        # Register 46 (Running Speed)
        # Bit 15 controls direction: 0=CW, 1=CCW
        # Valid range: 0-32767 steps/s (Physically limited by motor, approx 4000)

        def get_data(spd):
            direction = 0
            if spd < 0:
                direction = 1
                spd = -spd

            # Limit to 15-bit max (32767)
            if spd > 32767:
                spd = 32767

            if direction:
                spd |= 0x8000

            speed_l = spd & 0xFF
            speed_h = (spd >> 8) & 0xFF
            return [speed_l, speed_h]

        if isinstance(id, int):
            return self.write_register(id, SMS_STS_GOAL_SPEED_L, get_data(speed))
        else:
            data_list = []
            if isinstance(speed, list) or isinstance(speed, tuple):
                for i, sid in enumerate(id):
                    data_list.append((sid, get_data(speed[i])))
            else:
                d = get_data(speed)
                for sid in id:
                    data_list.append((sid, d))
            self.sync_write(SMS_STS_GOAL_SPEED_L, 2, data_list)

    def get_position(self, id):
        return self.read_s16(id, SMS_STS_PRESENT_POSITION_L)

    def get_speed(self, id):
        return self.read_s16(id, SMS_STS_PRESENT_SPEED_L)

    def get_load(self, id):
        return self.read_s16(id, SMS_STS_PRESENT_LOAD_L)

    def get_voltage(self, id):
        # Returns byte? Manual says value is voltage * 10?
        # SMS_STS.h: SMS_STS_PRESENT_VOLTAGE 62 (1 byte)
        v = self.read_byte(id, SMS_STS_PRESENT_VOLTAGE)
        if v is not None:
            # Usually unit is 0.1V
            return v
        return None

    def get_temperature(self, id):
        return self.read_byte(id, SMS_STS_PRESENT_TEMPERATURE)

    def enable_torque(self, id, enable):
        val = 1 if enable else 0
        if isinstance(id, int):
            return self.write_register(id, SMS_STS_TORQUE_ENABLE, [val], wait_response=True)
        else:
            # Sync write - no response expected
            data_list = []
            for sid in id:
                data_list.append((sid, [val]))
            self.sync_write(SMS_STS_TORQUE_ENABLE, 1, data_list)

    def unlock_eprom(self, id):
        return self.write_register(id, SMS_STS_LOCK, [0])

    def lock_eprom(self, id):
        return self.write_register(id, SMS_STS_LOCK, [1])

    def write_id(self, id, new_id):
        self.unlock_eprom(id)
        # SMS_STS_ID = 5
        ret = self.write_register(id, 5, [new_id])
        self.lock_eprom(new_id)
        return ret

    def ping(self, id):
        self._write_packet(id, INST_PING, [])
        return self._read_packet()

    def scan(self, start_id=0, end_id=253):
        found = []
        original_timeout = self.serial.timeout
        self.serial.timeout = 0.004  # 4ms timeout (1ms was too fast)
        self.serial.reset_input_buffer()  # Flush old data (e.g. from broadcast ping)

        for i in range(start_id, end_id + 1):
            try:
                # Use ping
                self._write_packet(i, INST_PING, [])
                ret = self._read_packet()
                if ret is not None:
                    found.append(i)
            except Exception:
                pass

        self.serial.timeout = original_timeout
        return found


def main():
    port = '/dev/ttyACM0'
    print(f"Connecting to {port}...")
    try:
        servo = StServo(port)
    except Exception as e:
        print(f"Failed to open port: {e}")
        return

    # IDs to test. Currently testing with ID 1 as discovered.
    # Add more IDs here if you have multiple servos connected.
    ids = [1, 2, 3, 4]

    print("--------------------------------------------------")
    print(" READING INITIAL STATUS ")
    print("--------------------------------------------------")
    for id in ids:
        ping_resp = servo.ping(id)
        if ping_resp is None:
            print(f"ID {id}: Ping Failed")
            continue

        pos = servo.get_position(id)
        volt = servo.get_voltage(id)  # 0.1V units usually
        temp = servo.get_temperature(id)
        mode = servo.read_byte(id, SMS_STS_MODE)

        print(f"ID {id}: Ping OK")
        print(f"  Position: {pos}")
        print(f"  Voltage:  {volt/10.0 if volt else '?'} V")
        print(f"  Temp:     {temp} C")
        print(f"  Mode:     {mode} (0=Pos, 1=Wheel)")
        print("")

    print("--------------------------------------------------")
    print(" POSITION MODE TEST ")
    print("--------------------------------------------------")
    # Ensure Position Mode
    servo.set_mode(ids, 0)
    servo.enable_torque(ids, True)

    time.sleep(0.5)

    # Move to 0
    print("Moving to 0 (Time=1000ms)...")
    servo.set_position(ids, 0, time=1000)
    time.sleep(1.5)
    for id in ids:
        print(f"ID {id} Pos: {servo.get_position(id)}")

    # Move to 4096
    print("Moving to 4096 (Time=1000ms)...")
    servo.set_position(ids, 4096, time=1000)
    time.sleep(1.5)
    for id in ids:
        print(f"ID {id} Pos: {servo.get_position(id)}")

    # Move to Center (2048)
    print("Moving to 2048 (Time=1000ms)...")
    servo.set_position(ids, 2048, time=1000)

    # Track while moving
    start = time.time()
    while time.time() - start < 1.2:
        for id in ids:
            pos = servo.get_position(id)
            print(f"[{time.time()-start:.2f}s] ID {id} Pos: {pos}")
        time.sleep(0.2)

    print("--------------------------------------------------")
    print(" WHEEL MODE TEST ")
    print("--------------------------------------------------")
    # Switch to Wheel Mode
    servo.set_mode(ids, 1)
    # servo.enable_torque(ids, True) # Should already be enabled

    time.sleep(0.1)

    print("Speed 32767...")
    servo.set_speed(ids, 32767)
    time.sleep(2.0)

    print("Speed -32767...")
    servo.set_speed(ids, -32767)
    time.sleep(2.0)

    print("Stop (Speed 0)...")
    servo.set_speed(ids, 0)
    time.sleep(1.0)

    print("--------------------------------------------------")
    print(" RESTORING ")
    print("--------------------------------------------------")
    servo.set_mode(ids, 0)  # Back to position mode
    servo.enable_torque(ids, False)  # Optional: disable torque

    print("Done.")


if __name__ == '__main__':
    main()
