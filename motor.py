from package import Package
from commands import Commands
import time
import threading
import serial
import random


class Motor:
    def __init__(self, id, is_can=False, can_ids=[], serialport=None):
        self.id = id
        self.is_can = is_can
        self.can_ids = can_ids
        self.offset = 0
        self.homed = False
        self.rotations = 0
        
        self.temp_fet = -1
        self.temp_motor = -1
        self.avg_motor_current = -1
        self.avg_motor_in = -1
        self.avg_id = -1
        self.avg_iq = -1
        self.duty_now = -1
        self.rpm = -1
        self.v_in = -1
        self.amp_hours = -1
        self.amp_hours_charged = -1
        self.watt_hours = -1
        self.watt_hours_charged = -1
        self.tachometer = -1
        self.tachometer_abs = -1
        self.mc_fault_code = -1
        self.pid_pos_now = -1
        self.actual_pos = -1
        self.last_clamped_pos = 0
        self.clamped_pos = None
        self.pos = -1
        self.temp_mos1 = -1
        self.temp_mos2 = -1
        self.temp_mos3 = -1
        self.avg_vd = -1
        self.avg_vq = -1
        self.status = -1

        if not self.is_can:
            self.heart_beat_thread = threading.Thread(target=self.heartbeat_func, args=(serialport,))
            self.heart_beat_thread.start()
        
        self.shutdown = False
        self.test_pos = -1

    def heartbeat_func(self, serialport):
        with serial.Serial(serialport, baudrate=115200, timeout=0.05) as ser:
            while not self.shutdown:
                package = Package()
                package.encode_command(Commands.COMM_ALIVE, [])
                package.send(ser)

                for can_id in self.can_ids:
                    package = Package()
                    package.encode_command(Commands.COMM_FORWARD_CAN, [can_id, 30])
                    package.send(ser)

                time.sleep(0.1)

    def print_values(self):
        print("temp_fet: ", self.temp_fet)
        print("temp_motor: ", self.temp_motor)
        print("avg_motor_current: ", self.avg_motor_current)
        print("avg_motor_in: ", self.avg_motor_in)
        print("avg_id: ", self.avg_id)
        print("avg_iq: ", self.avg_iq)
        print("duty_now: ", self.duty_now)
        print("rpm: ", self.rpm)
        print("v_in: ", self.v_in)
        print("amp_hours: ", self.amp_hours)
        print("amp_hours_charged: ", self.amp_hours_charged)
        print("watt_hours: ", self.watt_hours)
        print("watt_hours_charged: ", self.watt_hours_charged)
        print("tachometer: ", self.tachometer)
        print("tachometer_abs: ", self.tachometer_abs)
        print("mc_fault_code: ", self.mc_fault_code)
        print("pid_pos_now: ", self.pid_pos_now)
        print("temp_mos1: ", self.temp_mos1)
        print("temp_mos2: ", self.temp_mos2)
        print("temp_mos3: ", self.temp_mos3)
        print("avg_vd: ", self.avg_vd)
        print("avg_vq: ", self.avg_vq)
        print("status: ", self.status)

    def get_values(self, serial):
        package = Package()
        package.encode_command(Commands.COMM_SET_DETECT, [0])
        package.send(serial)

        package = Package()
        package.encode_command(Commands.COMM_GET_VALUES, [])
        package.send(serial)

        buffer = []
        terminate = False
        while not terminate:
            for byte in serial.read():
                buffer.append(byte)

            package = Package()
            valid, buffer = package.getFromBuffer(buffer)

            if valid and package.payload[0] == 4:
                terminate = True

        package.decode([self])

        if self.clamped_pos != None and self.last_clamped_pos != None and self.homed and self.last_clamped_pos != self.offset:
            delta = self.clamped_pos - self.last_clamped_pos
            if delta > 180:
                self.rotations -= 1
                self.pos -= 360
            elif delta < -180:
                self.rotations += 1
                self.pos += 360

    def set_duty(self, serial, duty):
        package = Package()
        if self.is_can:
            package.encode_command(Commands.COMM_FORWARD_CAN, [self.id, Commands.COMM_SET_DUTY.value, int(duty*1e5)])
        else:
            package.encode_command(Commands.COMM_SET_DUTY, [int(duty*1e5)])
        package.send(serial)

    def set_pos(self, serial, pos):
        self.get_values(serial)
        package = Package()
        package.encode_command(Commands.COMM_SET_POS, [int(((pos+self.offset)%360)*1e6)])
        package.send(serial)
        return True

    def go_to_pos(self, serial, pos, degrees_per_step=0.5, velocity=40, velocity_rampsteps=100):
        self.get_values(serial)
        if abs(self.pos - pos) < 2 * degrees_per_step:
            print("Already at position!")
            return

        mid_pos = self.pos
        mid_positions = [[mid_pos, 0]]
        total_time = 0

        if self.pos < pos:
            while mid_pos < pos:
                current_step = len(mid_positions)

                if current_step <= velocity_rampsteps:
                    current_velocity = (velocity / velocity_rampsteps) * current_step
                else:
                    current_velocity = velocity

                time_increment = degrees_per_step / current_velocity
                total_time += time_increment

                mid_pos += degrees_per_step
                mid_positions.append([mid_pos, total_time])
        else:
            while mid_pos > pos:
                current_step = len(mid_positions)

                if current_step <= velocity_rampsteps:
                    current_velocity = (velocity / velocity_rampsteps) * current_step
                else:
                    current_velocity = velocity

                time_increment = degrees_per_step / current_velocity
                total_time += time_increment

                mid_pos -= degrees_per_step
                mid_positions.append([mid_pos, total_time])

        if mid_positions[-1][0] != pos:
            mid_positions[-1][0] = pos

        print("Starting movement", self.pos, self.actual_pos, self.offset)

        start_time = time.perf_counter()
        for position, time_stamp in mid_positions:
            if time_stamp > time.perf_counter() - start_time:
                time.sleep(max(time_stamp - (time.perf_counter() - start_time), 0))
            else:
                continue
            
            tries = 1
            success = self.set_pos(serial, position)
            while not success and tries < 3:
                success = self.set_pos(serial, position)
                tries += 1

        self.get_values(serial)
        print("Final position: ", self.pos, self.actual_pos, self.offset, "Average speed: ", (mid_positions[-1][0] - mid_positions[0][0]) / (time.perf_counter() - start_time))
        print("Error: ", abs((self.pos) - pos))

    def do_homing(self, serial):
        self.get_values(serial)
        self.offset = self.pid_pos_now
        self.homed = True
        