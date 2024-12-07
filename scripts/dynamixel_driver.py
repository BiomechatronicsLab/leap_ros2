#!/usr/bin/env python3

import dynamixel_sdk as dxl
import numpy as np

ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4
ADDR_PRESENT_VELOCITY = 128
LEN_PRESENT_VELOCITY = 4
ADDR_PRESENT_CURRENT = 126
LEN_PRESENT_CURRENT = 2
ADDR_GOAL_POSITION = 116
LEN_GOAL_POSITION = 4
ADDR_P_GAIN = 84
LEN_P_GAIN = 2
ADDR_I_GAIN = 82
LEN_I_GAIN = 2
ADDR_D_GAIN = 80
LEN_D_GAIN = 2
ADDR_CURRENT_LIMIT = 38
LEN_CURRENT_LIMIT = 2
ADDR_TORQUE_ENABLE = 64
LEN_TORQUE_ENABLE = 1
ADDR_OPERATING_MODE = 11
LEN_OPERATING_MODE = 1
ADDR_BAUD_RATE = 8
LEN_BAUD_RATE = 1

# TODO: maybe paramterize this? TBD!
PROTOCOL_VERSION = 2.0

class DynamixelDriver():
    def __init__(self, motor_ids, baudrate, device_name, kP, kI, kD, curr_lim):
        self.baudrate = baudrate
        self.device_name = device_name
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.curr_lim  = curr_lim
        self.motor_ids = motor_ids # list(range(16)) # DYNAMIXEL MOTOR IDs

        # Initialize PortHandler and PacketHandler instances
        self.port_handler = dxl.PortHandler(self.device_name)
        self.packet_handler = dxl.PacketHandler(PROTOCOL_VERSION)

        # Open port
        if not self.port_handler.openPort():
            print("Failed to open the port")

        # Set port baudrate
        if not self.port_handler.setBaudRate(self.baudrate):
            print("Failed to change the baudrate")

        # Initialize GroupSyncRead instances for position, velocity, current (which are common)
        self.group_sync_read_position = dxl.GroupSyncRead(
            self.port_handler,
            self.packet_handler,
            ADDR_PRESENT_POSITION,
            LEN_PRESENT_POSITION,
        )

        self.group_sync_read_velocity = dxl.GroupSyncRead(
            self.port_handler,
            self.packet_handler,
            ADDR_PRESENT_VELOCITY,
            LEN_PRESENT_VELOCITY,
        )

        self.group_sync_read_current = dxl.GroupSyncRead(
            self.port_handler,
            self.packet_handler,
            ADDR_PRESENT_CURRENT,
            LEN_PRESENT_CURRENT,
        )

        # Initialize GroupSyncWrite instance for position commands
        self.group_sync_write_position = dxl.GroupSyncWrite(
            self.port_handler,
            self.packet_handler,
            ADDR_GOAL_POSITION,
            LEN_GOAL_POSITION,
        )

        # Add parameter storage for Dynamixel IDs 0-15
        for dxl_id in self.motor_ids:
            if not self.group_sync_read_position.addParam(dxl_id):
                print(
                    f"Failed to add param for Dynamixel ID: {dxl_id} (position)"
                )
            if not self.group_sync_read_velocity.addParam(dxl_id):
                print(
                    f"Failed to add param for Dynamixel ID: {dxl_id} (velocity)"
                )
            if not self.group_sync_read_current.addParam(dxl_id):
                print(
                    f"Failed to add param for Dynamixel ID: {dxl_id} (current)"
                )

        # # Initialize gains and operating mode
        # self.initialize_gains()

    def __del__(self):
        # Torque off the motors
        self.set_torque_enable(self.motor_ids, np.zeros(len(self.motor_ids)))

        # Close port
        if self.port_handler.is_open:
            self.port_handler.closePort()


    def sync_write(self, ids, values, address, length, group_sync_write=None):
        if group_sync_write == None:
            group_sync_write = dxl.GroupSyncWrite(
                self.port_handler, self.packet_handler, address, length
            )
        for i, dxl_id in enumerate(ids):

            # Dynamixel expects 32bit values
            param = [
                dxl.DXL_LOBYTE(dxl.DXL_LOWORD(int(values[i]))),
                dxl.DXL_HIBYTE(dxl.DXL_LOWORD(int(values[i]))),
                dxl.DXL_LOBYTE(dxl.DXL_HIWORD(int(values[i]))),
                dxl.DXL_HIBYTE(dxl.DXL_HIWORD(int(values[i]))),
            ]

            # Add parameters based on the length of the number of bits to send (based on register size)
            if not group_sync_write.addParam(dxl_id, param[:length]): 
                print(
                    f"Failed to add param for Dynamixel ID: {dxl_id} at address {address}"
                )
        dxl_comm_result = group_sync_write.txPacket()
        if dxl_comm_result != dxl.COMM_SUCCESS:
            print(
                f"GroupSyncWrite txPacket failed at address {address}: {self.packet_handler.getTxRxResult(dxl_comm_result)}"
            )
        group_sync_write.clearParam()

    ### Setters
    def set_torque_enable(self, motor_ids, values):
        try:
            # Enable torque for all motors
            self.sync_write(
                motor_ids, values, ADDR_TORQUE_ENABLE, LEN_TORQUE_ENABLE
            )
        except Exception as e:
            print(f"Error Enabling Torques: {str(e)}")

    def set_operating_mode(self, motor_ids, values):
        try:
            # Enable position-current control mode and default parameters
            self.sync_write(
                motor_ids, values, ADDR_OPERATING_MODE, LEN_OPERATING_MODE
            )
        except Exception as e:
            print(f"Error Enabling Position Control: {str(e)}")

    def set_kP(self, motor_ids, values):
        try:
            self.sync_write(
                motor_ids, values, ADDR_P_GAIN, LEN_P_GAIN
            )  # P gain stiffness

        except Exception as e:
            print(f"Error Setting kP Gains: {str(e)}")

    def set_kI(self, motor_ids, values):
        try:
            self.sync_write(
                motor_ids, values, ADDR_I_GAIN, LEN_I_GAIN
            )  # P gain stiffness

        except Exception as e:
            print(f"Error Setting kI Gains: {str(e)}")

    def set_kD(self, motor_ids, values):
        try:
            self.sync_write(
                motor_ids, values, ADDR_D_GAIN, LEN_D_GAIN
            )  # P gain stiffness

        except Exception as e:
            print(f"Error Setting kD Gains: {str(e)}")

    def set_current_limit(self, motor_ids, values):
        try:
            self.sync_write(
                motor_ids, values, ADDR_CURRENT_LIMIT, LEN_CURRENT_LIMIT
            ) 

        except Exception as e:
            print(f"Error Setting Current Limits: {str(e)}")

    def set_goal_position(self, motor_ids, values):
        try:
            self.sync_write(
                motor_ids, values, ADDR_GOAL_POSITION, 4, self.group_sync_write_position
            ) 

        except Exception as e:
            print(f"Error Setting Goal Position: {str(e)}")

    ### Getters

    def sync_read(self, ids, address, length, group_sync_read=None):

        data_arr = [] # Appending results to data arr corresponding to queried ids 
        if group_sync_read == None:
            group_sync_read = dxl.GroupSyncRead(self.port_handler,
                                self.packet_handler,
                                address,
                                length,
                            )
        for dxl_id in ids:
            group_sync_read.addParam(dxl_id)

        dxl_comm_result = group_sync_read.txRxPacket()
        if dxl_comm_result != dxl.COMM_SUCCESS:
            print(
                f"GroupSyncRead txRxPacket failed: {self.packet_handler.getTxRxResult(dxl_comm_result)}"
            )

        for dxl_id in ids:
            data = group_sync_read.getData(
                dxl_id, address, length
            )
            data_arr.append(data)
        
        group_sync_read.clearParam()
        return data_arr

    def get_kP(self, motor_ids):
        kP_arr = self.sync_read(motor_ids, ADDR_P_GAIN, LEN_P_GAIN)
        return kP_arr
    
    def get_kI(self, motor_ids):
        kI_arr = self.sync_read(motor_ids, ADDR_I_GAIN, LEN_I_GAIN)
        return kI_arr

    def get_kD(self, motor_ids):
        kD_arr = self.sync_read(motor_ids, ADDR_D_GAIN, LEN_D_GAIN)
        return kD_arr

    def get_current_limit(self, motor_ids):
        current_limit_arr = self.sync_read(motor_ids, ADDR_CURRENT_LIMIT, LEN_CURRENT_LIMIT)
        return current_limit_arr
    
    def get_torque_enable(self, motor_ids):
        torque_enable_arr = self.sync_read(motor_ids, ADDR_TORQUE_ENABLE, LEN_TORQUE_ENABLE)
        return torque_enable_arr

    def get_operating_mode(self, motor_ids):
        operating_mode_arr = self.sync_read(motor_ids, ADDR_OPERATING_MODE, LEN_OPERATING_MODE)
        return operating_mode_arr

    def get_position(self, motor_ids):
        pos_ticks_arr = self.sync_read(motor_ids, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION, self.group_sync_read_position)
        return pos_ticks_arr
    
    def get_velocity(self, motor_ids):
        vel_ticks_arr = self.sync_read(motor_ids, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY, self.group_sync_read_velocity)        
        return vel_ticks_arr
    
    def get_current(self, motor_ids):
        curr_arr = self.sync_read(motor_ids, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT, self.group_sync_read_current)        
        return curr_arr

    def get_baud_rate(self, motor_ids):
        baud_rate_arr = self.sync_read(motor_ids, ADDR_BAUD_RATE, LEN_BAUD_RATE)
        return baud_rate_arr

    def initialize_gains(self):

        # Need to turn off torque to change operating mode
        self.set_torque_enable(self.motor_ids, np.zeros(len(self.motor_ids))) # Set Torque on for the motor_ids
        self.set_operating_mode(self.motor_ids, 3.0*np.ones(len(self.motor_ids))) # Enable position control, #3 
        self.set_torque_enable(self.motor_ids, np.ones(len(self.motor_ids))) # Set Torque on for the motor_ids
        kP = np.ones(len(self.motor_ids)) * self.kP

        # TODO: PARAMTERIZE THIS "SCALING FACTOR"
        # P gain stiffness for side to side should be a bit less?
        # kP[[0, 4, 8]] = np.ones(3) * (self.kP * 0.75) 
        self.set_kP(self.motor_ids, kP) # Set kP gains

        kI = np.ones(len(self.motor_ids)) * self.kI
        # I gain stiffness for side to side should be a bit less?
        # kI[[0, 4, 8]] = np.ones(3) * (self.kI * 0.75)
        self.set_kI(self.motor_ids, kI) # Set kI gains

        kD = np.ones(len(self.motor_ids)) * self.kD
        # I gain stiffness for side to side should be a bit less?
        # kD[[0, 4, 8]] = np.ones(3) * (self.kD * 0.75)
        self.set_kD(self.motor_ids, kD)

        if self.curr_lim > 1750:
            print("The requested input for current limit is greater than its maximum allowable, which is 1750mA.")
            self.curr_lim = 1750
        self.set_current_limit(self.motor_ids, np.ones(len(self.motor_ids)) * self.curr_lim)

    def ticks_to_degrees(self, ticks):
        return (ticks - 2048) * 0.088

    def ticks_to_degrees_list(self, ticks_list):
        return [self.ticks_to_degrees(tick) for tick in ticks_list]


    def degrees_to_ticks(self, degrees):
        return int(degrees / 0.088) + 2048

    def degrees_to_ticks_list(self, degrees_list):
        return [self.degrees_to_ticks(deg) for deg in degrees_list]