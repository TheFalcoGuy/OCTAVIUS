#!/usr/bin/python
"""
Python script to control the Odrives on Hanjun's Superlimb
Roberto Bolli Jr.

Based on work from Daniel J. Gonzalez, Rachel Hoffman-Bice and Jerry Ng
for 2.12 Intro to Robotics
"""

import odrive
from odrive.enums import *
import time
import math
import fibre
import serial
import struct
import signal
import sys
import pdb

#Useful constants
pi = 3.1415927
in2mm = 25.4
mm2in = 1/in2mm
in2m = in2mm/1000
Nm2A = 0.00000604

imu_msgs_recieved = 0
initial_shoulder_vert_angle = 1000
initial_elbow_horiz_angle = 1000
initial_elbow_vert_angle = 1000
superlimb_base_angle = 0
superlimb_shoulder_angle = 0
superlimb_elbow_angle = 0

class Odrive:
    def __init__(self, usb_serial, axes, kp, kd, full_init=False):
        self.usb_serials=usb_serial
        self.axes = axes
        self.CPR2RAD = (2*math.pi/400000)
        self.connect_odrive_serial()
        if full_init == True:
            self.full_init()
        self.set_gains(kp, kd)
        self.is_angle_control_set = False

    def connect_odrive_serial(self):
        #Connects to odrives of specified serial ids
        print("Finding odrive: ", self.usb_serials, "...")
        odrv = odrive.find_any(serial_number = self.usb_serials)
        print("Found odrive!")
        self.odrv=odrv
        self.odrv.clear_errors()

    #Error checking print functions
    def print_controllers(self):
        print(' axis 0 controller: ', self.odrv.axis0.controller)
        print(' axis 1 controller: ', self.odrv.axis1.controller)

    def print_encoders(self):
        print(' axis 0 controller: ', self.odrv.axis0.encoder)
        print(' axis 1 controller: ', self.odrv.axis1.encoder)

    def printErrorStates(self):
        print(' axis 0 error:',hex(self.odrv.axis0.error))
        print( ' motor 0 error:',hex(self.odrv.axis0.motor.error))
        print( ' encoder 0 error:',hex(self.odrv.axis0.encoder.error))
        print(' axis 1 error:',hex(self.odrv.axis1.error))
        print( ' motor 1 error:',hex(self.odrv.axis1.motor.error))
        print( ' encoder 1 error:',hex(self.odrv.axis1.encoder.error))

    def printPos(self):
        print(' pos_estimate: ', self.odrv.axis1.encoder.pos_estimate)
        print(' count_in_cpr: ',self.odrv.axis1.encoder.count_in_cpr)
        print(' shadow_count: ', self.odrv.axis1.encoder.shadow_count)

    def print_all(self):
        self.printErrorStates()
        self.print_encoders()
        self.print_controllers()

    def reboot(self):
        #Reboot and reconnect function
        self.odrv.reboot()
        time.sleep(5)
        connect_odrive_serial()
        print('Rebooted ')
        
    def pos_control(self):
        if self.is_angle_control_set == False:
            print('Changing mode to position control.')
            if self.axes[0]:
                self.odrv.axis0.requested_state = AXIS_STATE_IDLE
                self.odrv.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
                self.odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

            if self.axes[1]:
                self.odrv.axis1.requested_state = AXIS_STATE_IDLE
                self.odrv.axis1.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
                self.odrv.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            time.sleep(1)
            
            self.is_angle_control_set = True


    def pos_move(self, desired_angle): #Desired position in turns/gear ratio (in our case, turns/100)
        self.pos_control()
        if self.axes[0]:
            try:
                self.odrv.axis0.controller.input_pos = desired_angle[0]
            except AttributeError:
                print('Failed to move axis 0')

        if self.axes[1]:
            try:
                self.odrv.axis1.controller.input_pos = desired_angle[1]
            except AttributeError:
                print('Failed to move axis 1')
            

    def set_idle(self, axes):
        if self.axes[0]:
            self.odrv.axis0.requested_state = AXIS_STATE_IDLE
        if self.axes[1]:
            self.odrv.axis1.requested_state = AXIS_STATE_IDLE


    def erase_and_reboot(self):
        #Erase the configuration of the system and reboots
        print('erasing config')
        self.odrv.erase_configuration()
        print('reboot')
        self.odrv.reboot()


    def startup_init(self):
        print('Initializing encoder calibration sequence')

        if self.axes[0]:
            self.odrv.axis0.requested_state = AXIS_STATE_IDLE
            time.sleep(1)
            self.odrv.axis0requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
            time.sleep(10)
            self.odrv.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
            time.sleep(10)
            self.odrv.axis0.requested_state = AXIS_STATE_IDLE
            time.sleep(1)

        if self.axes[1]:
            self.odrv.axis1.requested_state = AXIS_STATE_IDLE
            time.sleep(1)
            self.odrv.axis1.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
            time.sleep(10)
            self.odrv.axis1.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
            time.sleep(10)
            self.odrv.axis1.requested_state = AXIS_STATE_IDLE
            time.sleep(1)

    def full_init(self):
        self.printErrorStates()
        if self.axes[0]:
            self.odrv.axis0.motor.config.pre_calibrated = False
            #pole pairs
            self.odrv.axis0.motor.config.pole_pairs = 4
            self.odrv.axis0.controller.config.vel_limit = 50 
            self.odrv.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
            self.odrv.axis0.encoder.config.cpr = 4000
            self.odrv.axis0.encoder.config.use_index = True
            self.odrv.axis0.encoder.config.pre_calibrated = False
            #motor calibration current
            self.odrv.axis0.motor.config.calibration_current = 4

            time.sleep(1)
            self.printErrorStates()
            self.odrv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            print('Axis 0 calibration sequence in progress')
            time.sleep(20)
            self.printErrorStates()
            self.odrv.axis0.motor.config.pre_calibrated=True
            self.odrv.axis0.config.startup_encoder_index_search = True
            self.odrv.axis0.config.startup_encoder_offset_calibration = True
            self.odrv.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL

        if self.axes[1]:
            self.odrv.axis1.motor.config.pre_calibrated = False
            #pole pairs
            self.odrv.axis1.motor.config.pole_pairs = 4
            self.odrv.axis1.controller.config.vel_limit = 50
            self.odrv.axis1.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
            self.odrv.axis1.encoder.config.cpr = 4000
            self.odrv.axis1.encoder.config.use_index = True
            self.odrv.axis1.encoder.config.pre_calibrated = False
            #motor calibration current
            self.odrv.axis1.motor.config.calibration_current = 4

            time.sleep(1)
            self.printErrorStates()
            self.odrv.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            print('Axis 1 calibration sequence in progress')
            time.sleep(20)
            self.printErrorStates()
            self.odrv.axis1.motor.config.pre_calibrated=True
            self.odrv.axis1.config.startup_encoder_index_search = True
            self.odrv.axis1.config.startup_encoder_offset_calibration = True
            self.odrv.axis1.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL

        #self.odrv.save_configuration()
        time.sleep(2)
        print('Finished full calibration')
        self.printErrorStates()


    def set_gains(self, kp, kd, ki = [0, 0], make_permanent = 0):
        if self.axes[0]:
            self.odrv.axis0.requested_state=AXIS_STATE_IDLE
            self.odrv.axis0.controller.config.pos_gain = kp[0]
            self.odrv.axis0.controller.config.vel_gain = kd[0]
            self.odrv.axis0.controller.config.vel_integrator_gain = ki[0]

        if self.axes[1]:
            self.odrv.axis1.requested_state=AXIS_STATE_IDLE
            self.odrv.axis1.controller.config.pos_gain = kp[1]
            self.odrv.axis1.controller.config.vel_gain = kd[1]
            self.odrv.axis1.controller.config.vel_integrator_gain = ki[1]

        time.sleep(1)
        if make_permanent==1:
            self.odrv.save_configuration()
            
if __name__ == '__main__':
    board_0_driver = Odrive(usb_serial = '2059387F304E', axes = [False, True], kp = [0, 10], kd = [0, 0.001], full_init = False) #Board 0 controls the base rotation and shoulder joint
    board_1_driver = Odrive(usb_serial = '2083388E304E', axes = [True, True], kp = [10, 10], kd = [0.001, 0.001], full_init = False) #Board 1 controls the arm joint

    print('The motor driver has completed connection and calibration. It is ready to run.')

    #Try to connect to the Arduino serial port
    ser = serial.Serial('/dev/ttyACM2', 115200, timeout=1)
    while True:
        try:
            line = ser.readline().decode() #Wait for serial data to come in
            imu_orientations = line.split(", ") #Shoulder IMU x,y,z; elbow IMU x,y,z. Units are in degrees.

            if len(imu_orientations) >= 6:

                # Ignore first 10 messages
                if imu_msgs_recieved == 0:
                    print('Beginning calibration. Make sure arm is in the neutral position.')
                if imu_msgs_recieved < 100:
                    imu_msgs_recieved += 1
                    print(imu_orientations[0],'\t\t', imu_orientations[1], '\t\t', imu_orientations[5])
                elif imu_msgs_recieved == 100:
                    print('Almost done calibrating')
                    imu_msgs_recieved += 1
                else:

                    '''
                    ####### Option 1: Position Mapping #########
                    #Superlimb base pos is controller by horizontal rotation of the elbow, i.e. around the x axis.
                    if initial_elbow_horiz_angle == 1000:
                        initial_elbow_horiz_angle = float(imu_orientations[0])
                    superlimb_base_angle = max(-10, min(10, (-float(imu_orientations[0]) + initial_elbow_horiz_angle)/3))
                    
                    # Superlimb shoulder position is controlled by vertical swinging of the upper arm, i.e. rotation around the y axis
                    if initial_shoulder_vert_angle == 1000:
                        initial_shoulder_vert_angle = float(imu_orientations[1])
                    superlimb_shoulder_angle = max(-30, min(30, (float(imu_orientations[1]) - initial_shoulder_vert_angle)*2))

                    #Superlimb elbow position is controlled by elbow joint z (vertical) rotation
                    if initial_elbow_vert_angle == 1000:
                        initial_elbow_vert_angle = float(imu_orientations[5])
                    superlimb_elbow_angle = max(-30, min(30, (-float(imu_orientations[5]) + initial_elbow_vert_angle)*2))
                    ############################################
                    '''

                    ####### Option 2: Velocity Control with Deadband #######
                    #Superlimb base position is controller by horizontal rotation of the elbow, i.e. side to side, around the x axis
                    if initial_elbow_horiz_angle == 1000:
                        initial_elbow_horiz_angle = float(imu_orientations[0])
                    if abs(float(imu_orientations[0]) - initial_elbow_horiz_angle) > 5:
                        superlimb_base_angle = max(-10, min(10, superlimb_base_angle + (-float(imu_orientations[0]) + initial_elbow_horiz_angle)/40))
                        
                    # Superlimb shoulder position is controlled by vertical swinging of the upper arm, i.e. rotation around the y axis
                    if initial_shoulder_vert_angle == 1000:
                        initial_shoulder_vert_angle = float(imu_orientations[1])
                    if abs(float(imu_orientations[1]) - initial_shoulder_vert_angle) > 5:
                        superlimb_shoulder_angle = max(-30, min(30, superlimb_shoulder_angle + (float(imu_orientations[1]) - initial_shoulder_vert_angle)/10))

                    #Superlimb elbow position is controlled by elbow joint z rotation
                    if initial_elbow_vert_angle == 1000:
                        initial_elbow_vert_angle = float(imu_orientations[5])
                    if abs(float(imu_orientations[5]) - initial_elbow_vert_angle) > 5:
                        superlimb_elbow_angle = max(-30, min(30, superlimb_elbow_angle + (-float(imu_orientations[5]) + initial_elbow_vert_angle)/10))
                    ########################################################
                    

                    print('\t\t\t', imu_orientations[0],'\t\t', imu_orientations[1], '\t\t', imu_orientations[5])
                    print('Setpoints: \t\t',initial_elbow_horiz_angle,'\t\t', initial_shoulder_vert_angle,'\t\t', initial_elbow_vert_angle)
                    print('Superlimb base angle:', superlimb_base_angle, ',\tshoulder angle:', superlimb_shoulder_angle, ',\telbow angle:', superlimb_elbow_angle)
                    board_0_driver.pos_move([0, superlimb_base_angle])
                    #board_0_driver.pos_move([0, 15])
                    board_1_driver.pos_move([superlimb_elbow_angle, superlimb_shoulder_angle])

            else:
                print('Recieved garbled IMU data')

        except UnicodeDecodeError:
            print('Decoding error, trying again')

    # Testing code
    # board_0_driver.printErrorStates()
    # board_0_driver.pos_move([0, 10]) #base
    # board_1_driver.pos_move([imu_orientations[6]/90 - 20, 0]) #elbow, shoulder
    # time.sleep(5) #In seconds
    # board_0_driver.printErrorStates()
    # board_0_driver.pos_move([0, 0])
    # board_1_driver.pos_move([10, 0])
    # time.sleep(5)

    board_0_driver.set_idle([False, True])
    board_1_driver.set_idle([True, True])
