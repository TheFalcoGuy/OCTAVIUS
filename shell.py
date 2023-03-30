#Shell script to run OCTAVIUS platform

from led_control import interior
from led_control import eyes
from OdrivePythonController_Superlimb import Odrive
import threading

def motor_cmd():
    #board_0_driver.pos_move([0,0])
    #board_1_driver.pos_move([0,50])

if __name__ == "__main__":

    #Initialize LED arrays
    interior()
    eyes_light = threading.Thread(target=eyes)

    #Initialize ODrives
    #Odrive S/Ns as follows:
    # 20563882304E JOINT 0
    # 2071388D304E
    # 208637853548
    board_0_driver = Odrive(usb_serial = '20563882304E', axes = [True, True], kp = [10, 10], kd = [0.001, 0.001], full_init = True)
    board_1_driver = Odrive(usb_serial = '2071388D304E', axes = [True, True], kp = [10, 10], kd = [0.001, 0.001], full_init = True)
    board_2_driver = Odrive(usb_serial = '208637853548', axes = [True, True], kp = [10, 10], kd = [0.001, 0.001], full_init = True)

    print('The motor driver has completed connection and calibration. It is ready to run.')

    control = threading.Thread(target=motor_cmd)

    #Assign ODrives wrt motor map
    #Note: M0 -> Right, M1 -> Left
    #Note: odrv0 -> J0, odrv1 -> J1, odrv2 -> J2

    #Execute Threads
    eyes_light.start()
    control.start()

    eyes_light.join()
    control.join()