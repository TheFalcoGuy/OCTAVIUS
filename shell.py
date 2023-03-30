#Shell script to run OCTAVIUS platform

from led_control import interior
from led_control import eyes
from OdrivePythonController_Superlimb import Odrive
import threading

if __name__ == "__main__":

    #Initialize LED arrays
    interior()
    eyes_light = threading.Thread(target=eyes)

    #Initialize ODrives
    #Odrive S/Ns as follows:
    # 20563882304E
    # 2071388D304E
    # 208637853548
    board_0_driver = Odrive(usb_serial = '20563882304E', axes = [True, True], kp = [0.001, 10], kd = [0.001, 0.001], full_init = True)
    board_1_driver = Odrive(usb_serial = '2071388D304E', axes = [True, True], kp = [10, 10], kd = [0.001, 0.001], full_init = False)
    board_2_driver = Odrive(usb_serial = '208637853548', axes = [True, True], kp = [10, 10], kd = [0.001, 0.001], full_init = False)

    print('The motor driver has completed connection and calibration. It is ready to run.')

    #Assign ODrives wrt motor map
    #Note: M0 -> Right, M1 -> Left
    #Note: odrv0 -> J0, odrv1 -> J1, odrv2 -> J2

    #Configure each motor

    #Execute Threads
    eyes_light.start()

    eyes_light.join()