#Shell script to run OCTAVIUS platform

from led_control import interior eyes

#import odrive
import threading
#from odrive.enums import *

if __name__ = "__main__":

    #Initialize LED arrays
    interior()
    eyes_light = threading.Thread(target=eyes)

    #Initialize ODrives
    #Odrive S/Ns as follows:
    # 20563882304E
    # 2071388D304E
    # 208637853548

    #Assign ODrives wrt motor map
    #Note: M0 -> Right, M1 -> Left
    #Note: odrv0 -> J0, odrv1 -> J1, odrv2 -> J2

    #Configure each motor

    #Execute Threads
    eyes_light.start()

    eyes_light.join()