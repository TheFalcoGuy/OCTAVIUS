#Shell script to run OCTAVIUS platform

from led_control import led_initialize
from led_control import led_startup

import odrive
from odrive.enums import *

#Initialize LED arrays
led_initialize("internal")
led_startup("internal")
led_initialize("right_eye")
led_startup("right_eye")
led_initialize("left_eye")
led_startup("left_eye")

#Initialize ODrives
#Odrive S/Ns as follows:
# 20563882304E
# 2071388D304E
# 208637853548

#Assign ODrives wrt motor map
#Note: M0 -> Right, M1 -> Left
#Note: odrv0 -> J0, odrv1 -> J1, odrv2 -> J2

#Configure each motor