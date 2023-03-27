#Shell script to run OCTAVIUS platform

from led_control import led_initialize

#Initialize LED arrays
led_initialize("internal")
led_initialize("right_eye")
led_initialize("left_eye")
