import time
from rpi_ws281x import *
import argparse
import board
import neopixel
pixels = neopixel.NeoPixel(board.D17, 11)

def led_initialize(location):
    #LED strip configuration
    LED_COUNT = 11
    LED_PIN = 17
    LED_FREQ_HZ = 800000
    LED_DMA = 10
    LED_BRIGHTNESS = 100
    LED_INVERT = False
    LED_CHANNEL = 0
    if location == "internal":
        LED_COUNT = 11
        LED_PIN = 17
        print("Internal LED Initialized")
    if location == "right_eye" or "left_eye":
        LED_COUNT = 5
    if location == "right_eye":
        LED_PIN = 27
        print("Right Eye LED Initialized")
    if location == "left_eye":
        LED_PIN = 22
        print("Left Eye LED Initialized")

def colorWipe(strip, color, wait_ms=50):
    """Wipe color across display a pixel at a time."""
    for i in range(strip.numPixels()):
        strip.setPixelColor(i, color)
        strip.show()
        time.sleep(wait_ms/1000.0)

def theaterChase(strip, color, wait_ms=50, iterations=10):
    """Movie theater light style chaser animation."""
    for j in range(iterations):
        for q in range(3):
            for i in range(0, strip.numPixels(), 3):
                strip.setPixelColor(i+q, color)
            strip.show()
            time.sleep(wait_ms/1000.0)
            for i in range(0, strip.numPixels(), 3):
                strip.setPixelColor(i+q, 0)

def led_startup(location):
    if __name__ == '__main__':
        pixels.fill((0,255,0))
#        # Process arguments
#        parser = argparse.ArgumentParser()
#        parser.add_argument('-c', '--clear', action='store_true', help='clear the display on exit')
#        args = parser.parse_args()
#
#        # Create NeoPixel object with appropriate configuration.
#        strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
#        # Intialize the library (must be called once before other functions).
#        strip.begin()
#
#        print ('Press Ctrl-C to quit.')
#        if not args.clear:
#            print('Use "-c" argument to clear LEDs on exit')
#
#        try:
#
 #           while True:
 #               theaterChase(strip, Color(  0,   0, 255))  # Green theater chase
 #               colorWipe(strip, Color(0, 0, 255))  # Green wipe
 #               
 #       except KeyboardInterrupt:
 #           if args.clear:
 #               colorWipe(strip, Color(0,0,0), 10)