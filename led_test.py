import board
import neopixel
import time
import threading

Num_Int = 12
Num_Eye = 6

def interior():
    while True:
        for k in range(0,10,1):
            pixels_int = neopixel.NeoPixel(board.D12, Num_Int, auto_write=False, brightness=0.1*k)
            for i in range(Num_Int-1):
                pixels_int[i] = ((0,0,255))
                i = i+1
            pixels_int.show()
            time.sleep(0.1)
        for k in range(10,0,-1):
            pixels_int = neopixel.NeoPixel(board.D12, Num_Int, auto_write=False, brightness=0.1*k)
            for i in range(Num_Int-1):
                pixels_int[i] = ((0,0,255))
                i = i+1
            pixels_int.show()
            time.sleep(0.1)

def eyes():
    pixels = neopixel.NeoPixel(board.D18, Num_Eye)
    while True:
        for i in range(Num_Eye-1):
            pixels[i] = ((0,0,255))
            time.sleep(0.1)
        for i in range(Num_Eye-1):
            pixels[i] = ((0,0,0))
            time.sleep(0.1)

if __name__ =="__main__":
    int_light = threading.Thread(target=interior)
    eye_light = threading.Thread(target=eyes)

    int_light.start()
    eye_light.start()

    int_light.join()
    eye_light.join()