from arduino import Arduino
import time

b = Arduino('/dev/cu.usbmodemFD121')
pin = 13

#declare output pins as a list/tuple
b.output([pin])

for _ in xrange(4):
    b.setHigh(pin)
    time.sleep(1)
    print b.getState(pin)
    b.setLow(pin)
    print b.getState(pin)
    time.sleep(1)

b.close()