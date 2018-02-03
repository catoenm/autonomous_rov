# Linux
# Prerequisite: xboxdrv
# Use: $ sudo teleop.py
import binascii
import serial
import time
import xbox

ARDUINO_COM_PORT = '/dev/ttyACM0' # This changes. Get the value from the arduino ide
SERIAL_BAUD_RATE = 9600
SERIAL_TIMEOUT_SEC = 1
SERIAL_START_BYTE = 0xFF
SERIAL_GRADIENT_MIN_VALUE = 0x00
SERIAL_GRADIENT_MAX_VALUE = 0xFE


# Format floating point number to string format -x.xxx
def fmtFloat(n):
    return '{:6.3f}'.format(n)

# Maps -1 to 1 into 0 to 254
def map_float(float_val):
    return ((int) (127 * float_val + 127)) & 0xFF

# Protocol:
# 0xFF = start byte
# 0 - 0xFE: Left joystick y value
# 0 - 0xFE: Right joystick x value
# 0 - 0xA: Other button mappings

with serial.Serial(ARDUINO_COM_PORT, SERIAL_BAUD_RATE, timeout=SERIAL_TIMEOUT_SEC) as ser:
    print 'Attempting to connect to 360 controller'
    controller = xbox.Joystick()
    print 'Connected'

    while not controller.Back():
        byte_arr = bytearray([])
        if not controller.connected():
            print 'Controller not connected'

        if (ser.in_waiting > 0):
            try:
                print ser.read(ser.in_waiting)
            except Error as e:
                print e
                pass

        byte_arr.append(SERIAL_START_BYTE)
        byte_arr.append(map_float(controller.leftY()))
        byte_arr.append(map_float(controller.rightX()))

        if controller.Y():
            byte_arr.append(1)
        elif controller.B():
            byte_arr.append(2)
        elif controller.A():
            byte_arr.append(3)
        elif controller.X():
            byte_arr.append(4)
        elif controller.dpadUp():
            byte_arr.append(5)
        elif controller.dpadRight():
            byte_arr.append(6)
        elif controller.dpadDown():
            byte_arr.append(7)
        elif controller.dpadLeft():
            byte_arr.append(8)
        elif controller.Guide():
            byte_arr.append(9)
        elif controller.Start():
            byte_arr.append(10)
        else:
            byte_arr.append(0)

        print binascii.hexlify(byte_arr)
        ser.write(byte_arr)
        ser.flush()

        time.sleep(0.1)

    if not controller.connected():
        print 'Controller not connected'

    controller.close()
    print 'Exiting'
