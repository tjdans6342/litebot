#Robot Class package
#/home/jetson/Setup/venv/lib/python3.8/site-packages/tiki/mini/__init__.py

from tiki.mini import TikiMini
tiki = TikiMini()
import time

# #Battery Check, return voltage(V)
# tiki.get_battery_voltage()

# #Battery Checck, return current(A)
# tiki.get_current()

# # #OLED Display
# tiki.log("HELLO!") 


#OLED Log Clear
tiki.log_clear()


# #set_led(direction, index, R, G, B) 
# #direction : 0(top 16-bit LED Strip), 1(left 8-bit LED), 2(right 8-bit LED)
# #index : LED number(0~15 for top, 0~7 for left/right)
# #R,G,B : 0~255 brightness
# tiki.set_led(0, 1, 10, 0, 0)


# #Turn on/off all 16-bit Red LED.
# for i in range(16):
#     tiki.set_led(0, i, 0, 10, 0)
# time.sleep(2)
# for i in range(16):
#     tiki.set_led(0, i, 0, 0, 0)


# # Acceleration Sensor Values
# # X-axis, Y-axis, Z-axis
# tiki.get_imu() 

# Motor PWM mode setting
# PWM for simple speed control
# rpm : -127 ~ 127
tiki.set_motor_mode(tiki.MOTOR_MODE_PWM)

# tiki.set_motor_power(tiki.MOTOR_RIGHT, 20)  #-127~127



# tiki.stop()
# 36.5 : 127
# 31 : 100
# 18 : 63.5

tiki.forward(63.5)
time.sleep(1)

tiki.stop()
# tiki.backward(20)
# time.sleep(2)
# tiki.clockwise(20)
# time.sleep(2)
# tiki.counter_clockwise(20)
# time.sleep(2)
# tiki.stop()