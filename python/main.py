import vectormath
import math
import time
import collections
import gertbot as gb
import xbox as xb

USE_GB = True

## hardware constants
# gertbot board identifier
BOARD = 0
# wheel channels
FL = 2
FR = 3
RL = 0
RR = 1
# acceleration ramp
PWM_FREQ = 5000
PWM_DC_MUL = 0.7

## platform constants
WHEEL_DIAMETER = 6
TREAD_LENGTH = 19
WHEELBASE_LENGTH = 18

DZ_HARD = 0.3
DZ_SCALE = 0.1

joystick = xb.Joystick()
joystick_connected = False
run = True

RovManualMove = collections.namedtuple('RovManualMove', ['x', 'y', 'phi'], verbose=True)
RovWheelVelocities = collections.namedtuple('RovWheelVelocities', ['FL', 'FR', 'RL', 'RR'], verbose=True)


def joystick_read():
    global joystick_connected
    global run
    if joystick.Back():
        run = False
    if joystick_connected != joystick.connected():
        joystick_connected = joystick.connected()
        print("Joystick state changed: {}\n".format(joystick_connected))
    if joystick_connected:
        xy = vectormath.Vector2(joystick.rightX(0),joystick.rightY(0))
        rot_r = joystick.rightTrigger()
        rot_l = joystick.leftTrigger()
        rot = rot_r - rot_l
        if xy.length < DZ_HARD:
            xy.x = 0
            xy.y = 0
        else:
            xy.length = (xy.length - DZ_SCALE) / (1 - DZ_SCALE)
        if xy.length > 1:
            xy = xy.normalize()
        print("PS: len: {:06.4f} x: {:06.4f} y: {:06.4f} rot: {:06.4f} ".format(xy.length, xy.x, xy.y, rot))
        return RovManualMove(xy.x, xy.y, rot)
    else:
        return RovManualMove(0, 0, 0)

def rov_get_wheel_velocities(movedata: RovManualMove):
    #  https://github.com/neobotix/neo_driver/blob/indigo_dev/neo_platformctrl_mecanum/common/src/Mecanum4WKinematics.cpp
    velocities = RovWheelVelocities(
        (2 / WHEEL_DIAMETER * (movedata.y + movedata.x - (TREAD_LENGTH + WHEELBASE_LENGTH) / 2 * (movedata.phi/12))),
        (2 / WHEEL_DIAMETER * (movedata.y - movedata.x + (TREAD_LENGTH + WHEELBASE_LENGTH) / 2 * (movedata.phi/12))),
        (2 / WHEEL_DIAMETER * (movedata.y - movedata.x - (TREAD_LENGTH + WHEELBASE_LENGTH) / 2 * (movedata.phi/12))),
        (2 / WHEEL_DIAMETER * (movedata.y + movedata.x + (TREAD_LENGTH + WHEELBASE_LENGTH) / 2 * (movedata.phi/12))))
    print(
        "Vel: FL: {:06.4f} FR: {:06.4f} RL: {:06.4f} RR: {:06.4f}".format(velocities.FL, velocities.FR, velocities.RL,
                                                                            velocities.RR))
    return velocities


def gb_rov_move(wheelvel: RovWheelVelocities):
    gb.read_error_status(BOARD)
    # different directions.. therefore very verbose. >.<
    gb.pwm_brushed(BOARD, FL, PWM_FREQ, abs(wheelvel.FL) * 100 * PWM_DC_MUL)
    if wheelvel.FL < 0:
        gb.move_brushed(BOARD, FL, 1)
    elif wheelvel.FL > 0:
        gb.move_brushed(BOARD, FL, 2)
    else:
        gb.move_brushed(BOARD, FL, 0)

    gb.pwm_brushed(BOARD, FR, PWM_FREQ, abs(wheelvel.FR) * 100 * PWM_DC_MUL)
    if wheelvel.FR < 0:
        gb.move_brushed(BOARD, FR, 2)
    elif wheelvel.FR > 0:
        gb.move_brushed(BOARD, FR, 1)
    else:
        gb.move_brushed(BOARD, FR, 0)

    gb.pwm_brushed(BOARD, RL, PWM_FREQ, abs(wheelvel.RL) * 100 * PWM_DC_MUL)
    if wheelvel.RL < 0:
        gb.move_brushed(BOARD, RL, 1)
    elif wheelvel.RL > 0:
        gb.move_brushed(BOARD, RL, 2)
    else:
        gb.move_brushed(BOARD, RL, 0)

    gb.pwm_brushed(BOARD, RR, PWM_FREQ, abs(wheelvel.RR) * 100 * PWM_DC_MUL)
    if wheelvel.RR < 0:
        gb.move_brushed(BOARD, RR, 2)
    elif wheelvel.RR > 0:
        gb.move_brushed(BOARD, RR, 1)
    else:
        gb.move_brushed(BOARD, RR, 0)


def gb_init():
    gb.open_uart(1)
    gb.set_mode(BOARD, FL, gb.MODE_BRUSH)
    gb.set_mode(BOARD, FR, gb.MODE_BRUSH)
    gb.set_mode(BOARD, RL, gb.MODE_BRUSH)
    gb.set_mode(BOARD, RR, gb.MODE_BRUSH)

if USE_GB:
    gb_init()

while run:
    wheelvels = rov_get_wheel_velocities(joystick_read())
    if USE_GB:
        gb_rov_move(wheelvels)
