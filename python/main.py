import time
import collections
import gertbot as gb
import xbox as xb

USE_GB = True

## hardware constants
# gertbot board identifier
BOARD = 1
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
        return RovManualMove(joystick.rightX(), joystick.rightY(), -joystick.leftX() / 12)
    else:
        return RovManualMove(0, 0, 0)


def rov_get_wheel_velocities(movedata: RovManualMove):
    #  https://github.com/neobotix/neo_driver/blob/indigo_dev/neo_platformctrl_mecanum/common/src/Mecanum4WKinematics.cpp
    velocities = RovWheelVelocities(
        (2 / WHEEL_DIAMETER * (movedata.y + movedata.x - (TREAD_LENGTH + WHEELBASE_LENGTH) / 2 * movedata.phi)),
        (2 / WHEEL_DIAMETER * (movedata.y - movedata.x + (TREAD_LENGTH + WHEELBASE_LENGTH) / 2 * movedata.phi)),
        (2 / WHEEL_DIAMETER * (movedata.y - movedata.x - (TREAD_LENGTH + WHEELBASE_LENGTH) / 2 * movedata.phi)),
        (2 / WHEEL_DIAMETER * (movedata.y + movedata.x + (TREAD_LENGTH + WHEELBASE_LENGTH) / 2 * movedata.phi)))
    print(
        "Vel: FL: {:06.4f} FR: {:06.4f} RL: {:06.4f} RR: {:06.4f}".format(velocities.FL, velocities.FR, velocities.RL,
                                                                            velocities.RR))
    return velocities


def gb_rov_move(wheelvel: RovWheelVelocities):
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


while run:
    time.sleep(1)
    if USE_GB:
        gb_init()
    wheelvels = rov_get_wheel_velocities(joystick_read())
    if USE_GB:
        gb_rov_move(wheelvels)
