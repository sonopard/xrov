import vectormath
import math
import time
import collections
import mockbot as gb
import xbox as xb
from ServoMock import Servo

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

RovManualCmd = collections.namedtuple('RovManualCmd', ['x', 'y', 'x2', 'y2', 'phi', 'stop', 'mode_move', 'mode_actuator', 'act_close', 'act_open'], verbose=True)
RovWheelVelocities = collections.namedtuple('RovWheelVelocities', ['FL', 'FR', 'RL', 'RR'], verbose=True)
RovArmVelocities = collections.namedtuple('RovActuatorVelocities', ['BaseRZ', 'BaseRY', 'JointRY', 'FlangeRZ', 'FlangeRY', 'Tool1'], verbose=True)

servo_baseRZ = Servo(0x40)
servo_baseRY = Servo(0x40)
servo_jointRY = Servo(0x40)
servo_flangeRZ = Servo(0x40)
servo_flangeRY = Servo(0x40)
servo_tool = Servo(0x40)

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
        x2y2 = vectormath.Vector2(joystick.leftX(0),joystick.leftY(0))
        rot_r = joystick.rightTrigger()
        rot_l = joystick.leftTrigger()
        rot = rot_l - rot_r

        if xy.length < DZ_HARD:
            xy.x = 0
            xy.y = 0
        else:
            xy.length = (xy.length - DZ_SCALE) / (1 - DZ_SCALE)
        if xy.length > 1:
            xy = xy.normalize()
        if x2y2.length < DZ_HARD:
            x2y2.x = 0
            x2y2.y = 0
        else:
            x2y2.length = (x2y2.length - DZ_SCALE) / (1 - DZ_SCALE)
        if x2y2.length > 1:
            x2y2 = x2y2.normalize()

        stop = joystick.Y()
        mode_move = joystick.Start()
        mode_actuator = joystick.B()
        act_open = joystick.dpadUp()
        act_close = joystick.dpadDown()

        print("PS: len: {:06.4f} x: {:06.4f} y: {:06.4f} rot: {:06.4f} ".format(xy.length, xy.x, xy.y, rot))
        return RovManualMove(xy.x, xy.y, x2y2.x, x2y2.y, rot, stop, mode_move, mode_actuator, act_open. act_close)
    else:
        return RovManualMove(0, 0, 0, 0, 0, 1, 0, 0 , 0, 0) # stop


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

def gb_stop():
    gb.stop_all()

def servo_init():
    servo_baseRZ.set_low_limit(1.0)
    servo_baseRZ.set_high_limit(2.0)
    servo_baseRZ.move(100)
    servo_baseRZ.output_enable()

if USE_GB:
    gb_init()

class RovMode(Enum):
    STOP = 1
    MOVE = 2
    ACT = 3

rov_mode = RovMode.STOP

while run:
    cmd = joystick_read()
    if(cmd.stop):
       rov_mode = RovMode.STOP
    if(cmd.mode_move):
        gb_stop()
        rov_mode = RovMode.MOVE
    if(cmd.mode_act):
        rov_mode = RovMode.ACT

    if(rov_mode == RovMode.MOVE):
        wheelvels = rov_get_wheel_velocities(cmd)
        if USE_GB:
            gb_rov_move(wheelvels)

    if(rov_mode == RovMode.STOP):
        gb_rov_stop()

    if(rov_mode == RovMode.ACT):
        gb_rov_act(rov_get_act(cmd))

