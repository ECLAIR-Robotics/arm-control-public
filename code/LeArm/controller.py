import serial
import time
from collections.abc import Sequence
from dataclasses import dataclass


HEADER = '>>'  # header to start messages to arduino with
SERVO_MOVE_CODE = '$'  # command code to move servo
SERVO_READ_CODE = '#'  # command code to read from servo
VOLT_READ_CODE = '%'  # command code to read voltage


def clamp(x, low, high):
    if high < low:
        low, high = high, low

    return min(max(low, x), high)


@dataclass
class Servo:
    def __init__(self, servo_min: int, servo_max: int, angular_range: int,
                 max_speed: float = 0.09, init_position: float = 0.5, synced: bool = True):
        if servo_min >= servo_max:
            raise Exception('Servo min must be less than servo max!')

        self.min = servo_min
        self.max = servo_max
        self.range = servo_max - servo_min
        self.max_speed = max_speed
        self._delta = 0
        self._synced = synced
        self._angular_range = angular_range
        self._position = init_position

    @property
    def delta(self):
        return self._delta

    @property
    def synced(self):
        return self._synced

    @synced.setter
    def synced(self, new_state):
        self._synced = new_state

    @property
    def position_raw(self):
        return round(self.position * self.range + self.min)

    @position_raw.setter
    def position_raw(self, new_pos_raw):
        self.position = (new_pos_raw - self.min) / self.range

    @property
    def angle(self):
        return (self.position - 0.5) * self._angular_range

    @angle.setter
    def angle(self, new_angle):
        self.position = (new_angle / self._angular_range) + 0.5

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, new_pos):
        new_pos = clamp(new_pos, 0, 1)

        self._delta = new_pos - self.position
        self._position = new_pos
        self._synced = False


class Controller(Sequence):
    BAUDRATE = 9600
    TIMEOUT = .1
    SERVO_NAMES = {  # lol these names are kinda wack but idk what else to call them for now
        'base': 6,
        'shoulder': 5,
        'elbow': 4,
        'wrist': 3,
        'hand': 2,
        'claw': 1,
    }
    HOME_POSITION = 0.5  # position to set every servo to when homed
    HOME_TIME = 5000  # how long it takes to go to home positions when going to home

    def __init__(self, home=False, port='COM3', debug=False):
        self._arduino = serial.Serial(port=port, baudrate=self.BAUDRATE, timeout=self.TIMEOUT)
        self._servos = {
            1: Servo(servo_min=600, servo_max=1380, angular_range=180, max_speed=2),  # actual max is 1800, be careful
            2: Servo(servo_min=400, servo_max=2500, angular_range=180, max_speed=1),
            3: Servo(servo_min=450, servo_max=2600, angular_range=180, max_speed=1),
            4: Servo(servo_min=400, servo_max=2600, angular_range=180, max_speed=1),
            5: Servo(servo_min=400, servo_max=2500, angular_range=180, max_speed=0.5),
            6: Servo(servo_min=400, servo_max=2500, angular_range=180, max_speed=0.5),
        }
        self._debug = debug

        while True:  # wait until arduino gives ready signal
            ready = self._arduino.read()

            if ready:
                break

            time.sleep(self.TIMEOUT)

        if home:
            self.home()

    def home(self, action_time=HOME_TIME):
        for servo_id, servo in self._servos.items():
            self._setPosition(servo_id, self.HOME_POSITION)

        self._updateArm(action_time)

    def getPosition(self, servo_id):
        return self[servo_id]

    def setPosition(self, servo_id, action_time=1000, position=0.5):
        self._setPosition(servo_id, position)  # sets position logically
        self._updateArm(action_time)  # actually moves arm

    def setPositions(self, action_time=1000, **movements):
        for servo_id, position in movements.items():
            self._setPosition(servo_id, position)

        self._updateArm(action_time)

    def setPositionRaw(self, servo_id, action_time=1000, position_raw=1380):
        self._setPositionRaw(servo_id, position_raw)
        self._updateArm(action_time)

    def setPositionsRaw(self, action_time=1000, **movements_raw):
        for servo_id, position_raw in movements_raw.items():
            self._setPositionRaw(servo_id, position_raw)

        self._updateArm(action_time)

    def setAngle(self, servo_id, action_time=1000, angle=0):
        self._setAngle(servo_id, angle)
        self._updateArm(action_time)

    def setAngles(self, action_time=1000, **movements):
        for servo_id, angle in movements.items():
            self._setAngle(servo_id, angle)

        self._updateArm(action_time)

    def readPositions(self, *servo_ids):
        message = f'{len(servo_ids)}'
        for servo_id in servo_ids:
            message += f'{servo_id}'

        message = f'{SERVO_READ_CODE}{message}'
        self._write(message)
        time.sleep(0.05)
        return self._arduino.read_until('Q')

    def readVoltage(self):
        message = f'{VOLT_READ_CODE}'
        self._write(message)
        time.sleep(0.05)
        return self._arduino.read_until('Q')

    def _write(self, x):
        x = HEADER + x
        if self._debug:
            print(f'sent: {bytes(x, "utf-8")}')
        self._arduino.write(bytes(x, 'utf-8'))

    def _read(self):
        return self._arduino.read_until('Q')

    def _updateArm(self, action_time=1000):  # sends command to update all unsynced servos
        message = ''
        unsynced = 0
        for servo_id, servo in self._servos.items():
            if servo.synced:
                continue

            min_time = abs(servo.delta) / (servo.max_speed / 1000)  # max speed is in rot/sec, we want rot/ms
            if min_time > action_time:
                action_time = min_time
                print(f'warning: servo {servo_id} too fast, slowed movement to {action_time} ms')

            message += f'{servo_id}{servo.position_raw:06}'  # [0-6][position padded to 6 characters]
            unsynced += 1
            servo.synced = True

        message = f'{SERVO_MOVE_CODE}{unsynced}{message}{action_time:016}'  # code, then count, then data, then time
        self._write(message)

        time.sleep(action_time / 1000)  # wait until this command is finished before sending next one

    def _setPosition(self, servo_id, position):
        servo_id = self._validateId(servo_id)

        self._servos[servo_id].position = position

    def _setAngle(self, servo_id, angle):
        servo_id = self._validateId(servo_id)

        self._servos[servo_id].angle = angle

    def _setPositionRaw(self, servo_id, position_raw):
        servo_id = self._validateId(servo_id)

        self._servos[servo_id].position_raw = position_raw

    def _validateId(self, servo_id):
        if isinstance(servo_id, str):
            if servo_id not in self.SERVO_NAMES:
                raise Exception(f'{servo_id} is not a valid servo name')

            servo_id = self.SERVO_NAMES[servo_id]

        if servo_id not in self._servos:
            raise Exception(f'{servo_id} is not a valid servo id')

        return servo_id

    def __getitem__(self, i):
        i = self._validateId(i)

        return self._servos[i].position

    def __setitem__(self, key, value):
        self._setPosition(key, value)
        self._updateArm()

    def __len__(self):
        return len(self._servos)
