from dataclasses import dataclass

from gpiozero import Serial


@dataclass
class InfraredSensorHX1838Data:
    name: str = "HX1838"


class InfraredSensorHX1838:
    def __init__(self, receive_pin: int, baudrate=9600):
        self._sensor = Serial(rx_pin=receive_pin, baudrate=baudrate)
        self._baudrate = baudrate

    def read_value(self):
        while True:
            value = self._sensor.read()
            print(value)

    @property
    def baudrate(self) -> int:
        return self._baudrate
