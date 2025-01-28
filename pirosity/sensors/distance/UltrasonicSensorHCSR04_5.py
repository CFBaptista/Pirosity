from dataclasses import dataclass
import time

from gpiozero import DigitalInputDevice, DigitalOutputDevice


@dataclass
class HCSR04Data:
    duration_timeout: float = 0.038
    duration_trigger: float = 0.00001
    duration_wait: float = 0.000002


class UltrasonicSensorHCSR04_5:
    def __init__(self, pin_trigger: int, pin_echo: int, speed_of_sound: float = 343.0) -> None:
        self._pin_trigger = DigitalOutputDevice(pin_trigger)
        self._pin_echo = DigitalInputDevice(pin_echo)
        self._speed_of_sound = speed_of_sound

    def measure(self) -> float:
        self._trigger_burst()
        transmit_end_time = self._transmit_end_time()
        receive_start_time = self._receive_start_time()
        distance = self._compute_distance(transmit_end_time, receive_start_time)
        return distance

    def _trigger_burst(self) -> None:
        self.pin_trigger.off()
        time.sleep(HCSR04Data.duration_wait)
        self.pin_trigger.on()
        time.sleep(HCSR04Data.duration_trigger)
        self.pin_trigger.off()

    def _transmit_end_time(self) -> float:
        while not self.pin_echo.is_active:
            continue
        transmitted_signal_end_time = time.time()
        return transmitted_signal_end_time

    def _receive_start_time(self) -> float:
        while self.pin_echo.is_active:
            continue
        received_signal_start_time = time.time()
        return received_signal_start_time

    def _compute_distance(self, transmit_end_time: float, receive_start_time: float) -> float:
        time_elapsed = receive_start_time - transmit_end_time
        distance = self.speed_of_sound * time_elapsed / 2
        return distance

    def reset(self) -> None:
        self.pin_trigger.close()
        self.pin_echo.close()

    @property
    def pin_trigger(self) -> DigitalOutputDevice:
        return self._pin_trigger

    @property
    def pin_echo(self) -> DigitalInputDevice:
        return self._pin_echo

    @property
    def speed_of_sound(self) -> float:
        return self._speed_of_sound
