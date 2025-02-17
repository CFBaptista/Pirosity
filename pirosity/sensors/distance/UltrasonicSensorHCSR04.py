from dataclasses import dataclass
import time

from gpiozero import DigitalInputDevice, DigitalOutputDevice

from pirosity.utilities import clip


@dataclass(frozen=True)
class UltrasonicSensorHCSR04Data:
    """
    Data class to hold constants for the HC-SR04 sensor.

    Attributes
    ----------
    `distance_maximum` : float
        Maximum measurable distance by the sensor in meters.
    `distance_minimum` : float
        Minimum measurable distance by the sensor in meters.
    `duration_timeout` : float
        Duration after which the sensor times out if no echo signal is received.
    `duration_trigger` : float
        Duration required for TRIG pin is on in order to trigger the burst signal.
    `duration_wait` : float
        Duration to wait between consecutive measurements.
    """

    distance_maximum: float = 4.0
    distance_minimum: float = 0.02
    duration_timeout: float = 0.038
    duration_trigger: float = 0.00001
    duration_wait: float = 0.000002
    name: str = "HC-SR04"


class UltrasonicSensorHCSR04:
    """
    Class to interact with the HC-SR04 sensor.

    Attributes
    ----------
    `trigger_pin` : DigitalOutputDevice
        GPIO pin used to trigger the sensor.
    `echo_pin` : DigitalInputDevice
        GPIO pin used to receive the echo signal.
    `speed_of_sound` : float
        Speed of sound in m/s, default is 343.0.

    Examples
    --------
    Initialize the sensor and measure distance, assuming the TRIG pin is connected to GPIO 23 and
    the ECHO pin is connected to GPIO 24:

    >>> sensor = UltrasonicSensorHCSR04(23, 24)
    >>> distance = sensor.measure()
    >>> print(f"Distance: {round(distance, 3)} meters")
    """

    def __init__(self, trigger_pin: int, echo_pin: int, speed_of_sound: float = 343.0) -> None:
        self._trigger_pin = DigitalOutputDevice(trigger_pin)
        self._echo_pin = DigitalInputDevice(echo_pin)
        self._speed_of_sound = speed_of_sound

    def measure(self) -> float:
        """
        Measures the distance to an object in front of the HC-SR04 sensor. The minimum measurable
        distance is `self.minimum_distance` and the maximum measurable distance is
        `self.maximum_distance`.

        Returns
        -------
        float
            The measured distance in meters.
        """
        self._trigger_burst()
        transmit_end_time = self._transmit_end_time()
        receive_start_time = self._receive_start_time()
        raw_distance = self._compute_distance(transmit_end_time, receive_start_time)
        distance = clip(
            raw_distance,
            UltrasonicSensorHCSR04Data.distance_minimum,
            UltrasonicSensorHCSR04Data.distance_maximum,
        )
        return distance

    def _trigger_burst(self) -> None:
        self.trigger_pin.off()
        time.sleep(UltrasonicSensorHCSR04Data.duration_wait)
        self.trigger_pin.on()
        time.sleep(UltrasonicSensorHCSR04Data.duration_trigger)
        self.trigger_pin.off()

    def _transmit_end_time(self) -> float:
        while not self.echo_pin.is_active:
            continue
        transmitted_signal_end_time = time.time()
        return transmitted_signal_end_time

    def _receive_start_time(self) -> float:
        while self.echo_pin.is_active:
            continue
        received_signal_start_time = time.time()
        return received_signal_start_time

    def _compute_distance(self, transmit_end_time: float, receive_start_time: float) -> float:
        time_elapsed = receive_start_time - transmit_end_time
        distance = self.speed_of_sound * time_elapsed / 2
        return distance

    def reset(self) -> None:
        """
        Resets the sensor by closing the TRIG and ECHO pins.

        This method should be called to properly release the GPIO resources
        when the sensor is no longer needed.
        """
        self.trigger_pin.close()
        self.echo_pin.close()

    @property
    def trigger_pin(self) -> DigitalOutputDevice:
        """
        The GPIO pin used to trigger the sensor.

        Returns
        -------
        DigitalOutputDevice
            The GPIO pin used to trigger the sensor.
        """
        return self._trigger_pin

    @property
    def echo_pin(self) -> DigitalInputDevice:
        """
        The GPIO pin used to receive the echo signal.

        Returns
        -------
        DigitalInputDevice
            The GPIO pin used to receive the echo signal.
        """
        return self._echo_pin

    @property
    def speed_of_sound(self) -> float:
        """
        The speed used for the speed of sound in meters per second.

        Returns
        -------
        float
            The speed of sound in meters per second.
        """
        return self._speed_of_sound
