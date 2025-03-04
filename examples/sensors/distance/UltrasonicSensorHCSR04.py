import time

from pirosity.core.sensors import UltrasonicSensorHCSR04

sensor = UltrasonicSensorHCSR04(23, 24, speed_of_sound=343.0)

try:
    while True:
        distance = sensor.measure()
        print(f"Distance: {round(distance * 100, 1)} cm")
        time.sleep(0.25)
except KeyboardInterrupt:
    print("Exiting program due to keyboard interruption.")
except Exception as exception:
    print(f"An error occurred: {exception}")
finally:
    sensor.reset()
