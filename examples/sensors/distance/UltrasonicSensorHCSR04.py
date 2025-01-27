import time

from pirosity.sensors.distance import UltrasonicSensorHCSR04

sensor = UltrasonicSensorHCSR04(trigger_pin=16, echo_pin=18, speed_of_sound=343.0)

while True:
    try:
        distance = sensor.measure()
        print(f"Distance: {distance * 100} cm")
        time.sleep(0.25)
    except KeyboardInterrupt:
        print("Exiting program due to keyboard interruption.")
    except Exception as exception:
        print(f"An error occurred: {exception}")
    finally:
        sensor.reset()
        break
