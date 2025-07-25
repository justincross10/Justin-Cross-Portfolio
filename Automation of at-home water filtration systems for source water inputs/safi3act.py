from datetime import datetime, time
import RPi.GPIO as GPIO  # Assuming you are using Raspberry Pi GPIO

# Set up GPIO pin (replace 18 with your actual GPIO pin)
green = 17
red = 27
blue = 22

GPIO.setmode(GPIO.BCM)
GPIO.setup(green, GPIO.OUT)
GPIO.setup(red, GPIO.OUT)
GPIO.setup(blue, GPIO.OUT)

# Define the start and end times
start_time1 = time(11, 35, 0)
end_time1 = time(11, 35, 30)

start_time2 = time(11, 35, 30)
end_time2 = time(11, 36, 0)

while True:
    current_time = datetime.now().time()

    # Print the current time
    print(f"Current Time: {current_time}")

    # Check if the current time is within the specified interval
    if start_time1 <= current_time <= end_time1:
        GPIO.output(green, GPIO.LOW)
        GPIO.output(blue, GPIO.LOW)
        GPIO.output(red, GPIO.HIGH)
    elif start_time2 <= current_time <= end_time2:
        GPIO.output(green, GPIO.LOW)
        GPIO.output(red, GPIO.LOW)
        GPIO.output(blue, GPIO.HIGH)
    else:
        GPIO.output(green, GPIO.HIGH)
        GPIO.output(red, GPIO.HIGH)
        GPIO.output(blue, GPIO.HIGH)