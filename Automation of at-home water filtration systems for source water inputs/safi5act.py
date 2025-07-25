from datetime import datetime, time
import RPi.GPIO as GPIO  # Assuming you are using Raspberry Pi GPIO

# Set up GPIO pin (replace 18 with your actual GPIO pin)
green = 17
red1 = 27
blue1 = 22
red2 = 24
blue2 = 23

GPIO.setmode(GPIO.BCM)
GPIO.setup(green, GPIO.OUT)
GPIO.setup(red1, GPIO.OUT)
GPIO.setup(blue1, GPIO.OUT)
GPIO.setup(red2, GPIO.OUT)
GPIO.setup(blue2, GPIO.OUT)

# Define the start and end times
start_time1 = time(12, 44, 10)
end_time1 = time(12, 44, 20)

start_time2 = time(12, 44, 20)
end_time2 = time(12, 44, 30)

start_time3 = time(12, 44, 30)
end_time3 = time(12, 44, 40)

start_time4 = time(12, 44, 40)
end_time4 = time(12, 44, 50)

while True:
    current_time = datetime.now().time()

    # Print the current time
    print(f"Current Time: {current_time}")

    # Check if the current time is within the specified interval
    if start_time1 <= current_time <= end_time1:
        GPIO.output(green, GPIO.LOW)
        GPIO.output(red1, GPIO.LOW)
        GPIO.output(blue1, GPIO.HIGH)
        GPIO.output(red2, GPIO.HIGH)
        GPIO.output(blue2, GPIO.HIGH)
    elif start_time2 <= current_time <= end_time2:
        GPIO.output(green, GPIO.LOW)
        GPIO.output(red1, GPIO.HIGH)
        GPIO.output(blue1, GPIO.LOW)
        GPIO.output(red2, GPIO.HIGH)
        GPIO.output(blue2, GPIO.HIGH)
    elif start_time3 <= current_time <= end_time3:
        GPIO.output(green, GPIO.LOW)
        GPIO.output(red1, GPIO.HIGH)
        GPIO.output(blue1, GPIO.HIGH)
        GPIO.output(red2, GPIO.HIGH)
        GPIO.output(blue2, GPIO.LOW)
    elif start_time4 <= current_time <= end_time4:
        GPIO.output(green, GPIO.LOW)
        GPIO.output(red1, GPIO.HIGH)
        GPIO.output(blue1, GPIO.HIGH)
        GPIO.output(red2, GPIO.LOW)
        GPIO.output(blue2, GPIO.HIGH)
    else:
        GPIO.output(green, GPIO.HIGH)
        GPIO.output(red1, GPIO.HIGH)
        GPIO.output(blue1, GPIO.HIGH)
        GPIO.output(red2, GPIO.HIGH)
        GPIO.output(blue2, GPIO.HIGH)
