import RPi.GPIO as GPIO
from time import sleep
GPIO.setmode(GPIO.BOARD)
GPIO.setup(03, GPIO.OUT) # Edit this depending on which Pin is connected
pwm = GPIO.PWM(03, 50)
pwm.start(0)

'''
To set the angle of the servo, we need to send a specific signal to it. 
This can differ from servo to servo, as normally it's from 2.5-12.5%, and on the ones I'm using it's 2-12%. 
Regardless, it will be a 10% window, so to calculate the duty cycle for your desired angle, divide by 18, 
Then add the lowest available value, in this case 2.
So, for 90 degrees, divide by 18, which is 5, then add 2, and you get 7. So on this servo 7% duty is 90 degrees.
As you can see, this math is not very friendly and would be tedious to do every time you wanted to set an angle, 
so in order to simplify that we're going to write a function in Python that does the math automatically then sets the angle.
'''
def SetAngle(angle):
    duty = angle /18 + 2
    print("Duty Calculated")
    GPIO.output(03, True) # Turns on the pin for output
    pwm.ChangeDutyCycle(duty)  # Changes duty cycle to match what we calculated
    sleep(1)
    GPIO.output(03, False) # Turns off the pin
    pwm.ChangeDutyCycle(0) # Changes duty to back to 0 so we arn't continously sending inputs to the servo
    print("Angle Set")

SetAngle(0)
print("Angle Set to 0")
sleep(5)
SetAngle(90)
print("Angle Set to 90")
sleep(5)
SetAngle(180)
print("Angle Set to 180")