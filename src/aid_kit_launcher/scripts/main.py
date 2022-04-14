#!/usr/bin/python2.7
import rospy
import RPi.GPIO as GPIO
from time import sleep
from std_msgs.msg import Bool
from std_srvs.srv import SetBoolRequest,SetBoolResponse,SetBool

#Servo works in range 2.5-12.5 duty which means 0-180 degrees

global duty #Pwm duty cycle
global pwm #Pwm instance
duty=2.5 #Set servo 0 position duty value

GPIO.setmode(GPIO.BOARD) #This mode let us using board pinout numerate
GPIO.setwarnings(False) #XD 
print("GPIO set mode") 
GPIO.setup(33, GPIO.OUT) #Seting up pwm pin as gpio out
pwm = GPIO.PWM(33, 50) #Creating pwm instance
pwm.start(duty) #Start pwm with 0 position value 

#Service responsible for handling whether servo is open or closed
#True - Open, False - Closed
def move_servo(req):
	global duty
	global pwm
	try:
		if req.data:
			duty = 12.0		
		else:
			duty = 2.5
		pwm.ChangeDutyCycle(duty) #Set new Pwm duty cycle
		
		return True, "Successful operation"
	except ValueError:
		return False, ValueError
	

rospy.init_node('servo_server', anonymous=True) #Initialize ros node
s = rospy.Service('move',SetBool ,move_servo) #Create rosservice instance with callback
print ("Servo Service is ready")
rospy.spin() #Wait for ctrl+C
pwm.stop() #Stop Pwm signal
GPIO.cleanup() #Clean GPIO
